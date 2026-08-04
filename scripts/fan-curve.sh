#!/usr/bin/env bash
# Temperature-driven fan control for the ROCK 5B.
#
# Radxa's stock configuration leaves the pwm-fan cooling device pinned at
# maximum regardless of temperature — the board idles at 37 C with the fan at
# full. This replaces that with a curve: silent when idle, proportional in the
# middle, full only when it is actually needed.
#
# Why this matters on a robot rather than a desktop:
#   * A fan 10 cm from a microphone array is a permanent noise floor under
#     every ASR request.
#   * Full-speed fan power is real battery minutes on a 77 Wh pack.
#   * A robot that whines constantly is unpleasant to share a room with, and
#     this one is meant to play with a cat.
#
# SAFETY: if this daemon dies, the fan must fail to FULL, never off. The
# systemd unit enforces that with ExecStopPost, and Restart=always brings it
# back. Cooking an RK3588 because a shell script exited is not an acceptable
# failure mode.
set -euo pipefail

HWMON=/sys/class/hwmon/hwmon8          # "pwmfan" — verified on this board
PWM="$HWMON/pwm1"
PWM_ENABLE="$HWMON/pwm1_enable"

# Zones that actually matter. RK3588 reports seven; these are the ones that
# lead under load (NPU included, since inference is this robot's hot path).
ZONES=(
  /sys/class/thermal/thermal_zone0/temp   # soc
  /sys/class/thermal/thermal_zone1/temp   # bigcore0
  /sys/class/thermal/thermal_zone2/temp   # bigcore1
  /sys/class/thermal/thermal_zone5/temp   # gpu
  /sys/class/thermal/thermal_zone6/temp   # npu
)

# ---- curve ------------------------------------------------------------------
# MEASURED on this board, not guessed:
#
#   passive idle, fan fully off, settled 5 min ....... 52-53 C   (stable, safe)
#   idle with fan at ~30% ............................ 50-51 C
#   idle with fan at 100% ............................ 37-41 C
#   8 cores busy, fan modulating ..................... 62 C
#   RK3588 throttle point ............................ ~85 C
#
# The key number is passive idle: 53 C. Two earlier attempts set OFF_TEMP below
# it (48, then 50) and the fan could never switch off — it would cool the board
# to just above its own threshold and sit there indefinitely. An off-threshold
# has to be ABOVE the temperature the board reaches with no fan at all, or it is
# unreachable by construction.
#
# TRADE-OFF, stated plainly: with these settings the robot idles around 53 C
# silently instead of 38 C with the fan audible. 53 C is entirely safe for
# RK3588 — 30 C of headroom before throttling — and it buys silence near the
# future microphone plus battery minutes. If you would rather run cool than
# quiet, drop OFF_TEMP to ~45 and accept a fan that essentially never stops.
OFF_TEMP=60        # fan stays off below this; must exceed passive idle (53 C)
FULL_TEMP=80       # 100% here, just under the ~85 C throttle point

# Switch-off uses a DWELL TIME, not a temperature hysteresis band.
#
# Hysteresis was tried and does not work on this hardware: passive idle is 53 C
# and min-PWM equilibrium is 56 C, only 3 C apart. Any band wide enough to stop
# the fan chattering is also wide enough that the board never gets that cool
# while the fan is running — so the fan latches on forever at minimum speed.
#
# Requiring the temperature to stay under OFF_TEMP for a sustained window
# prevents chatter just as well, and cannot become unreachable.
OFF_DWELL=60       # seconds continuously below OFF_TEMP before switching off

# Small 5 V fans will not start from rest below roughly 40% duty, but once
# spinning they keep going much lower. Kick to START_PWM for a moment, then
# settle onto the curve.
MIN_PWM=70         # lowest duty that keeps it turning
START_PWM=140      # brief kick to overcome stiction
MAX_PWM=255

INTERVAL=3         # seconds between samples
STEP=25            # max PWM change per tick, so it ramps instead of stepping

# -----------------------------------------------------------------------------

log() { printf '[fan] %s\n' "$*"; }

[ -w "$PWM" ] || { log "cannot write $PWM (need root)"; exit 1; }

# 1 = manual control. Without this the kernel driver owns the pin and our
# writes are ignored.
echo 1 > "$PWM_ENABLE" 2>/dev/null || true

current=0
fan_on=0
cool_for=0

read_temp() {
  local max=0 t
  for z in "${ZONES[@]}"; do
    [ -r "$z" ] || continue
    read -r t < "$z"
    t=$(( t / 1000 ))
    (( t > max )) && max=$t
  done
  echo "$max"
}

# Ramp rather than jump: an abrupt change is audible as a click, and a slow
# ramp is far less noticeable than a fan that steps between speeds.
approach() {
  local target=$1
  if   (( target > current + STEP )); then current=$(( current + STEP ))
  elif (( target < current - STEP )); then current=$(( current - STEP ))
  else current=$target
  fi
  echo "$current" > "$PWM"
}

cleanup() {
  log "exiting — fan to full (fail safe)"
  echo "$MAX_PWM" > "$PWM" 2>/dev/null || true
  exit 0
}
trap cleanup INT TERM

log "started: on>=${OFF_TEMP}C  full>=${FULL_TEMP}C  off after ${OFF_DWELL}s cool  min_pwm=${MIN_PWM}"

while true; do
  temp=$(read_temp)

  if (( fan_on )); then
    # Count how long we have been comfortably cool. Reset the moment we are not,
    # so a brief dip does not switch the fan off mid-workload.
    if (( temp < OFF_TEMP )); then
      cool_for=$(( cool_for + INTERVAL ))
    else
      cool_for=0
    fi

    if (( cool_for >= OFF_DWELL )); then
      fan_on=0
      cool_for=0
      target=0
    elif (( temp >= FULL_TEMP )); then
      target=$MAX_PWM
    else
      span=$(( FULL_TEMP - OFF_TEMP ))
      over=$(( temp - OFF_TEMP ))
      (( over < 0 )) && over=0
      target=$(( MIN_PWM + (over * (MAX_PWM - MIN_PWM)) / span ))
    fi
  else
    if (( temp >= OFF_TEMP )); then
      fan_on=1
      cool_for=0
      # Kick past stiction, then let the curve take over next tick.
      echo "$START_PWM" > "$PWM"
      current=$START_PWM
      sleep 1
      continue
    fi
    target=0
  fi

  approach "$target"
  sleep "$INTERVAL"
done
