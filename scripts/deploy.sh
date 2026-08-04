#!/usr/bin/env bash
# Deploy a specific commit to the robot. Runs ON the robot.
#
#   ./scripts/deploy.sh <sha>        deploy that commit
#   ./scripts/deploy.sh --latest     deploy the newest release
#   ./scripts/deploy.sh --force      skip preflight gates (know why you're doing this)
#
# One git SHA is one complete robot state: images, firmware and config all come
# from the same commit. "What is running?" has exactly one answer.
#
# Everything here is reversible. If the health check fails, this rolls back
# automatically — a robot left in a half-deployed state is worse than one
# running slightly old code.
set -euo pipefail

ROOT="${KOBOLD_ROOT:-/opt/kobold}"
DEPLOY_STATE=/data/deploy
COMPOSE="docker compose -f $ROOT/docker/compose.yaml --env-file $ROOT/docker/versions.env"
PROFILES="${KOBOLD_PROFILES:-}"

# Physical gates. A robot has state a web service doesn't.
MIN_BATTERY_PCT=40
MIN_FREE_GB=8
IDLE_SECONDS=5

log()  { printf '\033[36m==>\033[0m %s\n' "$*"; }
warn() { printf '\033[33m[!]\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[31m[x]\033[0m %s\n' "$*" >&2; exit 1; }

FORCE=0
TARGET=""
for arg in "$@"; do
  case "$arg" in
    --force)  FORCE=1 ;;
    --latest) TARGET="latest" ;;
    -*)       die "unknown flag: $arg" ;;
    *)        TARGET="$arg" ;;
  esac
done
[ -n "$TARGET" ] || die "usage: deploy.sh <sha>|--latest [--force]"

# ---------------------------------------------------------------- preflight --

preflight() {
  log "preflight"

  # 1. The robot must not be moving. Restarting the bridge mid-drive leaves the
  #    motors in whatever state the last PWM write set until the watchdog fires.
  if command -v docker >/dev/null && docker ps --format '{{.Names}}' | grep -q kobold-bridge; then
    local moving=0
    for _ in $(seq 1 $IDLE_SECONDS); do
      if docker exec kobold-bridge timeout 1 ros2 topic echo /cmd_vel --once 2>/dev/null \
           | grep -qE 'x: [^0]|z: [^0]'; then
        moving=1; break
      fi
      sleep 1
    done
    [ "$moving" -eq 0 ] || die "robot is moving — stop it first (or --force)"
  fi

  # 2. Battery. A brownout during esptool can leave an ESP32 needing a manual
  #    BOOT-button recovery, which means physically reaching the robot.
  local pct
  pct=$(cat "$DEPLOY_STATE/battery_pct" 2>/dev/null || echo "")
  if [ -n "$pct" ] && [ "${pct%.*}" -lt "$MIN_BATTERY_PCT" ]; then
    die "battery ${pct}% is below ${MIN_BATTERY_PCT}% — charge before deploying"
  fi

  # 3. Disk. A pull that dies halfway leaves you with neither version.
  local free_gb
  free_gb=$(df -BG /data | awk 'NR==2{gsub(/G/,"",$4); print $4}')
  [ "$free_gb" -ge "$MIN_FREE_GB" ] || die "only ${free_gb}G free on /data (need ${MIN_FREE_GB}G)"

  # 4. The NPU version chain crosses the container boundary — assert it here so
  #    a mismatch fails at deploy rather than mysteriously at inference time.
  local want_driver have_driver
  want_driver=$(grep -A2 '^host_requirements:' "$ROOT/models/manifest.yaml" \
                 | awk -F'"' '/rknpu_driver/{print $2}')
  have_driver=$(sudo cat /sys/kernel/debug/rknpu/version 2>/dev/null | grep -oE '[0-9]+\.[0-9]+\.[0-9]+' || echo "")
  if [ -n "$want_driver" ] && [ -n "$have_driver" ] && [ "$want_driver" != "$have_driver" ]; then
    warn "rknpu driver is $have_driver, manifest expects $want_driver"
    [ "$FORCE" -eq 1 ] || die "refusing (use --force if intentional)"
  fi

  log "preflight OK"
}

# ------------------------------------------------------------------ deploy --

resolve_sha() {
  if [ "$TARGET" = "latest" ]; then
    git -C "$ROOT" fetch --tags --quiet origin
    git -C "$ROOT" rev-parse --short=8 origin/main
  else
    echo "${TARGET:0:8}"
  fi
}

record_state() {
  sudo mkdir -p "$DEPLOY_STATE"
  [ -f "$DEPLOY_STATE/current" ] && sudo cp "$DEPLOY_STATE/current" "$DEPLOY_STATE/previous"
  echo "$1" | sudo tee "$DEPLOY_STATE/current" >/dev/null
  date -Iseconds | sudo tee "$DEPLOY_STATE/deployed_at" >/dev/null
}

health_check() {
  log "health check"
  local ok=1

  sleep 8
  for c in kobold-base kobold-bridge kobold-app; do
    if ! docker ps --format '{{.Names}}' | grep -q "^${c}$"; then
      warn "$c is not running"; ok=0
    fi
  done

  # "It started" is not "it works". The bridge only publishes telemetry once the
  # firmware version handshake has actually succeeded.
  if docker ps --format '{{.Names}}' | grep -q kobold-bridge; then
    if ! docker exec kobold-bridge timeout 10 ros2 topic echo /battery_state --once >/dev/null 2>&1; then
      warn "no telemetry from the drive board — firmware version mismatch?"
      ok=0
    fi
  fi

  curl -sf --max-time 5 http://localhost:8000/healthz >/dev/null 2>&1 || { warn "app not responding"; ok=0; }

  return $((1 - ok))
}

main() {
  local sha; sha=$(resolve_sha)
  log "deploying $sha"

  [ "$FORCE" -eq 1 ] && warn "--force: skipping preflight" || preflight

  local previous; previous=$(cat "$DEPLOY_STATE/current" 2>/dev/null || echo "")

  # Config comes from the git checkout, not from the images. A config-only
  # change is therefore a checkout away, with no rebuild and no pull.
  log "checking out $sha"
  git -C "$ROOT" fetch --quiet origin
  git -C "$ROOT" checkout --quiet "$sha" || die "unknown commit: $sha"

  [ -f "$ROOT/docker/versions.env" ] || die "no versions.env at $sha — did CI publish it?"

  log "pulling images (by digest)"
  local profile_args=""
  for p in $PROFILES; do profile_args="$profile_args --profile $p"; done
  # shellcheck disable=SC2086
  $COMPOSE $profile_args pull --quiet || die "pull failed — old version still running"

  log "starting services"
  # shellcheck disable=SC2086
  $COMPOSE $profile_args up -d --remove-orphans

  log "firmware"
  "$ROOT/scripts/flash.sh" both || warn "firmware flash reported a problem"

  if health_check; then
    record_state "$sha"
    log "deployed $sha successfully"
  else
    warn "health check FAILED"
    if [ -n "$previous" ] && [ "$FORCE" -eq 0 ]; then
      warn "rolling back to $previous"
      exec "$ROOT/scripts/rollback.sh"
    fi
    die "deployment unhealthy and no previous version to roll back to"
  fi
}

main
