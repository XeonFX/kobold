#!/usr/bin/env bash
# One-time robot bootstrap. Run ON the robot, once, after Docker and /data exist.
#
#   curl -fsSL https://raw.githubusercontent.com/XeonFX/kobold/main/scripts/robot-setup.sh | bash
#   # or, from a checkout:  ./scripts/robot-setup.sh
#
# Creates /opt/kobold (the git checkout the deploy pipeline drives), the systemd
# units that bring the stack up at boot, and the update-check timer.
#
# Deliberately keeps a small floor of things OUTSIDE Docker: sshd, networking,
# udev, and an emergency esptool. Anything you need in order to FIX a broken
# robot must not itself depend on Docker.
set -euo pipefail

REPO="${KOBOLD_REPO:-https://github.com/XeonFX/kobold.git}"
ROOT=/opt/kobold

log()  { printf '\033[36m==>\033[0m %s\n' "$*"; }
die()  { printf '\033[31m[x]\033[0m %s\n' "$*" >&2; exit 1; }

[ "$(id -u)" -ne 0 ] || die "run as your normal user (it will sudo where needed)"
command -v docker >/dev/null || die "install Docker first"
mountpoint -q /data || die "/data is not mounted"

# ---------------------------------------------------------------- checkout --
log "checkout at $ROOT"
if [ -d "$ROOT/.git" ]; then
  git -C "$ROOT" fetch --quiet origin
else
  sudo mkdir -p "$ROOT"
  sudo chown "$USER:$USER" "$ROOT"
  git clone --quiet "$REPO" "$ROOT"
fi

# ------------------------------------------------------------------ layout --
log "data directories"
sudo mkdir -p /data/{models,maps,bags,targets,agent,deploy,firmware}
sudo chown -R "$USER:$USER" /data/{models,maps,bags,targets,agent,firmware}
sudo mkdir -p /data/deploy && sudo chown "$USER:$USER" /data/deploy

# --------------------------------------------------------- emergency tools --
# esptool on the HOST, not only in a container. If the container stack is broken
# you still want to reflash the microcontrollers over SSH rather than carrying
# the robot to a desk with a monitor.
log "emergency firmware tooling (host-side, outside Docker)"
if ! command -v esptool.py >/dev/null 2>&1; then
  pipx install esptool 2>/dev/null || pip3 install --user --break-system-packages esptool 2>/dev/null \
    || echo "  (install esptool manually — it is your recovery path)"
fi

# ----------------------------------------------------------------- systemd --
log "systemd units"

sudo tee /etc/systemd/system/kobold.service >/dev/null <<UNIT
[Unit]
Description=Kobold robot stack
Requires=docker.service
After=docker.service network-online.target data.mount
# Compose will happily start the bridge before /data mounts or before USB
# enumerates; these ordering constraints are what stop that.
Wants=network-online.target

[Service]
Type=oneshot
RemainAfterExit=yes
WorkingDirectory=$ROOT
Environment=KOBOLD_ROOT=$ROOT
ExecStart=/usr/bin/docker compose -f $ROOT/docker/compose.yaml --env-file $ROOT/docker/versions.env up -d
ExecStop=/usr/bin/docker compose -f $ROOT/docker/compose.yaml down
TimeoutStartSec=300
User=$USER

[Install]
WantedBy=multi-user.target
UNIT

# Checks for updates and NOTIFIES. Never installs. An auto-pull at 3 a.m. that
# breaks the bridge while the robot is mid-room is a genuinely bad night, and
# that is exactly what "everything auto-updates" produces.
sudo tee /etc/systemd/system/kobold-update-check.service >/dev/null <<UNIT
[Unit]
Description=Check for Kobold updates (notify only, never install)

[Service]
Type=oneshot
WorkingDirectory=$ROOT
User=$USER
ExecStart=/bin/bash -c 'git -C $ROOT fetch --quiet origin && \
  LOCAL=\$(git -C $ROOT rev-parse --short=8 HEAD) && \
  REMOTE=\$(git -C $ROOT rev-parse --short=8 origin/main) && \
  if [ "\$LOCAL" != "\$REMOTE" ]; then echo "\$REMOTE" > /data/deploy/available; \
  else rm -f /data/deploy/available; fi'
UNIT

sudo tee /etc/systemd/system/kobold-update-check.timer >/dev/null <<'UNIT'
[Unit]
Description=Hourly Kobold update check

[Timer]
OnBootSec=5min
OnUnitActiveSec=1h
Persistent=true

[Install]
WantedBy=timers.target
UNIT

sudo systemctl daemon-reload
sudo systemctl enable kobold-update-check.timer >/dev/null
log "enabled kobold-update-check.timer"
log "kobold.service created but NOT enabled — enable it once a deploy succeeds:"
echo "      sudo systemctl enable kobold.service"

# -------------------------------------------------------------------- done --
cat <<EOF

$(printf '\033[32mrobot bootstrap complete\033[0m')

  checkout   $ROOT
  data       /data  ($(df -h /data | awk 'NR==2{print $4}') free)
  update     hourly check, notify only

Next:
  1. ./scripts/fetch-models.sh          download models into /data/models
  2. cd $ROOT && ./scripts/deploy.sh --latest
  3. sudo systemctl enable kobold.service     once that deploy is healthy
EOF
