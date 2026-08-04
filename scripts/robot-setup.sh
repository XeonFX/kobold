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
sudo mkdir -p /data/{models,maps,bags,targets,agent,deploy,firmware,containerd}
sudo chown -R "$USER:$USER" /data/{models,maps,bags,targets,agent,firmware}
sudo mkdir -p /data/deploy && sudo chown "$USER:$USER" /data/deploy

# Docker 29 uses containerd's image snapshotter. Docker's `data-root` can say
# /data/docker while the multi-gigabyte snapshot store still lives on eMMC at
# /var/lib/containerd. Move the actual store and retain the old directory as a
# rollback copy; cleanup is intentionally a separate manual decision.
containerd_root="$(sudo containerd config dump 2>/dev/null \
  | awk -F"'" '/^root = / {print $2; exit}')"
if [ "$containerd_root" != /data/containerd ]; then
  [ -z "$(docker ps -q 2>/dev/null)" ] \
    || die "stop running containers before migrating containerd to NVMe"
  [ -z "$(sudo find /data/containerd -mindepth 1 -print -quit)" ] \
    || die "/data/containerd is non-empty; inspect it before migration"

  log "containerd snapshot storage -> /data/containerd"
  sudo systemctl stop docker.service docker.socket containerd.service
  sudo cp -a /var/lib/containerd/. /data/containerd/
  if sudo grep -q '^#root = "/var/lib/containerd"' /etc/containerd/config.toml; then
    sudo sed -i 's|^#root = "/var/lib/containerd"|root = "/data/containerd"|' \
      /etc/containerd/config.toml
  elif sudo grep -q '^root = ' /etc/containerd/config.toml; then
    sudo sed -i 's|^root = .*|root = "/data/containerd"|' /etc/containerd/config.toml
  else
    die "cannot locate containerd root setting in /etc/containerd/config.toml"
  fi
  sudo systemctl start containerd.service docker.service
  [ "$(sudo containerd config dump | awk -F"'" '/^root = / {print $2; exit}')" = /data/containerd ] \
    || die "containerd did not start with its NVMe root"
fi

# ---------------------------------------------------------- nvme stability --
# The Samsung 980 / PM991 family drops off the PCIe bus on RK3588 when it enters
# a deep power state: the controller stops answering, the kernel logs
#
#     nvme nvme0: controller is down; will reset: CSTS=0xffffffff
#
# and every write to /data returns EIO. Seen on this board after ~90 minutes
# idle. The kernel's own diagnostic recommends exactly these two parameters —
# one disables the drive's autonomous power states, the other stops the PCIe
# link from being powered down underneath it.
#
# Applied here rather than by hand because a fix that lives only in one
# machine's bootloader is a fix that vanishes on the next reflash.
has_nvme() {
  [ -e /dev/nvme0 ] && return 0
  # Also true when the drive has already dropped off: the PCI function stays
  # enumerated even after the nvme driver detaches, which is the exact state
  # this workaround exists to prevent.
  command -v lspci >/dev/null 2>&1 && lspci 2>/dev/null | grep -qi 'non-volatile memory'
}

if has_nvme && [ -f /etc/kernel/cmdline ]; then
  log "NVMe power-management workaround"
  command -v nvme >/dev/null 2>&1 || sudo apt-get install -y -qq nvme-cli || true

  if grep -q 'nvme_core.default_ps_max_latency_us' /etc/kernel/cmdline; then
    echo "      already applied"
  else
    sudo cp -n /etc/kernel/cmdline /etc/kernel/cmdline.bak-kobold
    sudo sed -i 's/$/ nvme_core.default_ps_max_latency_us=0 pcie_aspm=off/' /etc/kernel/cmdline
    if [ -x /usr/sbin/u-boot-update ]; then
      sudo /usr/sbin/u-boot-update >/dev/null
      echo "      applied — REBOOT REQUIRED before it takes effect"
    else
      echo "      /etc/kernel/cmdline updated, but u-boot-update was not found;"
      echo "      regenerate your bootloader config by hand"
    fi
  fi
fi

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
