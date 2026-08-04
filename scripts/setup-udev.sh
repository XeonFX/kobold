#!/usr/bin/env bash
# Install udev rules for stable MCU device names, and report what is connected.
#
#   sudo ./scripts/setup-udev.sh
set -euo pipefail

RULES_SRC="$(dirname "$0")/99-kobold.rules"
RULES_DST="/etc/udev/rules.d/99-kobold.rules"

if [[ "$(uname)" != "Linux" ]]; then
  echo "udev is Linux-only. On macOS the boards appear as /dev/cu.usbserial-*;"
  echo "pass those paths to the bridge directly with drive_port:= / sense_port:=."
  exit 1
fi

if [[ $EUID -ne 0 ]]; then
  echo "needs root: sudo $0" >&2
  exit 1
fi

echo "==> connected USB-serial devices"
found=0
for dev in /dev/ttyUSB* /dev/ttyACM*; do
  [[ -e "$dev" ]] || continue
  found=1
  vid=$(udevadm info -a -n "$dev" 2>/dev/null | grep -m1 'ATTRS{idVendor}' | cut -d'"' -f2 || true)
  pid=$(udevadm info -a -n "$dev" 2>/dev/null | grep -m1 'ATTRS{idProduct}' | cut -d'"' -f2 || true)
  ser=$(udevadm info -a -n "$dev" 2>/dev/null | grep -m1 'ATTRS{serial}' | cut -d'"' -f2 || true)
  path=$(udevadm info -q path -n "$dev" 2>/dev/null | grep -oE '[0-9]+-[0-9.]+' | head -1 || true)
  printf '  %-14s vid:pid=%s:%s serial=%-20s port=%s\n' \
    "$dev" "${vid:-?}" "${pid:-?}" "${ser:-<none>}" "${path:-?}"
done
[[ $found -eq 1 ]] || echo "  (none found — are the boards plugged in?)"

echo
echo "==> installing $RULES_DST"
install -m 0644 "$RULES_SRC" "$RULES_DST"
udevadm control --reload-rules
udevadm trigger --subsystem-match=tty
sleep 1

echo
echo "==> resulting symlinks"
for link in /dev/robot-drive /dev/robot-sense /dev/robot-lidar; do
  if [[ -e "$link" ]]; then
    printf '  %-18s -> %s\n' "$link" "$(readlink -f "$link")"
  else
    printf '  %-18s MISSING\n' "$link"
  fi
done

cat <<'EOF'

If a symlink is missing, the vid:pid in 99-kobold.rules does not match your
board. Copy the values printed above into the rules file and re-run.

If two boards report the SAME vid:pid and serial, you must match on physical USB
port instead — uncomment the KERNELS= fallback rules at the bottom of the file,
use the "port=" values above, and always plug each board into the same socket.
EOF
