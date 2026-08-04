#!/usr/bin/env bash
# Flash a kobold board and verify it comes back speaking the right protocol.
#
#   ./scripts/flash.sh drive
#   ./scripts/flash.sh sense
#   ./scripts/flash.sh both
#
# This is what the `updater` container runs on the robot. It refuses to flash
# while the robot is moving, keeps the previous binary for rollback, and
# verifies the version handshake afterwards — a board that flashes successfully
# but reports the wrong protocol version is still a broken board.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BACKUP_DIR="${ROOT}/.firmware-backup"
TARGET="${1:-both}"

flash_one() {
  local env_name="$1"
  local port="/dev/robot-${env_name}"
  local bin="${ROOT}/firmware/.pio/build/${env_name}/firmware.bin"

  echo "==> ${env_name}"

  if [[ ! -e "$port" ]]; then
    echo "    ERROR: $port not found. Run 'make udev', or pass the raw device." >&2
    return 1
  fi

  # Never flash a moving robot. A board that resets mid-drive leaves the motors
  # in whatever state the last PWM write set, until the watchdog catches it.
  if command -v ros2 >/dev/null 2>&1; then
    if timeout 2 ros2 topic echo /cmd_vel --once >/dev/null 2>&1; then
      echo "    ERROR: /cmd_vel is active — stop the robot before flashing." >&2
      return 1
    fi
  fi

  echo "    building..."
  (cd "${ROOT}/firmware" && pio run -e "$env_name" >/dev/null)

  mkdir -p "$BACKUP_DIR"
  if [[ -f "${BACKUP_DIR}/${env_name}.bin" ]]; then
    cp "${BACKUP_DIR}/${env_name}.bin" "${BACKUP_DIR}/${env_name}.prev.bin"
  fi
  cp "$bin" "${BACKUP_DIR}/${env_name}.bin"

  echo "    flashing $port..."
  (cd "${ROOT}/firmware" && pio run -e "$env_name" -t upload --upload-port "$port" >/dev/null)

  echo "    verifying handshake..."
  sleep 1
  if python3 - "$port" "$env_name" <<'PY'
import sys, time, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).parent))
sys.path.insert(0, "ros2_ws/src/kobold_bridge")
from kobold_bridge.serial_board import SerialBoard, BoardVersionMismatch

port, name = sys.argv[1], sys.argv[2]
b = SerialBoard(port, name=name)
try:
    b.open()
    v = b.wait_for_version(timeout=5.0)
    print(f"    OK  fw {v.fw_major}.{v.fw_minor}.{v.fw_patch}  proto v{v.protocol_version}")
except BoardVersionMismatch as e:
    print(f"    VERSION MISMATCH: {e}"); sys.exit(1)
except Exception as e:
    print(f"    NO RESPONSE: {e}"); sys.exit(1)
finally:
    b.close()
PY
  then
    echo "    done"
  else
    echo "    FAILED — previous binary kept at ${BACKUP_DIR}/${env_name}.prev.bin" >&2
    return 1
  fi
}

cd "$ROOT"
case "$TARGET" in
  drive|sense|head) flash_one "$TARGET" ;;
  both)             flash_one drive && flash_one sense ;;
  *) echo "usage: $0 {drive|sense|head|both}" >&2; exit 1 ;;
esac
