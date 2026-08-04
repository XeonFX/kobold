#!/usr/bin/env bash
# Container entrypoint: source ROS, then assert the version chain before doing
# anything else.
#
# The RKNN/RKLLM version coupling crosses the container boundary — the runtime
# library inside here must match the rknpu kernel driver on the host. That is
# the one place containerisation does NOT give independence, and getting it
# wrong produces failures that look like model corruption rather than a version
# problem. Print all three numbers at startup so a mismatch is the first thing
# in the log, not a two-hour debugging session.
set -e

[ -f /opt/ros/jazzy/setup.bash ] && source /opt/ros/jazzy/setup.bash
[ -f "${ROS_WS:-/ws}/install/setup.bash" ] && source "${ROS_WS:-/ws}/install/setup.bash"

banner() {
  echo "--------------------------------------------------------------"
  echo " kobold  sha=${KOBOLD_SHA:-dev}  container=$(hostname)"

  if [ -r /sys/kernel/debug/rknpu/version ]; then
    echo " rknpu driver (host): $(cat /sys/kernel/debug/rknpu/version 2>/dev/null)"
  fi

  local rt
  rt=$(ls /usr/lib/aarch64-linux-gnu/librknnrt.so.* 2>/dev/null | head -1)
  [ -n "$rt" ] && echo " librknnrt (container): $(basename "$rt")"

  local llm
  llm=$(ls /usr/lib/librkllmrt.so* 2>/dev/null | head -1)
  [ -n "$llm" ] && echo " librkllmrt (container): $(basename "$llm")"

  if [ -e /dev/dri/renderD129 ]; then
    echo " npu device: present  (core mask=${RKNN_CORE_MASK:-default})"
  else
    echo " npu device: ABSENT — /dev/dri not mapped in?"
  fi
  echo "--------------------------------------------------------------"
}

banner
exec "$@"
