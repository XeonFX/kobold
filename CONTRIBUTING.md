# Contributing

## Before opening a PR

```bash
make check      # protocol-check + Python tests + native firmware tests
```

No hardware is needed for any of it.

## If you touch the protocol

`protocol/protocol.yaml` is the single source of truth. Never hand-edit
`protocol_generated.h` or `protocol_generated.py`.

```bash
make protocol   # regenerate, then commit BOTH generated files
```

**Bump `version:` in protocol.yaml for any change to message ids or field
layouts.** The version byte is a safety interlock, not bookkeeping: the bridge
refuses to command a board whose version disagrees, which is what stops a
half-finished firmware update from driving the robot with mismatched struct
offsets.

If you change the framing itself (COBS, CRC, header layout), update **both**
implementations and the shared test vectors in:

- `firmware/test/native/test_codec.cpp` → `test_cross_language_vectors`
- `ros2_ws/src/kobold_bridge/test/test_link.py` → `test_cross_language_vectors`

Those two tests assert the same bytes on purpose. They are the only thing
preventing the two codecs from drifting apart silently.

## Firmware

Arduino framework, built with PlatformIO. Arduino for the mature libraries;
ESP-IDF APIs called directly wherever timing matters — `ledcAttach`/`ledcWrite`
for motor PWM, never `analogWrite`.

Rules that exist because of specific failure modes:

- **ISRs are `IRAM_ATTR` and do almost nothing.** Increment a counter, set a
  flag, return. No `Serial.print`, no I2C, no allocation.
- **Anything shared with an ISR** goes in a `portMUX` critical section or is
  read as an aligned 32-bit value. Multi-field snapshots (the four encoder
  counters) must be taken under one critical section, or odometry mixes pre-
  and post-tick values.
- **Every failure path ends with the motors stopped.** If you add a fault, add
  it to the `blocking` mask in `controlTask`.
- **Safety decisions stay on the MCU.** Anything with a deadline under ~100 ms
  cannot round-trip through the SBC. Cliff detection is the worked example.

## ROS 2

- The bridge does **no** sensor fusion and publishes **no** `odom→base_link`
  transform. `robot_localization` owns both. Two writers on one TF edge is a
  debugging nightmare.
- Ranges with no echo publish as `+inf`, never `0`. Zero reads downstream as
  "obstacle touching the sensor".
- New tools exposed to the agent must be coarse-grained and safe to call in any
  state. `navigate_to("kitchen")` is a good tool; `set_left_motor_pwm(180)` puts
  a 1 Hz model inside a 100 Hz loop.

## Style

Python is formatted with `ruff` (`make format`). C++ follows the surrounding
code: 2-space indent, 100 columns, `lower_snake_case` for functions and
variables, `UpperCamelCase` for types.

Comments should explain **why**, not what. `docs/` is where the reasoning lives;
if a decision took real thought, write it down there and link to it.

## Reporting hardware issues

Include the output of `make check`, your board's `version` line from the bridge
log (it carries firmware version and git hash, and flags builds from a dirty
tree), and which of the two boards is involved.
