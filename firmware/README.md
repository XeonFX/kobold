# Firmware

Three ESP32 DevKit V1 boards (ESP-WROOM-32, 30-pin), one PlatformIO project.

| Environment | Board | Job |
|---|---|---|
| `drive` | USB-C variant | Motors, encoders, IMU, battery, **cliff reflex** |
| `sense` | CH340/CP2102 | Ultrasonics, IR, buzzer, OLED, **safety line** |
| `head` | spare | Pan/tilt servos, PIR (Phase 5 — leave unpopulated) |
| `native` | your laptop | Codec unit tests, no hardware |

## Build and flash

```bash
pio run -e drive              # build
pio run -e drive -t upload    # build + flash
pio test -e native            # codec tests on your machine
```

Or from the repo root: `make firmware`, `make flash`, `make firmware-test`.

## Why the drive board is the USB-C one

Its USB-serial bridge chip has a different VID/PID from the other two, which
makes the udev rule unambiguous. CH340 clones frequently share both a VID/PID
*and* a serial number — and the failure mode of confusing the boards is sending
motor commands to the ultrasonic hub.

## Why two boards

Pin budget forces it, but the better reason is timing isolation. HC-SR04 echo
width is measured in microseconds. On one board it would share a core with motor
PWM and four encoder ISRs, so a burst of encoder interrupts during a hard turn
would corrupt exactly the distance reading you need *during a hard turn*.

## The safety architecture

Two paths that never touch the SBC:

**Cliff → drive board, directly.** Driving on a table gives ~166 ms between the
leading sensor clearing the edge and the wheel following it at 0.3 m/s. A round
trip through USB → bridge → ROS 2 → back is 50–200 ms. It does not fit, so the
decision stays on the MCU: ISR → `motors::stopFromISR()` → direct GPIO register
write, under a millisecond.

Cliff detection uses **both** edge interrupts and level polling. Not belt-and-
braces: an edge-triggered interrupt cannot fire if the robot boots with a sensor
already over the edge, because no transition ever occurs. Without the level
poll, a robot powered on at the table edge would consider itself safe.

**Sense → drive over one wire.** The sense board pulls `SAFETY_OUT` low on an
imminent collision; the drive board coasts on a falling-edge interrupt. It
releases by going high-impedance rather than driving high, so the drive board's
pull-up defines the idle state and a dead sense board reads as "no danger"
rather than shorting the line.

## Task layout (drive board)

```
core 1   control task   100 Hz, vTaskDelayUntil  — PID, watchdog, faults
core 0   comms task     serial framing + 50 Hz telemetry
ISRs     4 encoders, 4 cliff, 1 safety line
```

Splitting the loops means inbound frames cannot perturb motor timing, and a
stalled control loop cannot silence telemetry.

## Things that will bite you

**`esptool` auto-reset.** Test it on day one — remote flashing depends on it:

```bash
esptool.py --port /dev/robot-drive --before default_reset --after hard_reset chip_id
```

If it demands a manual BOOT press, add a 10 µF cap from EN to GND. The
bulletproof fix is wiring two Rock 5B GPIOs to EN and IO0.

**Power LM393 modules at 3.3 V.** Both the encoders and the IR sensors are
LM393 comparator boards. At 5 V, `D0` is a 5 V signal into a 3.3 V pin. The
LM393 works down to 2 V, so at 3.3 V the output swings 0–3.3 V and needs no
level shifting. Only the HC-SR04s genuinely require 5 V — divide their ECHO.

**The ESP32 ADC is nonlinear.** Calibrate `BATT_ADC_SCALE` in
`src/drive/config.h` against a multimeter. Do not trust the nominal value.

**Measure the geometry.** `ENCODER_COUNTS_PER_REV`, `WHEEL_DIAMETER_MM` and
`TRACK_WIDTH_MM` in `config.h` are estimates. Wrong values do not fail loudly —
they quietly corrupt odometry and you will blame the EKF for days.

`TRACK_WIDTH_MM` is **not** the physical wheel spacing. On a skid-steer chassis
the wheels scrub sideways through every turn, so it is an empirical fudge
factor: command ten 360° turns, measure the actual rotation, scale until they
agree.

## Tuning the PID

The Serial Plotter in the Arduino IDE is genuinely the best tool for this, and
using it alongside PlatformIO is fine — same core, same libraries.

Easier: stream telemetry and plot `meas_l_mm_s` against your commanded velocity.
Gains are live-tunable at runtime via the `set_pid` message, so you can tune
without reflashing.

Order: raise `KP` until it responds crisply and just starts to oscillate, back
off ~30%, then raise `KI` until steady-state error disappears. `KD` stays near
zero — these encoders are 20 counts/rev and derivative on a coarse signal is
mostly amplified noise.

## Adding a message

1. Add it to `protocol/protocol.yaml`
2. `make protocol` from the repo root
3. Handle it in the board's `onFrame`, and in `bridge_node.py`
4. Bump `version:` if you changed any existing layout
