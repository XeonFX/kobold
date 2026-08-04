# Wiring and Pin Assignments

**Board revision:** all three MCUs are **ESP32 DevKit V1 (ESP-WROOM-32, 30-pin)** — no ESP32-S3.
One has USB-C with a different USB-serial controller.

⚠️ Verify every pin against your board silkscreen before soldering. This is a proposed allocation.

---

## 0. What the ESP32 classic changes

Dropping from S3 to WROOM-32 costs you three things, all of which have clean workarounds:

| Lost | Consequence | Workaround |
|---|---|---|
| Native USB CDC | All boards enumerate as `/dev/ttyUSB*` via CP2102/CH340, not `/dev/ttyACM*` | Fine. Flashing still works over the same cable via DTR/RTS auto-reset |
| ~4 usable GPIO | Pin budget is genuinely tight now | Per-side motor wiring (§2) drops the motor pin count from 13 to 7 |
| RAM / speed | 520 KB SRAM, 240 MHz dual-core | Irrelevant — the firmware is a few KB of PID and interrupt handlers |

**Usable GPIO on the 30-pin DevKit V1:**

- **Safe, output-capable (16):** 4, 5, 13, 14, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33
- **Input-only (4):** 34, 35, 36 (VP), 39 (VN) — no pull-ups, can't drive anything.
  Perfect for encoders and ultrasonic ECHO, which is exactly why the budget works out.
- **Avoid:** 0, 2, 12, 15 (strapping — GPIO 12 held high at boot bricks the boot voltage),
  1 / 3 (UART0, used by the USB bridge), 6–11 (SPI flash, not exposed)
- **ADC:** use **ADC1 only** (GPIO 32–39). ADC2 is unusable whenever WiFi is active.

**Use the USB-C board as the drive controller.** Its different USB-serial chip gives it a distinct
VID/PID, which makes the udev rule trivial (§5) — and USB-C is the more robust connector for the
board doing the most important job.

---

## 1. Power distribution

```
   ┌─ 3S PACK A (BMS, balancing ✓) ─┐
   │                                ├──╫── COMMON 3S BANK ── 9.0–12.6 V, ~77 Wh
   └─ 3S PACK B (BMS, balancing ✓) ─┘  │
                                       │
      ┌────────────────────┬───────────┴────────┬────────────────────┐
      │                    │                    │                    │
  [10 A fuse]        [7.5 A fuse]           [3 A fuse]           [2 A fuse]
      │               + KILL SWITCH             │                    │
  HW-674 buck         HW-674 buck          LM2596 buck          LM2596 buck
     5.1 V              6.5–8 V               5.0 V                5.0 V
      │                    │                    │                    │
  + TVS + 1000 µF     motor driver         2× ESP32             2× servo
      │                    │                lidar, OLEDs        (pan/tilt)
  Rock 5B GPIO         4 motors
  pins 2 + 4
```

**Rules:**

- **Single star ground** near the pack. Motor return current through a daisy-chained ground is the
  classic source of phantom sensor noise and unexplained resets.
- **Servos on their own regulator.** Stall inrush is amps for tens of milliseconds and will drag a
  shared 5 V rail under the ESP32 brownout threshold.
- **470–1000 µF + 0.1 µF at each motor driver.** Prevents brownout resets on direction reversal.
- **Kill switch on the motor rail only.** Cutting compute power mid-write corrupts the filesystem.
- **Never parallel the packs** until both are charged and within ~50 mV.

### 1.1 Rock 5B via GPIO header — you've proven this works, now make it safe

You're powering it at 5.1 V into the header and it runs. Keep it. Four things to add, because this
path bypasses the board's input protection entirely:

| Add | Why |
|---|---|
| **TVS diode, 5.6–6.0 V, across the rail at the header** | If the XL4016's high-side switch fails short, full pack voltage (12.6 V) lands on the 5 V rail and kills the board. A €0.30 TVS plus the fuse is insurance on a €150 board. **Do this one.** |
| **1000 µF low-ESR cap at the header** | Motor current spikes sag the pack; the cap rides out the transient |
| **Both 5 V pins (2 and 4) + at least 4 grounds** (6, 9, 14, 20, 25, 30, 34, 39), short thick wire | Header pins are ~2–3 A each. Both 5 V pins in parallel gets you ~4–5 A ≈ 20–25 W |
| **Measure voltage *at the header* under load** | Not at the buck output. 4 A through thin Dupont wire drops 300 mV easily — 5.1 V at the buck becomes 4.7 V at the board, which is instability you'll blame on software |

**Never connect USB-C power and GPIO power at the same time.**

**Burn-in test before you trust it:** run `stress-ng --cpu 8` plus an NPU load plus a WiFi transfer
for 30 minutes while the motors drive, and watch for resets and for `dmesg` undervoltage warnings.
The failure mode you're looking for is a reboot when the motors stall, not steady-state draw.

---

## 2. ESP32 #1 (USB-C board) — drive controller

Motors, encoders, IMU, **cliff sensors**, battery. 100–200 Hz hard real-time.

### Why per-side motor wiring

On a skid-steer chassis the four wheels are mechanically coupled through the floor — front and rear
on a side can never usefully differ. So tie them: per side, one PWM signal and one direction pair
feed **both** channels of a TB6612FNG, keeping four independent H-bridges for current capacity but
using only 3 GPIO per side.

That's 7 pins instead of 13, and it's the better control design anyway. Per-wheel PID on a skid
steer has the wheels fighting each other through the carpet; per-side velocity PID is what
differential drive actually wants.

| Function | Pin | Notes |
|---|---|---|
| **Motors — 2× TB6612FNG, per-side tied** | | |
| PWM left (→ PWMA+PWMB of driver #1) | GPIO 4 | LEDC, 20 kHz (above audible) |
| LIN1 / LIN2 (→ AIN1+BIN1 / AIN2+BIN2) | GPIO 5 / 16 | |
| PWM right (→ PWMA+PWMB of driver #2) | GPIO 17 | |
| RIN1 / RIN2 | GPIO 18 / 19 | |
| STBY (both drivers) | GPIO 23 | **LOW = coast all motors.** The software e-stop |
| **Encoders (LM393, D0 only)** | | |
| Front-left / front-right | GPIO 34 / 35 | Input-only. Interrupt on RISING |
| Rear-left / rear-right | GPIO 36 / 39 | Input-only |
| **Cliff sensors — 4× IR downward at corners** | | |
| Cliff FL / FR | GPIO 13 / 14 | ⚠️ On the drive board **on purpose** — see §2.1 |
| Cliff BL / BR | GPIO 25 / 26 | |
| **I²C (MPU-6050)** | | |
| SDA / SCL | GPIO 21 / 22 | 3.3 V, 400 kHz |
| **Misc** | | |
| Battery sense | GPIO 32 | **ADC1.** 47 k / 10 k divider: 12.6 V → 2.21 V. Calibrate against a multimeter — ESP32 ADC is nonlinear |
| **SAFETY_IN** from sense board | GPIO 27 | Hardware interrupt → immediate coast. See §4 |
| Spare | GPIO 33 | |

Total: 15 of 16 safe pins + all 4 input-only. Tight but it fits.

### 2.1 Cliff sensors live here, not on the sense board

You want to drive on a table. That makes cliff detection the highest-priority safety input in the
system, and it has a hard deadline.

At 0.3 m/s with a sensor mounted 5 cm ahead of the wheel, you have **166 ms** from detection to the
wheel leaving the table. A round trip through USB → serial bridge → ROS 2 → Nav2 → back is 50–200 ms
on a good day. That doesn't fit.

So the 4 downward IRs go straight to the drive board, and the drive firmware stops the motors in
**under 1 ms** without asking anyone's permission. The SBC finds out afterwards.

**Table-driving rules for the firmware:**

- **Any** cliff sensor triggering → immediate coast (STBY low), then latch a `CLIFF` fault.
- Recovery: back away **in the direction of the sensors still reading "table"**, slowly, then stop
  and report. Never resume automatically.
- **Cap speed at 0.15 m/s in table mode.** Both detection distance and stopping distance scale with
  speed, and you have very little of either.
- Rear cliff sensors matter as much as the front — the cat game reverses constantly.
- Calibrate the Flying-Fish potentiometers **on your actual table**. Glossy surfaces reflect
  specularly and dark matte surfaces absorb; both fool IR. Test the real surface, at the real
  mounting angle, before trusting it.

**Known gap:** with all 4 corner IRs pointing down, the only horizontal close-range sensing is the
front and rear IR plus 4 ultrasonics with a ~15° cone. The **sides have no close-range coverage**,
and skid steer sweeps sideways during every turn. Either turn slowly near obstacles, or add 2 more
IR modules (~€4) for left/right horizontal.

---

## 3. ESP32 #2 — sensor hub

Ultrasonics, horizontal IR, buzzer, OLED. 20–50 Hz.

| Function | Pin | Notes |
|---|---|---|
| **Ultrasonic** | | |
| TRIG (all 4, shared) | GPIO 23 | Fired **sequentially**, never together — they hear each other's pings |
| ECHO front / back | GPIO 34 / 35 | Input-only. **5 V output → level converter or 10 k/20 k divider** |
| ECHO left / right | GPIO 36 / 39 | Input-only |
| **Horizontal IR** | | |
| IR front / IR back | GPIO 13 / 14 | The two forward/rearward-facing modules |
| **I²C (OLED SSD1306)** | | |
| SDA / SCL | GPIO 21 / 22 | Address 0x3C |
| **Buzzer** | GPIO 5 | Via S9013 NPN + 100 Ω base resistor |
| **SAFETY_OUT** → drive board | GPIO 25 | See §4 |
| Spare | 4, 16, 17, 18, 19, 26, 27, 32, 33 | Room for the PIR and servos later |

### Sense firmware behaviour

- **Sequential ranging**, ~60 ms cycle: fire, wait for echo or timeout, next sensor.
- **Median-of-3** per range. HC-SR04s throw wild outliers, and one bad reading triggering an e-stop
  mid-drive is maddening.
- Publish "no echo" as `+inf`, **never `0`** — a zero reads downstream as "obstacle touching the
  sensor" and hard-stops the robot for no reason.
- Assert SAFETY_OUT on any range below the danger threshold (§4).

---

## 4. The hardware safety line

One wire between the two ESP32s: **sense GPIO 25 → drive GPIO 27.**

When the sense board sees an imminent collision, it pulls this line and the drive board coasts the
motors on a hardware interrupt. Latency is under a millisecond, versus 50–200 ms through the SBC.

It costs one wire and it means **the emergency stop path has no software above the two MCUs in it**.
The Rock 5B can be rebooting, the LLM can be hallucinating, Docker can be pulling images — the robot
still stops.

```
   sense: obstacle < 15 cm  ─┐
   sense: IR front/back hit ─┼─► SAFETY_OUT (GPIO 25) ──wire──► drive GPIO 27 ──► STBY low
   sense: watchdog expired  ─┘                                        (< 1 ms)

   drive: cliff sensor hit ──────────────────────────────────────► STBY low (local, < 1 ms)
```

Also share a ground wire between the two boards alongside the safety line.

---

## 5. USB and device naming

All three boards use USB-serial bridges, so everything is `/dev/ttyUSB*`.

| Device | Chip | Symlink |
|---|---|---|
| ESP32 drive (USB-C board) | CH9102 / CH343 — distinct VID/PID | `/dev/robot-drive` |
| ESP32 sense | CP2102 or CH340 | `/dev/robot-sense` |
| Lidar (if bought) | CP2102 | `/dev/robot-lidar` |

```
# /etc/udev/rules.d/99-kobold.rules
# Drive board — identified by its distinct USB-C bridge chip
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", SYMLINK+="robot-drive"
# Sense board — CH340 clones often share a serial, so match the physical port instead
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", KERNELS=="1-1.2", SYMLINK+="robot-sense"
```

⚠️ **CH340 clones frequently have no unique serial number.** If both boards share a VID/PID *and* a
serial, you must match on the physical USB port path (`KERNELS==`) — which means always plugging
each board into the same port. Label the ports on the Rock 5B with tape. This is exactly why the
USB-C board (different chip → different VID/PID) should be the drive controller.

Find the values with `udevadm info -a -n /dev/ttyUSB0 | head -40`.

### Flashing over USB

`esptool` auto-resets via DTR/RTS on both CP2102 and CH340, so remote flashing works without
touching the board. **Test this on day one**, because some cheap boards have a marginal auto-reset
circuit:

```bash
esptool.py --port /dev/robot-drive --before default_reset --after hard_reset chip_id
```

If it demands a manual BOOT press, the standard fix is a 10 µF cap from EN to GND. The bulletproof
fix is wiring two Rock 5B GPIOs to EN and IO0 and driving the reset sequence yourself — worth doing
if you're going to be flashing remotely and often.

**Backup path:** these are ESP32s with WiFi. Keep ArduinoOTA compiled in as a second update route
for when USB flashing fails at an inconvenient moment.

---

## 6. LM393 modules — power them at 3.3 V

Your encoders (VCC / GND / **D0** / A0) and the IR Flying-Fish modules are both LM393
comparator boards. Same rule applies to both:

- **Power them from 3.3 V, not 5 V.** The LM393 works down to 2 V, and at 3.3 V its output swings
  0–3.3 V — directly ESP32-safe with no level shifting. At 5 V, D0 is a 5 V signal that will damage
  a 3.3 V GPIO.
- Trade-off: the IR emitter is dimmer at 3.3 V, so detection range shortens. For cliff sensing at
  2–5 cm that's irrelevant. If the horizontal IR range disappoints, run those two at 5 V through
  your level converters.
- **D0 only.** A0 (raw analog) is useful once, for diagnostics — if a wheel gives erratic counts,
  scope A0 to see whether the disc is dirty or the gap is misaligned.
- **LM393 has no hysteresis**, so its output chatters near the threshold. Debounce in firmware:
  reject encoder edges closer together than the physical maximum tick rate.

### Odometry resolution

Typical slotted disc is 20 slots/rev, single channel:

- **No direction sensing** — inferred from PWM sign. Publish raw counts and apply the sign in the
  bridge so the convention lives in one place.
- Wheel ≈ 65 mm dia → 204 mm circumference → **~10 mm per tick.** That's genuinely decent linear
  resolution.
- At 0.3 m/s: ~30 ticks/s per wheel, ~120 interrupts/s total. Nothing for a 240 MHz dual-core.
- Heading from these is still unreliable on a skid-steer platform — **the MPU-6050 gyro is your
  yaw source**, fused via the EKF. This hasn't changed.

---

## 7. Serial protocol

Framed binary, versioned, CRC-checked. Same format both directions, both boards.

```
  ┌──────┬─────┬──────┬─────┬─────────────┬───────┐
  │ 0xAA │ VER │ TYPE │ LEN │   PAYLOAD   │ CRC16 │
  │  1B  │ 1B  │  1B  │ 1B  │   0–255 B   │  2B   │
  └──────┴─────┴──────┴─────┴─────────────┴───────┘
```

- `VER` is checked on every frame. Mismatch → the bridge refuses to start and logs both versions.
  This is the safety interlock that makes remote firmware updates safe.
- COBS-encode the payload so resync after a dropped byte is trivial.
- 921600 baud (both boards go through USB-serial bridges now).
- Every command frame is acked. Motor commands unacknowledged for 300 ms trip the watchdog — the
  MCU treats silence as "stop", never as "carry on".

---

## 8. Physical build notes

- **Solder before the robot moves.** Breadboard jumpers plus vibration produce intermittent faults
  that look exactly like software bugs and cost weekends.
- **Cliff sensors mounted ahead of the wheels**, angled down, rigid. A sensor that flexes changes
  its trigger distance as the robot accelerates.
- **Wheel guards** before the first cat session.
- **Route motor wires away from sensor wires**, twist each motor pair. I²C is the most
  noise-vulnerable bus on the robot and the MPU-6050 sits on it.
- **MPU-6050 flat, near the centre of rotation, on foam.** Rigid mounting couples motor vibration
  into the gyro, and gyro yaw is load-bearing for your odometry.
- **Rock 5B cooling must breathe.** RK3588 throttles hard under sustained NPU load. Don't bury it
  under the battery pack.
- **Batteries low and centred.** A top-heavy skid-steer robot tips during hard direction changes,
  and the cat game is nothing but hard direction changes. On a table, tipping means falling.

---

*See [PROJECT_PLAN.md](PROJECT_PLAN.md) for architecture and [COMPONENTS.md](COMPONENTS.md) for the
inventory.*
