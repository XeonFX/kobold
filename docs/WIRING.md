# Wiring and Pin Assignments

**Board revision:** all three MCUs are **ESP32 DevKit V1 (ESP-WROOM-32, 30-pin)** — no ESP32-S3.
All three carry a CP2102 bridge with the same factory serial; they are told apart by reprogrammed
serials, not by chip type (§5).

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

**Use the USB-C board as the drive controller** — not because it is electrically
distinguishable (it is not; see §5, all three are CP2102 with serial `0001` from the factory) but
because USB-C is the more robust connector for the board doing the most important job.

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
  + TVS + 1000 µF    + 1000 µF at VM       2× ESP32             2× servo
      │                    │                lidar, OLEDs        (pan/tilt)
  Rock 5B GPIO         4 motors
  pins 2 + 4
```

**Rules:**

- **Single star ground** near the pack. Motor return current through a daisy-chained ground is the
  classic source of phantom sensor noise and unexplained resets.
- **Servos on their own regulator.** Stall inrush is amps for tens of milliseconds and will drag a
  shared 5 V rail under the ESP32 brownout threshold.
- **Decoupling: see §1.3.** Most of it is already on the modules; only two places actually need
  parts added.
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

### 1.2 Component orientation — get these wrong and you lose parts

**Electrolytic capacitors** (1000 µF, 100 µF)

```
        ┌──────┐
        │▓▓▓▓▓▓│ ◄── stripe / arrow marks the NEGATIVE lead
        │▓▓▓▓▓▓│
        └─┬──┬─┘
          │  │
        long short
         (+)  (−)
```

The **stripe is negative**. On an untrimmed part the **longer leg is positive**.
Reverse one and it heats, vents, and occasionally bursts — they are one of the
few components that fail loudly.

Voltage rating at least 1.5–2× the rail: **16 V** parts for the 5 V and 6.5 V
rails, **25 V** if you ever put one directly on the 12.6 V pack.

**TVS diode** (unidirectional, e.g. SMBJ6.0A)

```
                   +5.1 V rail
                        │
                     ───┴───
                       ▲ │    ◄── BAND (cathode) faces the POSITIVE rail
                     ──┴─┴──
                        │
                       GND
```

The **band goes to the positive rail**, the unbanded end to ground. It sits
reverse-biased doing nothing until the rail rises past its breakdown voltage.

Fitting it backwards makes it an ordinary forward-biased diode across your
supply — a dead short that blows the fuse the instant you power up. Annoying,
but it tells you immediately rather than silently leaving the board unprotected.

**Be clear about what TVS + fuse actually buys you.** A 6.0 V TVS does not hold
the rail at 6 V. Under a 12.6 V fault it clamps around 10 V while drawing tens of
amps — and that current is what opens the fuse, in milliseconds. So the board
sees roughly 10 V for a few milliseconds instead of 12.6 V indefinitely. That is
damage *limitation*, not a guarantee. Proper protection is an OVP/eFuse IC; this
is the €0.30 version and it is enormously better than nothing.

**Ceramic 100 nF** — no polarity, fits either way. Always in *parallel* with the
electrolytic and as close to the load pins as you can get it. The electrolytic
handles bulk energy; the ceramic handles the fast edges the electrolytic's ESR
cannot.

---

### 1.3 Where every capacitor and TVS goes

```
  3S PACK  9.0 – 12.6 V   (both BMS packs, common bank, single star ground)
      │
      ├──[10 A fuse]──► HW-674 #1 ──► 5.1 V ──────────────────┐
      │                                                        │
      │                        ┌───────────────────────────────┴────────┐
      │                        │  AT THE ROCK 5B HEADER, not at the buck│
      │                        │                                        │
      │                        │   +5.1 V ─┬────────┬────────┬──► pin 2 │
      │                        │           │        │        │    pin 4 │
      │                        │        [TVS]   [1000 µF]  [100 nF]     │
      │                        │        band↑    (+)↑       │           │
      │                        │           │        │        │           │
      │                        │   GND ────┴────────┴────────┴──► pins 6,9,│
      │                        │                                  14,20,25 │
      │                        └────────────────────────────────────────┘
      │
      ├──[7.5 A fuse]──[KILL SWITCH]──► HW-674 #2 ──► 6.5 V ──┐
      │                                                        │
      │                        ┌───────────────────────────────┴────────┐
      │                        │  AT THE TB6612 VM PIN, not at the buck │
      │                        │                                        │
      │                        │   +6.5 V ─┬──────────┬──► TB6612 VM    │
      │                        │           │          │                 │
      │                        │      [1000 µF]   [100 nF]              │
      │                        │        (+)↑         │                  │
      │                        │           │          │                 │
      │                        │   GND ────┴──────────┴──► TB6612 GND   │
      │                        └────────────────────────────────────────┘
      │
      ├──[3 A fuse]────► LM2596 #1 ──► 5.0 V ──┬──[100 µF]──► ESP32 drive VIN
      │            (module already has 220 µF)  ├──[100 µF]──► ESP32 sense VIN
      │                                         └────────────► OLED, lidar
      │
      └──[2 A fuse]────► LM2596 #2 ──► 5.0 V ─────────────────► servos (Phase 5)
                   (kept off the logic rail deliberately —
                    servo inrush browns out ESP32s)
```

**Why "at the load, not at the buck".** A capacitor works against the inductance
of the wire between it and the thing it is feeding. Put the 1000 µF at the
regulator output and 30 cm of Dupont wire later, the ESP32 or the Rock 5B sees
almost none of its benefit. Same for the TVS: a fault clamped 30 cm away still
arrives at the header as a spike.

**What to actually fit.** Your modules already carry most of the decoupling they
need — the LM2596 has a 220 µF output cap, the ESP32 DevKit has ceramics around
its AMS1117, and the GY-521 has a 100 nF on VCC. Adding more at those same points
buys almost nothing. Two places genuinely need parts:

| Where | Fit | Verdict |
|---|---|---|
| **TB6612 VM pin** | 1000 µF/16 V + 100 nF | **Essential.** The only node on the robot switching amps; direction reversal dumps inductive energy straight back into the rail |
| **Rock 5B header** | 1000 µF/16 V + 100 nF + **TVS 6.0 V** | **Essential.** 4–5 A at the far end of a wire, on a path that bypasses the board's own input protection |
| Each ESP32 VIN | 100 µF | **Worth it.** The ESP32 is the thing that must not reset when the motors stall; its onboard 10 µF is thin for that |
| LM2596 output | — | **Skip.** The module already has 220 µF there |
| ESP32 VIN ceramic | — | **Skip.** VIN feeds a *linear* regulator that has its own ceramics. The high-frequency argument applies to switching loads like the TB6612, not to an AMS1117 |
| MPU-6050 VCC | — | **Optional.** The GY-521 already has one. Belt-and-braces if you want it; it costs nothing |

So: **two electrolytics, two ceramics, one TVS, plus a 100 µF per ESP32.** Not
the shopping list the diagram above might suggest.

---

## 2. Drive board — ESP32 #1, serial `KOBOLD-DRIVE`

Motors, encoders, IMU, **cliff sensors**, battery. 100–200 Hz hard real-time.

### 2.1 Pin allocation

| Function | Pin | Notes |
|---|---|---|
| **Motors — TB6612FNG** | | |
| PWM left | GPIO 4 | LEDC, 20 kHz (above audible) |
| LIN1 / LIN2 | GPIO 5 / 16 | |
| PWM right | GPIO 17 | |
| RIN1 / RIN2 | GPIO 18 / 19 | |
| STBY | GPIO 23 | **LOW = coast all motors.** The software e-stop |
| **Encoders (LM393, D0 only)** | | |
| Front-left / front-right | GPIO 34 / 35 | Input-only. Interrupt on RISING |
| Rear-left / rear-right | GPIO 36 / 39 | Input-only |
| **Cliff sensors — 4× IR facing DOWN** | | |
| Cliff FL / FR | GPIO 13 / 14 | On this board on purpose — §2.3 |
| Cliff BL / BR | GPIO 25 / 26 | |
| **I²C (MPU-6050)** | | |
| SDA / SCL | GPIO 21 / 22 | 3.3 V, 400 kHz |
| **Misc** | | |
| Battery sense | GPIO 32 | **ADC1.** 47 k / 10 k divider |
| **SAFETY_IN** from sense board | GPIO 27 | Hardware interrupt → immediate coast. §4 |
| Spare | GPIO 33 | |

Total: 15 of 16 safe pins + all 4 input-only. Tight but it fits.

**This allocation is identical whether you fit one TB6612FNG or two.** The
firmware drives one PWM and one direction pair per *side* plus `STBY`; it has no
idea how many chips are behind those signals. See §2.4.1.

### 2.2 Why per-side motor wiring

On a skid-steer chassis the four wheels are mechanically coupled through the
floor — front and rear on a side can never usefully differ. So tie them: one PWM
signal and one direction pair per side.

That's 7 pins instead of 13, and it's the better control design anyway. Per-wheel
PID on a skid steer has the wheels fighting each other through the carpet;
per-side velocity PID is what differential drive actually wants.

### 2.3 Why the cliff sensors are on this board

You want to drive on a table. That makes cliff detection the highest-priority
safety input in the system, and it has a hard deadline.

At 0.3 m/s with a sensor mounted 5 cm ahead of the wheel you have **166 ms** from
detection to the wheel leaving the table. A round trip through USB → serial
bridge → ROS 2 → Nav2 → back is 50–200 ms on a good day. That doesn't fit.

So the four downward IRs go straight to the drive board, and the drive firmware
stops the motors in **under 1 ms** without asking anyone's permission. The SBC
finds out afterwards.

**Table-driving rules for the firmware:**

- **Any** cliff sensor triggering → immediate coast (STBY low), then latch a
  `CLIFF` fault.
- Recovery: back away **in the direction of the sensors still reading "table"**,
  slowly, then stop and report. Never resume automatically.
- **Cap speed at 0.15 m/s in table mode.** Both detection distance and stopping
  distance scale with speed, and you have very little of either.
- Rear cliff sensors matter as much as the front — the cat game reverses
  constantly.
- Calibrate the Flying-Fish potentiometers **on your actual table**. Gloss
  reflects specularly, dark matte absorbs; both fool IR, in opposite directions.

**Known gap:** with all four corner IRs pointing down, the only horizontal
close-range sensing is the front and rear IR plus four ultrasonics with a ~15°
cone. The **sides have no close-range coverage**, and skid steer sweeps sideways
during every turn. Either turn slowly near obstacles, or add two more IR modules
(~€4) for left/right.

### 2.4 Schematics

Six independent blocks. Build and test them in this order — **everything except
the motors is verifiable with nothing but a USB cable:**

**IMU → encoders → cliff → battery sense → safety line → motors.**

#### 2.4.1 Motors — TB6612FNG

**One driver or two?** One is enough to bring the drivetrain up, and the wiring
below is for one. The difference is only current headroom:

| | Two drivers (design target) | **One driver (build this now)** |
|---|---|---|
| H-bridges | 4, one per **motor** | 2, one per **side** |
| Left pair | split across driver #1 A+B | **paralleled** on channel A |
| Right pair | split across driver #2 A+B | **paralleled** on channel B |
| Headroom | 1.2 A per motor | 1.2 A per **side** |

```
   6.5 V motor rail ──┬──[1000 µF]──┬──► VM ─┐
                      │    (+)↑     │        │
                      │             │        │   TB6612FNG
                      │          [100 nF]    │  ┌──────────────┐
                     GND ───────────┴────────┼──┤ GND      AO1 ├──► LEFT motors (both)
                                             └──┤ VM       AO2 ├──►   parallel, same polarity
   ESP32 3V3 ────────────────────────────────┬──┤ VCC          │
                                             │  │          BO1 ├──► RIGHT motors (both)
   ESP32 GPIO 4  ────────────────────────────┼──┤ PWMA     BO2 ├──►   parallel, same polarity
   ESP32 GPIO 5  ────────────────────────────┼──┤ AIN1         │
   ESP32 GPIO 16 ────────────────────────────┼──┤ AIN2         │
   ESP32 GPIO 17 ────────────────────────────┼──┤ PWMB         │
   ESP32 GPIO 18 ────────────────────────────┼──┤ BIN1         │
   ESP32 GPIO 19 ────────────────────────────┼──┤ BIN2         │
   ESP32 GPIO 23 ────────────────────────────┼──┤ STBY         │
   ESP32 GND ────────────────────────────────┴──┤ GND          │
                                                └──────────────┘
```

With two drivers, the same six signals fan out to both chips — PWM left to
PWMA+PWMB of driver #1, etc. Nothing else changes.

**Current, with two motors per channel.** TT gear motors draw ~150 mA
free-running and 700 mA–1 A stalled at 6 V:

| | Per channel | vs 1.2 A continuous / 3.2 A peak |
|---|---|---|
| Free-running | ~300 mA | comfortable |
| Normal driving | ~400–600 mA | comfortable |
| **Both wheels on a side stalled** | **~1.4–2 A** | over continuous, under peak |

Fine for everything except a *sustained* stall — a carpet edge, a jammed wheel,
driving into a wall and holding throttle. The TB6612FNG has proper thermal
shutdown rather than the L293D's gradual fade, so it protects itself; you lose
the motors until it cools.

**Three mistakes that cost an afternoon:**

- **STBY floating = motors permanently coasted.** It must be driven HIGH to
  enable, and an unconnected pin is indistinguishable from a dead driver. This is
  the single most common "nothing happens and I've checked everything".
- **VM and VCC are not interchangeable.** VM is the 6.5 V motor rail, VCC is
  3.3 V logic. VCC on the motor rail destroys the chip.
- **Check polarity when paralleling.** One motor wired backwards makes that side
  fight itself, drawing near-stall current while barely moving.

Grounds must be **common** between the ESP32, the driver and the motor rail.

#### 2.4.2 Encoders — 4× LM393, powered at 3.3 V

```
   ESP32 3V3 ──┬──► VCC ─┐
               │         │  LM393 module
               │         │ ┌─────────┐
              GND ───────┼─┤ GND  D0 ├──► ESP32 GPIO 34  (front-left)
                         └─┤ VCC  A0 ├──   leave unconnected
                           └─────────┘
                                          GPIO 35  front-right
                                          GPIO 36  rear-left
                                          GPIO 39  rear-right
```

**Power them from 3.3 V, not 5 V.** D0 swings to whatever VCC is, and GPIO 34–39
have no clamping diodes — 5 V on those pins damages the ESP32. This is the one
place on this board where getting the supply wrong is destructive rather than
merely wrong.

A0 is the analogue output, useful only for diagnosing a dirty or misaligned
slotted disc. Leave it off.

#### 2.4.3 Cliff sensors — 4× IR, facing DOWN at the corners

```
   ESP32 3V3 ──┬──► VCC ─┐
               │         │  IR Flying-Fish
              GND ───────┼─┤ GND  OUT ├──► ESP32 GPIO 13  (front-left)
                         └─┤ VCC      │                14  front-right
                           └──────────┘                25  rear-left
                                                       26  rear-right
```

3.3 V, same reasoning as the encoders.

#### 2.4.4 IMU — MPU-6050

```
                       MPU-6050 (GY-521)
                      ┌──────────────┐
   ESP32 3V3 ─────────┤ VCC      INT ├──  unused
                      │              │
   ESP32 GND ─────────┤ GND          │
                      │              │
   ESP32 GPIO 21 ─────┤ SDA          │
   ESP32 GPIO 22 ─────┤ SCL          │
              GND ────┤ AD0          │   LOW → address 0x68
                      └──────────────┘
```

**AD0 decides the address.** Low = 0x68, which is what the firmware expects. Tie
it high and the chip answers at 0x69, `WHO_AM_I` fails, and you get
`MPU-6050 not responding` with everything apparently wired correctly. Most
GY-521 boards already pull AD0 low, so leaving it unconnected usually works —
tying it to GND removes the doubt.

Mount it **flat, near the centre of rotation, on foam, away from the motors.**

#### 2.4.5 Battery sense — resistor divider into ADC1

```
   3S pack + (12.6 V max)
        │
      [47 kΩ]
        │
        ├──────────► ESP32 GPIO 32   (ADC1)
        │
      [10 kΩ]
        │
       GND  (common with pack ground)
```

12.6 V × 10/(47+10) = **2.21 V** at full charge, inside ADC1's range.

Use **ADC1 only** — ADC2 is unavailable whenever WiFi is active, and that failure
looks like a battery reading that works on the bench and dies in the field.
Calibrate against a multimeter; the ESP32 ADC is noticeably nonlinear near both
rails.

#### 2.4.6 Hardware safety line — in from the sense board

```
   sense GPIO 25 ──────────wire──────────► drive GPIO 27
   sense GND     ──────────wire──────────► drive GND      ◄── run BOTH wires
```

The ground wire is not optional. A logic signal between two boards sharing only a
power-supply ground is a signal referenced to whatever the motor current is doing
to that ground at the time.

**Polarity, as implemented:** the sense board drives the line LOW to assert danger
and *releases it to high-impedance* to clear. The drive board holds
`INPUT_PULLUP` with a falling-edge interrupt, so the pull-up defines idle. That is
deliberate — two boards can never fight over the line, and a sense board
mid-reboot cannot hold the drive board stopped.

⚠️ **The consequence is that this line fails OPEN.** A broken wire, a pulled
connector, or a dead sense board all read as "no danger", and the hardware e-stop
is silently gone. Nothing above the MCUs catches it either — going through the
SBC is exactly what this wire exists to avoid.

Check continuity when you build it, and treat this as a known limitation. The
cheap fix, when the sense firmware next gets attention, is a **heartbeat**: have
the sense board toggle the line at a few Hz while safe rather than idling, and
have the drive board fault if the transitions stop. Same wire, same
sub-millisecond assert path, but a cut wire becomes detectable instead of
invisible.

---

## 3. Sense board — ESP32 #2, serial `KOBOLD-SENSE`

Ultrasonics, horizontal IR, buzzer, OLED. 20–50 Hz.

### 3.1 Pin allocation

| Function | Pin | Level |
|---|---|---|
| Ultrasonic TRIG ×4 (shared) | GPIO 23 | 3.3 V out, fired **sequentially** |
| Ultrasonic ECHO front / back | GPIO 34 / 35 | **5 V → divider**, input-only pin |
| Ultrasonic ECHO left / right | GPIO 36 / 39 | **5 V → divider**, input-only pin |
| Horizontal IR front / back | GPIO 13 / 14 | 3.3 V |
| I²C SDA / SCL (OLED, 0x3C) | GPIO 21 / 22 | 3.3 V |
| Buzzer (via NPN) | GPIO 5 | 3.3 V |
| **SAFETY_OUT** → drive board | GPIO 25 | Released high-impedance to clear. §4 |
| Spare | 4, 16, 17, 18, 19, 26, 27, 32, 33 | PIR, servos later |

### 3.2 Firmware behaviour

- **Sequential ranging**, ~60 ms cycle: fire, wait for echo or timeout, next sensor.
  Four sensors pinging together hear each other and return confident nonsense.
- **Median-of-3** per range. HC-SR04s throw wild outliers, and one bad reading
  triggering an e-stop mid-drive is maddening.
- Publish "no echo" as `+inf`, **never `0`** — a zero reads downstream as
  "obstacle touching the sensor" and hard-stops the robot for no reason.
- Assert SAFETY_OUT on any range below the danger threshold (§4).

### 3.3 Schematics

#### 3.3.1 Ultrasonics — 4× HC-SR04, and the one that bites

HC-SR04 needs **5 V** to work reliably, and its ECHO pin therefore swings to 5 V.
GPIO 34–39 have no clamping diodes. **ECHO must be divided down or level-shifted
— never wired directly.**

```
   5 V logic rail ──────► VCC ─┐
                               │   HC-SR04
                               │  ┌──────────┐
   ESP32 GPIO 23 ──────────────┼──┤ TRIG     │      (all four TRIG tied together)
                               │  │          │
                               └──┤ VCC ECHO ├──┐
                     GND ─────────┤ GND      │  │
                                  └──────────┘  │
                                                │
                                  ┌─────────────┘
                                  │
                               [10 kΩ]
                                  │
                                  ├──────────► ESP32 GPIO 34   (front)
                                  │
                               [20 kΩ]
                                  │
                                 GND
```

5 V × 20/(10+20) = **3.33 V**. One divider per sensor — four dividers total.

| Sensor | ECHO → |
|---|---|
| Front | GPIO 34 |
| Back | GPIO 35 |
| Left | GPIO 36 |
| Right | GPIO 39 |

TRIG is 5 V-tolerant as an input on the HC-SR04 and 3.3 V drives it fine, so all
four TRIG pins share **GPIO 23** directly, no divider. The firmware fires them
**sequentially** — four sensors pinging together hear each other and return
confident nonsense.

#### 3.3.2 Horizontal IR — 2× forward/rearward facing

```
   ESP32 3V3 ──┬──► VCC ─┐
               │         │  IR module
              GND ───────┼─┤ GND  OUT ├──► ESP32 GPIO 13   (front)
                         └─┤ VCC      │                14   (back)
                           └──────────┘
```

3.3 V, same as the cliff sensors on the drive board. These face **horizontally** —
the downward-facing ones are on the drive board deliberately (§2.1).

#### 3.3.3 OLED — SSD1306 over I²C

```
                        SSD1306
                      ┌───────────┐
   ESP32 3V3 ─────────┤ VCC       │
   ESP32 GND ─────────┤ GND       │
   ESP32 GPIO 21 ─────┤ SDA       │   address 0x3C
   ESP32 GPIO 22 ─────┤ SCL       │
                      └───────────┘
```

Most SSD1306 modules run on 3.3 V and have their own pull-ups. If you ever put
the OLED and something else on this bus, only **one** device should provide
pull-ups.

#### 3.3.4 Buzzer — via NPN, not driven directly

```
   5 V ──────────[buzzer]──┐
                           │
                           C
   ESP32 GPIO 5 ──[100 Ω]──B   S9013 NPN
                           E
                           │
                          GND
```

An active buzzer draws 30–50 mA — beyond what an ESP32 pin should source, and
inductive kick from a passive buzzer is worse. The 100 Ω limits base current;
the transistor does the work.

If you use a **passive** (magnetic) buzzer, add a flyback diode across it,
cathode to +5 V.

#### 3.3.5 Safety line — out to the drive board

```
   sense GPIO 25 ──────────wire──────────► drive GPIO 27
   sense GND     ──────────wire──────────► drive GND
```

See §2.4.6 for polarity and the fail-open caveat.

### 3.4 Build-order checklist — both boards

Both boards, in the order that keeps each step independently testable:

| # | Step | Testable without | How you know it worked |
|---|---|---|---|
| 1 | Flash both boards | anything | `flash.sh both` reports fw + proto |
| 2 | MPU-6050 → drive | motors, ROS | `MPU-6050 not responding` stops appearing |
| 3 | Encoders → drive | motors | tick counts change when you spin a wheel by hand |
| 4 | Cliff IR → drive | motors | fault flag sets when you lift the corner off the table |
| 5 | Battery divider → drive | motors | reported mV tracks a multimeter |
| 6 | Ultrasonics → sense | motors | ranges track a hand moved in front |
| 7 | Safety line | motors | asserting on sense sets the fault on drive |
| 8 | **TB6612 + motors** | — | wheels turn, and STBY low coasts them |

Steps 2–7 need only a USB cable. Do the motor driver last, once everything that
can stop it is already proven.

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

**Wiring, polarity, and the fail-open caveat are in §2.4.6.** Read it before you
rely on this — the line is active-low and released to high-impedance, so a cut
wire reads as "no danger".

---

## 5. USB and device naming

All boards use USB-serial bridges, so everything is `/dev/ttyUSB*`.

**Both ESP32s turned out to be CP2102 with the factory-default serial `0001`** —
identical idVendor, idProduct *and* serial. An earlier version of this section
claimed the USB-C board had a distinct CH9102 VID/PID; that was written before
anything was plugged in and is simply wrong. There was nothing electrically
unique to match on.

Fixed by rewriting the CP2102 EEPROM serial on each board, so identity follows
the **board** rather than the socket:

```bash
git clone https://github.com/DiUS/cp210x-cfg && cd cp210x-cfg && make
sudo ./cp210x-cfg -l                              # find bus/dev
sudo ./cp210x-cfg -d <bus>.<dev> -S KOBOLD-DRIVE
sudo ./cp210x-cfg -d <bus>.<dev> -S KOBOLD-SENSE
```

Two traps: `-d` wants `n.n` despite the help text saying `bus:dev`, and the
device re-enumerates mid-write, so the tool prints `No such device` read errors
*after a successful write*. Verify with `cp210x-cfg -l`, not the exit code.

| Device | Serial | Symlink |
|---|---|---|
| ESP32 drive | `KOBOLD-DRIVE` | `/dev/robot-drive` |
| ESP32 sense | `KOBOLD-SENSE` | `/dev/robot-sense` |
| ESP32 head (spare) | still factory `0001` | give it one before fitting |

```
# /etc/udev/rules.d/99-kobold.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="KOBOLD-DRIVE", SYMLINK+="robot-drive", MODE="0660", GROUP="dialout"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="KOBOLD-SENSE", SYMLINK+="robot-sense", MODE="0660", GROUP="dialout"
```

Verified 2026-08-04: moving the sense board to a different USB *host controller*
(bus 3 → bus 5) still produced `/dev/robot-sense`. Under the previous
port-matching rules it would have matched nothing at all.

⚠️ **Do not write a bare `10c4:ea60` rule for a lidar.** An earlier revision did,
and because the ESP32s use the same CP2102 bridge it grabbed them both —
`/dev/robot-lidar` pointed at an ESP32 and the second board silently overwrote
the first one's symlink. Match a lidar on its own serial.

**Backstopped in software.** The firmware announces its board id in the version
frame at boot, and `kobold_bridge` refuses to talk to a board whose id is not the
one that port is meant to hold. Verified against real firmware — pointing the
drive expectation at the sense port produces a refusal, not motor commands.

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

### 8.1 Breadboard or solder? Both, in that order

**Phase 1 — breadboard, for logic bring-up only.** IMU, encoders, cliff IR, the
safety line. Everything in the build-order checklist up to step 7. Fast to
change, and being able to move a wire in five seconds is worth a lot while you
are still finding out what works.

**Phase 2 — soldered, before the robot ever moves.** Vibration plus spring
contacts produces intermittent faults that look exactly like software bugs.

**Never on a breadboard: motor current, and the decoupling capacitors.**

Two separate reasons, and the second is the one people miss:

- A breadboard contact is **10–50 mΩ**, and it degrades with use. Two or three
  contacts in the motor path at 2 A is a few hundred millivolts lost as heat, in
  a spring you cannot inspect.
- **A 100 nF on a breadboard is not a 100 nF.** Lead length plus two spring
  contacts adds roughly 20–30 nH, which drags its self-resonance down from tens
  of MHz to around 3 MHz. Above that it is an inductor. You added the ceramic
  specifically to cover the frequencies the bulk cap cannot reach, and mounting
  it this way puts it right back in the same band. It looks connected, measures
  connected, and does nothing.

### 8.2 Where each capacitor physically goes

Only the parts §1.3 says to fit — the rest is already on your modules.

| Cap | Mount it | Lead length |
|---|---|---|
| 1000 µF + 100 nF at TB6612 | Carrier board, at the module's VM/GND header pins | 100 nF closest |
| 1000 µF + 100 nF + TVS at Rock 5B | Small adapter board on the GPIO header | as short as physically possible |
| 100 µF at each ESP32 | Carrier board, at the VIN/GND header pins | few cm is fine |

The rule everywhere: **ceramic nearest the pins, electrolytic behind it.** If you
have to choose which one gets the good position, the ceramic wins — the
electrolytic still works from a few centimetres away, the ceramic does not.

### 8.3 Carrier board layout

Perfboard, not a custom PCB. Roughly 7 × 9 cm:

```
   ┌──────────────────────────────────────────────────┐
   │  ┌────────────┐        ┌──────────────┐          │
   │  │  ESP32     │        │  TB6612FNG   │          │
   │  │  (female   │        │  (female     │  [screw  │
   │  │   headers) │        │   headers)   │   term]  │◄─ 6.5 V in
   │  │            │        │              │          │
   │  └────────────┘        └──────────────┘  [screw  │
   │    ▲                     ▲   ▲            term]  │◄─ motors out
   │    │                     │   │                   │
   │  100 µF               1000 µF                    │
   │  + 100 nF             + 100 nF                   │
   │  at VIN               at VM                      │
   │                                                  │
   │  [pin headers for sensors: encoders, cliff, IMU] │
   └──────────────────────────────────────────────────┘
```

**Female headers for both modules**, so the ESP32 and the driver stay removable —
you will damage one eventually, and desoldering a 30-pin module you cooked is a
bad afternoon.

**Screw terminals for anything carrying motor current.** Dupont connectors are
crimped onto 26–28 AWG and rated around 0.5 A; they are for signals.

### 8.4 Wire gauge — Dupont is not enough for power

| Path | Current | Minimum |
|---|---|---|
| Pack → bucks | up to 8 A | **18 AWG** |
| Motor rail → TB6612 VM | 2–4 A | **18 AWG** |
| TB6612 → each motor | 0.5–1 A | **22 AWG** |
| Buck → Rock 5B header | 4–5 A | **18 AWG**, both 5 V pins |
| Logic 5 V → ESP32s | < 0.5 A | 24 AWG |
| Every signal line | mA | 26–28 AWG (Dupont fine) |

The docs already warn that 4 A through thin Dupont drops 300 mV easily. That is
5.1 V at the buck arriving as 4.7 V at the Rock 5B — instability you will spend a
day blaming on software.


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
