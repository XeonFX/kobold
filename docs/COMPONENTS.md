# Component Inventory

Every part you own, what it does in this build, and what it's good for if it isn't used now.

**Status legend:** 🟢 on the robot · 🔵 off-robot infrastructure · 🟡 reserve / future use · ⚪ shelved

---

## 1. Compute — single-board computers

| # | Component | Status | Role | Notes |
|---|---|---|---|---|
| 1 | **Radxa Rock 5B** — RK3588S, 8 GB RAM, 64 GB eMMC, active cooling, free M.2 | 🟢 | **Robot brain.** ROS 2, perception, LLM, web app — all of it | 6 TOPS NPU is the whole reason this project is feasible. Only board you own supported by RKLLM/RKNN. OS on eMMC, Docker + models on NVMe |
| 2 | **Odroid N2** — S922X, 4 GB RAM, 16 GB eMMC | 🔵 | **Home base station.** Docker registry, MQTT, rosbag archive, Foxglove, Grafana | Runs on mains 24/7. Good CPU, no useful NPU — infrastructure, not inference. Host the 2 TB NVMe here over USB 3 |
| 3 | **Radxa Zero 3W** — RK3566, 2 GB RAM, 64 GB microSD, **MIPI CSI + ~1 TOPS NPU** | 🟡 | **Smart room node** — fixed camera + mic in a corner of the room | Better than first credited: it can run RKNN vision models on its own NPU and publish *detections* over MQTT instead of streaming raw video. Gives the robot a second viewpoint (finds the cat when the cat isn't in the robot's FOV) and a second listening point. ⚠️ **RKLLM does not support RK3566** — vision yes, LLM no. Also still a fine charging-dock controller |
| 4 | **Raspberry Pi Zero 2W** — 512 MB RAM, 64 GB microSD | 🟡 | Second, dumber room node | Stream-only, no NPU. 512 MB won't run ROS 2 comfortably |
| 5 | **Raspberry Pi 3** — 1 GB RAM | 🟡 | Dev / test / scratch box | Somewhere to try risky things that shouldn't touch the robot |

---

## 2. Compute — microcontrollers

| # | Component | Status | Role | Notes |
|---|---|---|---|---|
**Correction:** all three are **ESP32 DevKit V1 (ESP-WROOM-32, 30-pin)** — there is no S3. One has
USB-C with a different USB-serial controller. All enumerate as `/dev/ttyUSB*`.

| 6 | **ESP32 DevKit V1 — USB-C variant** | 🟢 | **Drive controller.** Motors, per-side PID, 4 encoders, MPU6050, battery ADC, **4× cliff IR**, e-stop, watchdog | CP2102 bridge, serial reprogrammed to `KOBOLD-DRIVE` — see below |
| 7 | **ESP32 DevKit V1 #2** | 🟢 | **Sensor hub.** 4× ultrasonic, 2× horizontal IR, buzzer, OLED, safety line | GPIO 34/35/36/39 are input-only — perfect for ultrasonic ECHO. Keeps µs-precision echo timing away from encoder interrupt storms |
| 8 | **ESP32 DevKit V1 #3** | 🟡 | **Head unit** (Phase 5+): pan/tilt servos, PIR, second OLED. Also the **spare** | Don't populate until needed — an identical spare on the shelf beats a third USB cable |
| 9 | **Genuino 101** — Intel Curie | ⚪ | **Shelved** | EOL since 2017. No modern toolchain, no ESP-IDF, no micro-ROS. Only distinguishing feature is onboard BLE + 6-axis IMU if you ever want a standalone wireless gadget. It *is* Arduino-Uno form factor, so it's the only board the HW-130 shield plugs into directly |


### Telling the three ESP32s apart

All three are the same board with the same USB bridge, and they arrived
indistinguishable. Read off the hardware 2026-08-04:

```
idVendor 10c4   idProduct ea60   manufacturer "Silicon Labs"
product "CP2102 USB to UART Bridge Controller"   serial "0001"    <-- on BOTH
```

An earlier note here claimed the USB-C board had a distinct VID/PID. It does
not — that was written before the boards were plugged in. Nothing electrically
distinguishes them.

**Fixed by reprogramming the CP2102 EEPROM serial**, so identity follows the
board into any socket rather than following the socket:

```bash
git clone https://github.com/DiUS/cp210x-cfg && cd cp210x-cfg && make
sudo ./cp210x-cfg -l                              # find bus/dev
sudo ./cp210x-cfg -d <bus>.<dev> -S KOBOLD-DRIVE
```

Two traps: `-d` wants `n.n` despite the help text saying `bus:dev`, and the
device re-enumerates mid-write, so the tool prints `No such device` read errors
*after a successful write*. Verify with `cp210x-cfg -l`, not the exit code.

| Board | Serial | Symlink |
|---|---|---|
| drive | `KOBOLD-DRIVE` | `/dev/robot-drive` |
| sense | `KOBOLD-SENSE` | `/dev/robot-sense` |
| head (spare) | still factory `0001` | give it one before fitting |

**Backstopped in software.** The firmware announces its board id in the version
frame at boot, and `kobold_bridge` refuses to talk to a board whose id is not
the one that port is meant to hold. Before this, `board_id` was logged and never
checked — a swapped cable would have sent motor commands to the ultrasonic
board in silence.

---

## 3. Storage

| # | Component | Status | Role | Notes |
|---|---|---|---|---|
| 10 | **Samsung PM991 256 GB** M.2 NVMe **(2280 ✓)** | 🟢 | **Robot NVMe** — `/data/containerd`, models, maps, ring-buffer rosbag | Containerd's snapshotter ignores Docker's `data-root`; its own root is explicitly set to `/data/containerd`. Chosen for power: ~2.5 W under load vs 6–7 W for the PM9A1 |
| 11 | **Samsung PM9A1 2 TB** M.2 NVMe | 🔵 | **Archive** on the Odroid N2 via USB 3 enclosure: registry, rosbags, datasets, model versions | Far too power-hungry for a battery robot, and rosbags belong off-robot anyway |
| 12 | **ADATA SX8200 Pro 512 GB** M.2 NVMe | 🟡 | Spare / desktop | SM2262EN controller — the hottest-running of the three. Poor fit for a sealed chassis |
| 13 | **64 GB eMMC** (on Rock 5B) | 🟢 | Root filesystem | Keep the OS here: recoverable with a card reader if you brick it |
| 14 | **2× Lexar Silver Plus 64 GB microSD** | 🟡 | OS for Zero 3W / Pi Zero 2W | In use if those boards get deployed |
| 15 | **Lexar LPAH100 M.2 heatsink** + thermal pads | 🟢 | Fitted to the PM991 | Rock 5B puts the M.2 slot on the **underside** — a thermal dead zone with no airflow and no fan coverage. Idles at 40–42 °C with the heatsink |

### The PM991 falls off the PCIe bus without two kernel parameters

Observed 2026-08-04: after ~90 minutes idle the controller stopped answering.

```
nvme nvme0: controller is down; will reset: CSTS=0xffffffff, PCI_STATUS=0x10
nvme nvme0: Removing after probe failure status: -19
nvme0n1: detected capacity change from 500118192 to 0
```

Every write to `/data` then returns `EIO` while `df` still reports the old free
space from a cached superblock — an unusually confusing failure mode, because
the mount looks healthy right up until you touch it.

**The fix** — applied automatically by `scripts/robot-setup.sh`, appended to
`/etc/kernel/cmdline` and regenerated into `extlinux.conf` with `u-boot-update`:

```
nvme_core.default_ps_max_latency_us=0 pcie_aspm=off
```

The first disables the drive's autonomous power state transitions; the second
stops the PCIe link being powered down underneath it. On RK3588 the root complex
cannot reliably bring either back.

**Thermal was investigated and ruled out**, not assumed. SMART lifetime counters
after the incident:

| Counter | Value | Meaning |
|---|---|---|
| `Thermal Management T1/T2 Total Time` | **0** | drive has never throttled itself |
| `Critical Composite Temperature Time` | **0** | never reached `cctemp` (85 °C) |
| `Warning Temperature Time` | 36 min | lifetime, on a used OEM pull — predates this build |
| `media_errors` | 0 | no data damage |

A hot-to-the-touch drive at the time was a *consequence*, not the cause: once
the nvme driver detaches, the slot keeps supplying 3.3 V and nothing is left
managing the controller's power or thermals.

The 1,373 entries in the drive's error log are all `sqid 0` /
`0x2002 Invalid Field in Command` — the host probing an admin feature this
firmware does not implement. Benign.

**Also secure the card mechanically.** An M.2 2280 held only by its connector
lifts when the board is moved, and on a robot that vibrates continuously that is
its own source of the identical `CSTS=0xffffffff` signature. Standoff screw at
the 80 mm position is mandatory, not optional.

---

## 4. Networking

| # | Component | Status | Role | Notes |
|---|---|---|---|---|
| 15 | **Radxa Wireless A8** (RTL8852BE, M.2 E-key) | 🟢 | **Robot WiFi** — first choice | Vendor-validated for the Rock 5B, so it'll work out of the box. Antennas confirmed present ✓ |
| 16 | **Intel AX210NGW** (M.2 E-key) | 🟡 | **WiFi upgrade path** | `iwlwifi` is mainline and rock-solid on ARM — swap this in if the RTL8852BE proves flaky under sustained load. Needs 2× MHF4 antennas (not included) |
| 17 | **Cudy WU650 USB** (RTL8811cu) | 🟡 | Last-resort dongle / AP mode on a Pi | Needs an out-of-tree driver that breaks on kernel updates. Avoid on the robot |
| 18 | Rock 5B onboard 2.5 GbE | 🟢 | Bench debugging, bulk transfers, initial setup | Your lifeline when WiFi misbehaves |

---

## 5. Cameras and vision

⚠️ **The ROCK 5B has ONE MIPI CSI connector** (plus one DSI) — the 5B**+** is the two-CSI variant.
So only one of these goes on the robot, and the €0 stereo idea is dead. That single connector is
the *whole* reason: contrary to an earlier note here, the two modules' optics turn out to be nearly
identical (62° vs 62.2° horizontal), which would have made them a perfectly serviceable stereo pair
had there been somewhere to plug the second one in. See PROJECT_PLAN §6.

| # | Component | Status | Role | Notes |
|---|---|---|---|---|
| 19 | **Radxa Camera 8M 219** (IMX219) | 🟢 | **Robot camera** | 2.95 mm, **f/2.5**, FOV **D=74° H=62° V=49°**, TV-distortion <0.3%, 1G4P, 32×32 mm. **15-pin 1.0 mm** FPC. Verified working end to end 2026-08-04 |
| 20 | **Arducam B0390** (IMX219) | 🟡 | Spare — **cannot connect to the Rock 5B** | 3.04 mm, **f/2.0**, FOV **H=62.2° V=48.8°**, 25×24 mm. Better optics, but a **22-pin 0.5 mm** socket: needs a non-standard 22→15 adapter cable. See below |
| 21 | **Radxa Camera 4K** (IMX415) | 🔴 | **Tested 2026-08-04 — does not stream on this board** | Better sensor on paper (1/2.8″, 1.45 µm pixels, **75° H FOV**, M12 swappable lens) but the MIPI link fails with continuous ECC errors. See below |
| 22 | **Intel Neural Compute Stick 2** (Myriad X) | ⚪ | **Shelved — do not spend time on this** | OpenVINO dropped Myriad X support after 2022.3, and ARM64 host support was never good. The RK3588's NPU is faster, better supported, and already in the robot |

### Why the Radxa wins — connector, not optics

**Decision: Radxa Camera 8M 219.** The Arducam has genuinely better optics (see below)
but carries a **22-pin 0.5 mm** connector on the camera board, while the Rock 5B has a
**15-pin 1.0 mm** CSI socket. Connecting it needs a 22→15 adapter cable oriented
narrow-at-camera — the reverse of a normal Pi Zero cable, and not something to assume
is wired correctly without testing.

That settles it, and for a better reason than convenience: **camera ribbons work loose on
a machine that vibrates.** The Radxa uses a commodity 15-pin cable that can be replaced
from any parts drawer. The Arducam would put a special-order adapter on the critical path
of the robot's only camera — a part needing its own spare and its own re-supply. Two-thirds
of a stop does not justify that on a moving platform.

Revisit only if detection accuracy actually disappoints in Phase 4, and treat the adapter
cable as the cost of that experiment.

### The optics comparison, for the record

Both modules are the **same sensor** (IMX219, 3280×2464, 1/4″, rolling shutter) on the **same
driver** and the **same overlay** — so the swap costs nothing in *software*. The blocker is purely
mechanical (see above). Radxa publishes
no optics on its product page, which is why an empirical FOV test was originally planned — but the
[product brief](https://dl.radxa.com/accessories/camera-8m-219/radxa_camera_8m_219_product_brief_Revision_1.0.pdf)
has the numbers, and they are effectively the same lens: both frame ~120 cm of a tape measure at
1 m. A ruler cannot resolve 62° from 62.2°.

The one axis they differ on is **aperture: f/2.0 vs f/2.5 — 1.56× more light** to the Arducam.
That matters here specifically because indoor domestic lighting runs 50–200 lux, where a 1.12 µm
pixel is noise-limited, and because the robot is *moving while it looks*. The extra light buys
either a shorter exposure (less motion blur, which is what actually breaks detection and feature
tracking) or lower analogue gain (less noise into the NPU).

Marginal, though: two-thirds of a stop. The Radxa is the one with a *published* distortion figure,
so if monocular depth calibration later proves fussy, swapping back is cheap and worth trying.

### The Radxa Camera 4K does not work here — MIPI link failure

Tested 2026-08-04. On paper this is the better robot camera: **1/2.8″ sensor with
1.45 µm pixels (1.68× the area of the IMX219's 1.12 µm), 75° horizontal FOV against
62°, and an M12×P0.5 mount so the lens is swappable.** Wider view and better low light
are exactly what an indoor robot wants. It does not stream.

The sensor is alive — `imx415 3-001a: Detected imx415 id 0000e0` over I2C at address
0x1a — but the differential lanes deliver corrupt data continuously:

```
mipi2-csi2-hw ERR1:0x10000000 (ecc2)     ~4600 per capture attempt
```

`ecc2` is an uncorrectable 2-bit ECC error in the MIPI packet header. The ISP never
emits a frame; the pipeline sits in PLAYING until it times out.

Every configuration was tried, including a hand-built 2-lane overlay
(`rock-5b-radxa-camera-4k-2lane.dtbo`, kept disabled in `/boot/dtbo/`):

| Config | Lanes | Link rate | Frames | ECC errors |
|---|---|---|---|---|
| IMX219 (reference) | 2 | 456 MHz | 300 ✓ | 0 |
| IMX415 native | 4 | 891 MHz | 0 | 4663 |
| IMX415 1080p mode | 4 | 446 MHz | 0 | 4582 |
| IMX415 2-lane custom overlay | 2 | 446 MHz | 0 | 4660 |
| IMX415 reseated both ends | 4 | 891 MHz | 0 | 4663 |
| IMX415 ribbon reversed | 4 | 891 MHz | 0 | 4662 |

The 2-lane row is the same lane count and effectively the same clock as the working
IMX219, which **rules out signal integrity at high rates and rules out lane count**.
Reseating and reversing the ribbon changed nothing — and the sensor still answered on
I2C after reversal, so that flip did not alter the electrical mapping either.

The fault is in the cable or the module itself — neither substitutable, since there is
exactly one cable of that type. Not diagnosable further without a spare.

Note the connector differs from the IMX219 modules: the 4K camera and the Rock 5B share
the same socket type, so it uses a straight-through cable, while the 8M 219 needs a
15-pin-to-board cable.

**If a replacement cable ever turns up, retry** — the sensor is genuinely better and the
2-lane overlay is already built and waiting. Until then this is a dead component.

### Verified camera pipeline (2026-08-04)

Confirmed end to end on the Rock 5B with the Radxa module:

- Probes as `imx219 3-0010` — **I2C bus 3, address 0x10**. A reversed FPC yields no probe at all,
  so a successful probe is sufficient proof the ribbon is the right way round at both ends.
- Full chain works: sensor → `csi2-dphy0` → `rkcif-mipi-lvds2` → `rkisp0-vir0` → V4L2, with
  `rkaiq_3A` running. Captured 1920×1080 NV12 via GStreamer from **`/dev/video11`** (`rkisp_mainpath`).
- Sensor subdev is **`/dev/v4l-subdev2`**, exposing `horizontal_flip` / `vertical_flip`. Default
  orientation comes out **180° rotated**; set both to 1 once the module is bolted to the chassis.
  Note that flipping an IMX219 also changes the Bayer order — check colours after, and rotate
  downstream in the ROS node instead if they go wrong.
- **`rkaiq` auto-exposure converges once per stream, then holds — it does not track the scene.**
  Measured: within one session two completely different scenes both held `exposure`=2100 /
  `analogue_gain`=1536; across a restart the gain moved to 1146 and produced a correctly exposed
  frame (mean 115/255). Brightness was also flat across all 300 frames of a single capture, so
  there is no per-frame hunting.

  **This matters for Phase 4.** A robot driving from a lit room into a shadow will keep the
  exposure it converged on at stream start. Perception should either drive exposure explicitly
  through the sensor subdev controls, or restart the stream on large luminance shifts — not
  assume 3A adapts. Discarding the first ~20 frames after opening a stream remains cheap
  insurance while it settles.

---

## 6. Chassis and drivetrain

| # | Component | Status | Role | Notes |
|---|---|---|---|---|
| 23 | **4WD rectangular chassis** ([botland 7289](https://botland.com.pl/podwozia-robotow/7289-chassis-rectangle-4wd-4-kolowe-podwozie-robota-z-napedem-5904422310127.html)) | 🟢 | The platform. Skid-steer 4WD | Skid steer means the wheels scrub sideways on every turn — encoder-derived heading is unreliable by design. Gyro fusion is mandatory, not optional |
| 24 | **4× wheel encoders** — LM393 modules, VCC/GND/**D0**/A0 | 🟢 | Wheel velocity → linear odometry, per-side PID | 4-pin confirms **single-channel, no direction sensing** (inferred from PWM sign). ~20 slots/rev on a 65 mm wheel → **~10 mm per tick**, which is decent. **Power at 3.3 V** so D0 is ESP32-safe. Use D0 only; A0 is for diagnosing a dirty or misaligned disc |
| 25 | **HW-130 motor shield** (2× L293D + 74HC595) | 🟡 | **Currently in use at 8 V** — works, but see note | The 8 V trick compensates for the L293D's ~2 V drop with brute force, and the extra current means *more* heat in the chip, not less. Thermal shutdown fades in gradually, so the robot goes sluggish before it stops — a confusing failure. **Heatsink it now**, and replace with TB6612FNGs (same wheel voltage at 6.5 V, a quarter of the waste, double the current headroom, no level shifters) |

---

## 7. Sensors

| # | Component | Status | Role | Notes |
|---|---|---|---|---|
| 26 | **4× HC-SR04 ultrasonic** (F/B/L/R, mounted) | 🟢 | Primary obstacle ring, 2 cm–4 m, ~15° cone. On the **sense** board | ⚠️ **ECHO is a 5 V output** — level-shift or divide it, never straight into a GPIO. Defeated by glass and by soft/angled surfaces |
| 27 | **4× IR Flying-Fish** (FL/FR/BL/BR, **facing DOWN**) | 🟢 | **Table-edge / cliff detection.** Wired to the **drive** board | The highest-priority safety input in the system, because you want to drive on tables. At 0.3 m/s you have ~166 ms from edge to wheel-off, and an SBC round trip is 50–200 ms — so these go straight to the drive MCU, which coasts the motors in under 1 ms. Calibrate the pots **on your actual table**: gloss reflects specularly, dark matte absorbs, both fool IR |
| 28 | **2× IR Flying-Fish** (front + back, **horizontal**) | 🟢 | Close-range obstacle backstop, ~2–30 cm. On the **sense** board | Covers what ultrasonics miss: chair legs, sound-absorbing surfaces. ⚠️ **With all four corner sensors pointing down, the sides have no close-range coverage** — and skid steer sweeps sideways in every turn. Either turn slowly near obstacles or add 2 more modules (~€4) |
| 29 | **MPU-6050** accel + gyro | 🟢 | **Gyro yaw for the odometry EKF** — the load-bearing heading source | Mount flat, near the centre of rotation, away from the motors. On a skid-steer platform this beats wheel-derived heading by a wide margin |
| 30 | **PIR HC-SR501** | 🟡 | "Something moved, go look" trigger — **only while parked** | Useless in motion: ego-motion triggers it constantly. Nice low-power wake source for the cat game |
| 31 | **NA27 load cell 2 kg + HX711** | 🟡 | Future: cargo tray ("did I pick it up?"), or a force bumper | No role in the core build. Fun once there's a payload |

---

## 8. Power

| # | Component | Status | Role | Notes |
|---|---|---|---|---|
| 32 | **7× INR18650-35E** 3400 mAh | 🟢 | 2× 3S1P packs (6 cells) + 1 spare. ~77 Wh total → **~2.5 h usable runtime** | Good cells, ~8 A continuous — comfortably more than this robot draws |
| 33 | **2× 3S 18650 enclosure** | 🟢 | Pack housing | |
| 34 | **2× BMS 3S 20 A** | 🟢 | Per-pack protection **and balancing ✓** | Balancing confirmed — that removes the manual top-balancing chore entirely |
| 35 | **2× HW-674 buck, 8 A** (XL4016) | 🟢 | **#1: Rock 5B @ 5.1 V into GPIO pins 2 & 4 (proven working).** #2: motor rail @ 6.5–8 V | The GPIO path works but bypasses the board's input protection — **add a 5.6–6.0 V TVS diode and a 1000 µF cap at the header.** If the XL4016's high-side switch fails short, full pack voltage lands on a €150 board. €0.30 of insurance |
| 36 | **4× LM2596 HW-411 buck, 3 A** | 🟢 | 5 V logic rail; **separate** 5 V servo rail; spares | Keep servos on their own regulator — servo inrush browns out logic rails and produces bugs you'll chase for days |
| 37 | **4× MT3608 boost, 2 A** | 🟡 | Spare. Not needed: every rail steps *down* from 3S | Useful if you ever want 12 V for something from a lower source |
| 38 | **4× TP4056 charger** | 🟡 | Single-cell side projects; emergency cell recovery | Your BMSes balance, so routine top-balancing isn't needed. Wrong tool for a 3S pack — but see §11, the bench supply covers that |
| 39 | **3× IP2721 USB-C PD module** | 🟡 | **Charging dock:** wall PD charger → IP2721 → 20 V → buck → 12.6 V CC/CV into the pack | These are PD **sinks**, not sources. No longer needed for main power now that the GPIO 5 V path is proven — the dock is their real job |

---

## 9. Interface, logic, and passives

| # | Component | Status | Role | Notes |
|---|---|---|---|---|
| 40 | **4× level converter** | 🟢 | HC-SR04 ECHO (5 V) → ESP32 (3.3 V) | Run the IR modules, encoders and MPU6050 at **3.3 V** instead and none of them need shifting — LM393 works down to 2 V and its output then swings 0–3.3 V. Only the HC-SR04s genuinely require 5 V |
| 41 | **2× OLED 0.96" SSD1306** | 🟢 / 🟡 | Robot "face" and status display on the sense hub; second one for the head unit | I²C, shares the bus with the MPU6050. Cheap personality — worth doing |
| 42 | **2× LCD 2×16 (HD44780)** | 🟡 | Spare / bench debugging | The OLEDs are better in every way for the robot. These are fine for a charging dock readout |
| 43 | **4× buzzer** | 🟢 | Audible state changes, cat-game taunt, low-battery alarm | Keep the volume low — piezo resonance is unpleasant at feline hearing range |
| 44 | **Servo Tower Pro SG90** | 🟡 | Camera **pan** (Phase 5) | Own 5 V rail. Sweeping the head is what makes `look_around()` and place recognition work |
| 45 | **Servo Redox S90** | 🟡 | Camera **tilt** (Phase 5) | |
| 46 | **NE555 pulse generator** | 🟡 | No planned role | The ESP32s generate every waveform this robot needs in software |
| 47 | **10× NPN S9013 / 10× PNP A92** | 🟢 | Buzzer drive, LED drive, motor-rail kill MOSFET gate drive, level shifting | General-purpose glue |
| 48 | **Resistors, capacitors, diodes** | 🟢 | Dividers (5 V ECHO, battery sense), pull-ups, bulk decoupling on the motor rail, flyback protection | Put generous bulk capacitance near the motor drivers — it's the cheapest fix for brownout-induced resets |
| 49 | **Breadboards + jumper wires** | 🟢 | Prototyping | Move to soldered connections before the robot drives around. Vibration + breadboard = intermittent faults that look like software bugs |

---

## 11. Bench equipment

| # | Component | Status | Role | Notes |
|---|---|---|---|---|
| 50 | **KORAD KA3005D** bench supply, 0–30 V / 0–5 A linear | 🔵 | **Pack charging, bench powering, and current measurement** | Does three jobs a €10 charger cannot — see below |
| 51 | **12 V fixed power supply** | 🔵 | Gentle routine charging | 12 V on a 3S pack is 4.0 V/cell ≈ 85% state of charge. That is not a limitation, it is the **longevity setting**: stopping at 4.0 V/cell roughly doubles Li-ion cycle life versus a full 4.2 V charge. Use this for day-to-day charging and the KORAD at 12.6 V only when you want maximum runtime |

### Why the KA3005D replaces a dedicated charger

It **is** a CC/CV source, which is exactly what Li-ion charging is. Set 12.6 V, set the current
limit, connect: it holds constant current until the pack reaches 12.6 V, then tapers. That is the
entire algorithm.

```
   full charge      12.60 V,  limit 1.5 A   (~0.45C — easy on the cells)
   storage/daily    12.00 V,  limit 1.5 A   (~85%, much longer pack life)
   done when        current tapers to ~0.3 A  (C/10)
```

It also beats a fixed charger on things that matter here:

- **Adjustable current limit** — charge at 0.5C or gentler, rather than whatever a cheap brick does.
- **Live current readout** — you can *see* the CV taper and know when charging is actually finished.
- **Adjustable voltage** — the storage-charge option above simply is not available on a 12.6 V brick.

⚠️ **One real caveat: a bench supply has no termination logic.** A dedicated charger cuts off; the
KORAD will sit at 12.6 V indefinitely, trickling. Holding Li-ion at full charge for days accelerates
ageing. Disconnect when the current tapers — don't leave it connected overnight. Also disconnect
before switching the supply off, since some units can sink a little current when unpowered.

Balancing is already handled: your 3S BMS boards balance (confirmed), so the pack self-corrects.

### The other two jobs

**Bench-powering the robot.** Set 5.1 V, current limit 5 A, feed the Rock 5B's GPIO header directly
and develop with no battery in the loop at all. Removes a whole class of "is this a software bug or
a brownout?" confusion during Phases 0–3.

**Measuring the power budget empirically.** The figures in [PROJECT_PLAN.md](PROJECT_PLAN.md) §4.3
are estimates. With this supply you can read actual draw at idle, under NPU load, and during motor
stalls — and replace the estimates with measurements. Worth doing before sizing anything else around
them.

---

## 12. Not owned — see [SHOPPING_LIST.md](SHOPPING_LIST.md)

Highest-impact gaps, in order:

1. **TVS diode + bulk cap** — protects the Rock 5B on the unprotected GPIO power path, ~€1
2. **2× TB6612FNG** — replaces the drivetrain's weakest link, ~€6
3. Fuses, kill switch, XT30/XT60 connectors — safety basics
4. **2D lidar** — back to essential. The €0 stereo path is dead (one CSI connector + mismatched
   lenses), so this is now the main thing standing between wandering and navigating
5. USB mic array + speaker — deferred by design; the phone app covers it until then

**No longer needed** — a 12.6 V charger was previously listed here. The KORAD bench supply (§11)
does the job better, so it has been removed rather than downgraded.

---

*Inventory as of 2026-08-04, rev 3. Corrections so far: ESP32 classic throughout (no S3); one CSI
connector, not two; host is Radxa Debian 12; charging is covered by existing bench equipment. See
[PROJECT_PLAN.md](PROJECT_PLAN.md) for architecture and [WIRING.md](WIRING.md) for pin assignments.*
