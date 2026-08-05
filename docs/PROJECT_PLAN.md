# Kobold — Project Plan

A 4WD indoor robot that navigates a room (and a table), finds objects presented to it as photos, plays
chase-and-flee with a cat, and is controlled through a phone app. Everything on the robot runs in
Docker and is updateable over the network; the microcontrollers are flashed over USB from the SBC.

- **Brain:** Radxa Rock 5B (RK3588S, 8 GB, 6 TOPS NPU)
- **Reflexes:** 2× ESP32 DevKit V1 over USB
- **Stack:** Radxa Debian 12 (BSP kernel 6.1) → Docker → ROS 2 Jazzy + RKNN/RKLLM + Hermes Agent
- **Validated local model:** Qwen3.5-2B W8A8 plus its paired vision encoder on the NPU via RKLLM
  1.3.0; the native C++ service exposes text chat, streaming, and tool calls (§5.2)

*Rev 4 — Qwen3.5-2B/RKLLM 1.3.0 adopted and validated; ESP32 classic throughout (no S3); GPIO
power path proven; stereo ruled out (one CSI connector); host is Radxa Debian 12, not Ubuntu.*

### Hardware validation snapshot — 2026-08-04

The current Rock 5B was tested directly rather than relying on model/runtime claims:

- RKNN C API 2.3.0 reached RKNPU driver 0.9.8 and executed YOLOv5s on real silicon. The official
  decoder produced correct detections on its reference image; live IMX219 capture sustained
  19.4 FPS with no new camera errors during concurrent NPU inference.
- Equivalent 500-run tests showed Python RKNNLite at 49.0 FPS and about 355 MiB RSS, C++ with float
  outputs at 50.8 FPS and 44 MiB RSS, and C++ with raw INT8 outputs at 60.4 FPS and 38 MiB RSS.
  Production perception therefore uses C++ and keeps Python for conversion and diagnostics.
- Qwen3.5-2B W8A8 and its paired RKNN vision encoder loaded with RKLLM 1.3.0 on driver 0.9.8.
  Text generated at 10.44 tokens/s and used 2.22 GiB peak container memory. The VLM correctly
  described its reference image at 11.71 tokens/s after a 2.10 s visual prefill and peaked at
  3.17 GB. Both paths exercised real NPU silicon.
- A native C++ HTTP server now provides OpenAI-compatible non-streaming and SSE chat completions,
  stateless multi-turn context, and normalized tool calls. It rejects concurrent inference with
  `503` instead of queueing unbounded work. The HTTP text path exercised all three NPU cores.
- The RKLLM artifact's core count is compiled into the model. During concurrent generation, the
  detector remained at 47.8 FPS, while Qwen3.5 generation ran at 7.96 tokens/s. Keep perception
  live by default; pause detector inference, but not capture, when a
  fastest-possible LLM response matters.
- English voice closed the loop with Silero VAD, Whisper base.en q5_1, and Piper Amy. On A76 cores
  5–7, Whisper transcribed 3.35 s of synthesized speech in 3.69 s; the A55-only placement took
  11.9 s and was rejected.

These are stack-validation artifacts, not all final application nodes: YOLO11n still needs a
versioned off-board RKNN conversion, and Hermes/app integration remains Phase 6.

---

## 0. Read this first — the findings that shape the design

**1. Qwen3.5-2B is now the adopted local model.** A community-published, version-matched RK3588
pair now exists: a W8A8 `.rkllm` language decoder plus `.rknn` vision encoder. Both files were
checksum-pinned and exercised with the official RKLLM 1.3.0 runtime on this Rock 5B. The production
HTTP endpoint currently serves text/tool requests; the separately validated vision encoder still
needs to be connected to that API. §5.1–5.2.

**2. Model conversion needs x86_64, which is not available locally.**
`rknn-toolkit2` and `rkllm-toolkit` ship as `linux_x86_64` wheels only. Plan: pull pre-converted
files from HuggingFace, with Rosetta-backed x86 Docker or a €0.50/hr cloud VM as the fallback. The
version-matching trap in §5.2 is the one that causes real failures.

**3. Power via the GPIO header works — verified.** 5.1 V from the HW-674 into header pins 2
and 4. That's now the primary path and the PD source module is off the shopping list. But it bypasses
the board's input protection, so add a TVS diode, a bulk cap, and a proper load test (WIRING §1.1).

**4. The motor shield works at 8 V — which is itself the diagnosis.** The supply is raised to
compensate for the L293D's ~2 V drop. It works, at the cost of heat
and battery. §3.1 has the numbers and what to do about it.

**5. Table driving makes cliff detection the highest-priority input in the system.** All four corner
IRs face down. At 0.3 m/s there is ~166 ms from edge detection to a wheel leaving the table, and a
round trip through the SBC is 50–200 ms. So cliff sensors wire directly to the drive MCU and stop
the motors in under a millisecond. This drove a real change to the board split (§1.3).

**6. Stereo vision is off the table — buy the lidar.** I was wrong about this in rev 2. The ROCK 5B
has **one** 4-lane MIPI CSI connector (plus one DSI), not two, and the two IMX219 modules have
visibly different lenses, so their intrinsics won't match. Both blockers are independent and each
one alone kills the €0 stereo plan. Navigation is now **mono depth on the NPU + a €70 lidar**. §6.

**7. Polish changes the RAM budget substantially.** Polish Whisper fine-tunes are only
distributed as **f16** — quantising them wrecks Polish transcription — so ASR costs 0.5–1.5 GB
instead of the ~60 MB an English `base` would. That collides with the LLM in 8 GB and may push voice
onto the Zero 3W after all. §7.

---

## 1. System architecture

### 1.1 The four-tier control stack

The single most important rule: **the LLM is never in the control loop.** It picks goals; it does
not steer. Each tier runs an order of magnitude slower than the one below and can fail without
taking the tier below it down.

| Tier | Runs on | Rate | Responsibility | Fails safe by |
|---|---|---|---|---|
| **−1 — Hardware** | direct wire | <1 ms | Cliff → motor coast. Sense-board safety line → motor coast | Physics |
| **0 — Reflex** | ESP32 drive | 100–200 Hz | Motor PWM, per-side PID, encoders, IMU, battery, cmd_vel watchdog | Stopping motors |
| **0b — Sensing** | ESP32 sense | 20–50 Hz | Ultrasonic ranging, horizontal IR, buzzer, OLED | Asserting the safety line |
| **1 — Motion** | Rock 5B | 10–30 Hz | Nav2 controller, costmap, local planner, odometry EKF | Zeroing cmd_vel |
| **2 — Perception** | Rock 5B NPU | 5–30 Hz | YOLO, embedding match, VSLAM or place recognition | Reporting no detections |
| **3 — Cognition** | Rock 5B NPU | 0.1–1 Hz | Qwen3.5 + Hermes: goal selection, conversation, tool calls | Falling back to idle |

Tier −1 is new in this revision and is what makes table operation possible. The
emergency stop path contains **no software above the two microcontrollers** — the Rock 5B can be
rebooting, Docker can be pulling images, the LLM can be confidently wrong, and the robot still stops
at the table edge.

### 1.2 Physical topology

```
                    ┌──────────────────────────────────────────┐
                    │  Radxa Rock 5B  (RK3588S, 8 GB, 6 TOPS)  │
                    │  Debian 12 (BSP 6.1) · Docker · ROS 2    │
                    └──┬────┬────────┬─────────┬───────────────┘
        M.2 M-key ─────┤    │        │         └── 2.5 GbE (bench/debug)
        PM991 256 GB   │    │        │
        M.2 E-key ─────┤    │        └── MIPI CSI ×1 ── IMX219 on pan/tilt (§6)
        Radxa A8 WiFi  │    │
   GPIO pins 2/4 ──────┤    │
   5.1 V from HW-674   │    └── USB ── LD19 lidar
                       │
            USB ───────┼── /dev/robot-drive   ESP32 (USB-C) → motors, encoders, MPU6050,
                       │                                       4× cliff IR, battery
                       └── /dev/robot-sense   ESP32          → 4× HC-SR04, 2× IR, buzzer, OLED
                                    │                                 │
                                    └───── SAFETY LINE (1 wire) ──────┘

            ESP32 #3 = spare / head unit (2× servo, PIR, second OLED)
```

All three MCUs are ESP32 DevKit V1 (ESP-WROOM-32). All enumerate as `/dev/ttyUSB*`, all are
flashable with `esptool` over the same cable that carries their telemetry.

### 1.3 Why two boards, and which sensors go where

The split rule is now sharper than "pin budget": **anything that must trigger an emergency stop in
under 100 ms lives on the drive board.**

- **Cliff sensors → drive board.** Table driving has a hard deadline the SBC round trip can't meet.
- **Ultrasonic + horizontal IR → sense board**, which asserts a single hardware safety line to the
  drive board for imminent collisions. One wire, sub-millisecond, no software in the path.
- **Everything advisory → over USB to the SBC** at whatever rate it manages.

Pin budget confirms the split works: drive board needs 15 of its 16 safe GPIO plus all 4 input-only
pins; sense board needs 6 of 16 plus its 4 input-only. Both fit, with the sense board holding room
for the servos and PIR later.

The second reason still stands: **HC-SR04 echo timing is measured in microseconds** and would
otherwise share a core with motor PWM and encoder interrupts. Splitting means an interrupt storm
during a hard turn cannot corrupt the distance reading required *during* a hard turn.

---

## 2. Compute assignment

| Board | Role | Reasoning |
|---|---|---|
| **Rock 5B 8 GB** | Robot brain — ROS 2, perception, LLM, voice, web app | Only board with a usable NPU (6 TOPS) and the RAM for a 2–4B model plus ROS 2. Only one RKLLM supports |
| **ESP32 #1 (USB-C)** | Drive controller + cliff safety | The distinct USB-serial chip gives it a unique VID/PID, making udev rules trivial. USB-C is also the more robust connector for the most important job |
| **ESP32 #2** | Sensor hub | Isolates µs-precision ranging from motor interrupts |
| **ESP32 #3** | Head unit (Phase 5) / spare | Pan-tilt, PIR, OLED face. Leave it on the shelf until needed — an identical spare is worth more than a third USB cable |
| **Odroid N2 4 GB** | Home base station (off-robot) | Docker registry, MQTT, rosbag archive, Foxglove. Mains-powered, 24/7 |
| **Radxa Zero 3W 2 GB** | **Smart room node** (§7.4) | RK3566 has a ~1 TOPS NPU and a CSI port, so it can run YOLO on its own camera. Not enough for the main stack, and **RKLLM doesn't support RK3566** — but as a fixed second viewpoint with a mic, it's genuinely useful |
| **Pi Zero 2W 512 MB** | Second room node, or spare | Same idea, less capable. MJPEG streaming only |
| **Pi 3 1 GB** | Dev / test box | Somewhere to try risky things |
| **Genuino 101** | **Shelved** | Intel Curie, EOL 2017. No modern toolchain |
| **Intel NCS2** | **Shelved** | OpenVINO dropped Myriad X after 2022.3, ARM64 host support was never good. RK3588's NPU is faster and already in the robot |

**Storage.** PM991 256 GB (confirmed 2280, no adapter needed) in the Rock 5B — chosen for power,
not capacity: ~2.5 W under load versus 6–7 W for the PM9A1, which is real battery minutes. OS stays
on the eMMC (recoverable with a card reader after a bad flash); `/var/lib/docker` and `/data` go on
the NVMe. PM9A1 2 TB goes in a USB 3 enclosure on the Odroid as the rosbag archive.

**WiFi.** Radxa A8 in the E-key slot, antennas confirmed present. AX210 is the upgrade path if the
RTL8852BE proves flaky under load. **Configure a fallback AP** — a systemd unit that brings up an
access point if the robot can't associate within 60 s. Without it, one router reboot means carrying
the robot to a desk.

---

## 3. Drivetrain and sensing

### 3.1 Motors — why 8 V works, and why it's the wrong fix

Your 8 V hack works because the L293D drops ~1.5–2 V regardless of supply voltage. At 6 V the motors
saw ~4 V; at 8 V they see ~6 V. You bought back the missing torque with brute force.

The cost is heat. The L293D's dissipation is roughly `I × 2 V` per active channel, and raising the
voltage raises the current too, so **more** power is burned in the chip than before. L293D
thermal shutdown fades in gradually — the robot gets sluggish before it stops, which is a confusing
failure to debug.

| | L293D @ 8 V | TB6612FNG @ 6.5 V |
|---|---|---|
| Voltage at motor | ~6.0 V | ~6.0 V |
| Wasted in driver | ~2 W | ~0.5 W |
| Continuous current | 0.6 A/ch | 1.2 A/ch |
| Logic level | 5 V (needs shifters) | 3.3 V direct |

Same wheel voltage, a quarter of the waste, double the current headroom, and no level converters.
€6 for two boards.

**Two things to check right now on the current setup:**

1. **Heatsink the L293Ds** and measure chip temperature after 10 minutes of driving. Above ~70 °C
   it's already throttling.
2. **Check the motor voltage rating.** The chassis motors are commonly 3–6 V. If they are 6 V,
   running them at 8 V supply is fine intermittently but will shorten brush life. With a TB6612 at
   6.5 V gives the same wheel speed without over-driving them.

Wire a **7.5 A fuse and a physical kill switch on the motor rail**, upstream of everything.

**Per-side wiring:** on a skid-steer chassis, front and rear on a side are coupled through the
floor, so tie their PWM and direction lines. Four independent H-bridges for current capacity, 7 GPIO
instead of 13, and per-side velocity PID — which is what differential drive actually wants. Per-wheel
PID has the wheels fighting each other through the carpet.

### 3.2 Odometry

Your encoders are LM393 modules (VCC/GND/D0/A0), which confirms: **single channel, no direction
sensing.** Direction comes from PWM sign. Typical 20-slot disc on a 65 mm wheel gives **~10 mm per
tick** — decent linear resolution, and only ~120 interrupts/s total at cruise.

**Power them at 3.3 V** so D0 is directly ESP32-safe (WIRING §6). At 5 V, D0 is a 5 V signal into a
3.3 V pin.

Heading is the problem: **skid-steer wheels scrub sideways through every turn by design**, so
encoder-derived yaw is close to useless. Take linear velocity from the encoders and **yaw rate from
the MPU-6050 gyro**, fused through `robot_localization`'s EKF. Then calibrate an empirical
track-width factor: command a 360° turn ten times, measure the actual rotation, scale until they
agree.

### 3.3 Sensor layout (corrected)

| Sensor | Count | Orientation | Purpose |
|---|---|---|---|
| HC-SR04 ultrasonic | 4 | Front, back, left, right — horizontal | Primary obstacle ring, 2 cm–4 m, ~15° cone |
| IR Flying-Fish | 4 | **FL, FR, BL, BR — facing DOWN** | **Table-edge / cliff detection.** Wired to the drive board |
| IR Flying-Fish | 2 | Front, back — horizontal | Close-range obstacle backstop, ~2–30 cm |
| MPU-6050 | 1 | Flat, centre of rotation | Gyro yaw — load-bearing for odometry |
| PIR HC-SR501 | 1 | Forward | "Something moved" wake trigger — **only while parked** |

**Consequence of the all-down corner layout:** the sides have **no close-range sensing** except the
side ultrasonics, whose 15° cone and 2 cm minimum range leave a real gap. Skid steer sweeps sideways
during every turn. Either keep turns slow near obstacles, or add 2 more IR modules (~€4) facing left
and right. Recommended for driving in cluttered rooms.

**Both sensor types are needed because they fail differently:** ultrasonics are defeated by glass and
soft/angled surfaces; IR is defeated by dark matte and glossy surfaces. Each covers the other's
blind spots. Neither sees a table edge from a horizontal mount, which is why the corner sensors
point down.

---

## 4. Power

### 4.1 Packs

7× INR18650-35E, 2× 3S enclosures, 2× BMS (**confirmed balancing** — good, that removes the manual
top-balancing chore). Two 3S1P packs paralleled after their BMSes into a common ~77 Wh bank, then
split into separate rails at the regulator stage.

Before first parallel connection, charge both individually and confirm they're within ~50 mV, or
a large circulating current flows the moment they are joined.

**Charging is already covered by existing bench equipment** — no charger purchase needed. The KORAD
KA3005D (0–30 V / 0–5 A linear) *is* a CC/CV source, which is the whole of Li-ion charging:

| Mode | Set | Notes |
|---|---|---|
| Full charge | **12.6 V**, limit 1.5 A | ~0.45C. Maximum runtime |
| Daily / storage | **12.0 V**, limit 1.5 A | 4.0 V/cell ≈ 85%. Roughly **doubles cycle life** — use this most of the time |
| Done when | current tapers to ~0.3 A | C/10, observed on the live readout |

Your spare fixed 12 V supply does the daily-charge row on its own, which makes it the better routine
charger of the two — 85% is where Li-ion wants to live.

⚠️ **A bench supply has no termination logic.** It will hold 12.6 V indefinitely, and sitting at full
charge for days ages the cells. Disconnect once the current tapers; don't leave it overnight.

Balancing is handled by the BMS boards (confirmed), so nothing further is needed there.

The same supply is worth using during Phases 0–3 to **power the robot directly at 5.1 V with no
battery in the loop**, and to replace the estimates in §4.3 with measured draw.

### 4.2 Rails

```
  3S bank (9.0–12.6 V, 77 Wh)
    ├── [10 A fuse] ── HW-674 @ 5.1 V ── +TVS +1000 µF ── Rock 5B GPIO pins 2 & 4   ~20 W
    ├── [7.5 A fuse + KILL SWITCH] ── HW-674 @ 6.5–8 V ── motor driver ── 4 motors  ~8 W avg
    ├── [3 A fuse] ── LM2596 @ 5.0 V ── 2× ESP32, lidar, OLEDs                      ~4 W
    └── [2 A fuse] ── LM2596 @ 5.0 V ── servo rail ONLY                             ~2 W
```

The GPIO power path is proven working. Make it safe (WIRING §1.1): **TVS diode** across the rail at
the header (a shorted buck puts 12.6 V on a €150 board), 1000 µF bulk cap, both 5 V pins plus four
grounds with short thick wire, and measure voltage **at the header** under load — not at the buck
output, where a 300 mV drop across thin Dupont wire hides.

### 4.3 Budget

| Load | Typical | Peak |
|---|---|---|
| Rock 5B + NVMe + WiFi (NPU active) | 12 W | 20 W |
| 4× motors, indoor carpet | 6 W | 25 W (stall) |
| ESP32 ×2, cameras, sensors, OLEDs | 3 W | 4 W |
| Lidar (if fitted) | 2.5 W | 2.5 W |
| Servos (intermittent) | 0.5 W | 5 W |
| **Total** | **~22 W** | ~55 W |

77 Wh ÷ 22 W ≈ **3.5 h theoretical, ~2.8 h usable** with reserve. Runtime is not a constraint on
this project.

Add an **INA226** on the pack output (€3, I²C) — battery percentage becomes a topic the agent can
read, so it says "15%, heading back" instead of dying mid-room. The ESP32's divider on GPIO 32 is
the redundant cutoff that works even if I²C wedges.

---

## 5. Models — what runs where

### 5.1 Is Qwen3.5-2B/4B the right pick?

**Yes, and for reasons that mostly aren't the benchmarks.** In priority order:

1. **RKLLM v1.3.0 explicitly supports Qwen3.5.** On a vendor NPU toolkit, *supported* beats *better
   on paper* every time. A model 5 points higher on MMLU that won't convert is worth zero.
2. **Native multimodality in the same weights.** One model doing chat, planning, and vision instead
   of two sets of weights in 8 GB of shared RAM. This is the single biggest practical argument.
3. **Apache-2.0** — no licence friction.
4. **Qwen has the strongest tool-calling track record among small open models**, which is precisely
   what Hermes needs from it.

**The honest caveat:** benchmark leadership at 2–4B does not transfer cleanly to this workload. What
actually matters here is instruction-following under a long tool schema, *not* inventing tool
arguments, and visual grounding on low-resolution robot camera frames. No headline benchmark
measures any of those.

Qwen3.5-2B is now the default, but a **30-prompt eval built from this robot** is still required — real
photos from its camera, its tool list, its room, **and prompts in Polish** —
and score 2B vs 4B vs Gemma 4 on that. It's an afternoon of work and it's the only benchmark that
describes this robot. RKLLM supports Gemma 4, SmolVLM, MiniCPM-V and InternVL3 too, so switching is
cheap if the eval produces a surprise.

**Include Gemma 4 in that eval specifically for Polish** — Qwen is Chinese/English-centric by
training priority and Gemma is generally stronger multilingually. See §7.6; at 2B, language ability
is exactly what gets sacrificed.

**Availability is resolved for the selected 2B build — see §5.2.** The exact decoder, vision
encoder, repository revision, checksums, and RKLLM 1.3.0 runtime are pinned in
`models/manifest.yaml`. This validates one specific artifact chain, not arbitrary Qwen3.5
conversions or 4B builds.

**Start at 2B, not 4B.** 4B at w8a8 is ~4.5 GB of weights plus KV cache against 8 GB shared with
ROS 2, Docker, and the camera pipeline. 2B is ~2.2 GB and leaves headroom. Cap context at
**4096–8192** either way — the 262 K window is irrelevant here and the KV cache would consume the RAM budget.
Configure zram plus NVMe swap; the OOM killer should never be what ends a mission.

If Qwen3.5 keeps Qwen3's hybrid thinking toggle, **disable thinking for routine tool calls** and
enable it only for planning. At 5–10 tok/s, a reasoning trace before "drive forward" is unbearable.

**Escape hatch:** the agent's `base_url` is a variable. On WiFi, point it at a 9B on the Odroid or a
frontier model in the cloud for hard problems; offline it falls back to the local 2B. Same code path.

### 5.2 No x86? Use pre-converted models — carefully

This is a genuine constraint and it is workable. Three routes, in recommended order:

**Route A — pre-converted from HuggingFace (primary).**

There's an active community publishing `.rkllm` files for RK3588. Known-good sources:

| Source | What's there |
|---|---|
| [`Qengineering/Qwen3.5-2B-rk3588`](https://huggingface.co/Qengineering/Qwen3.5-2B-rk3588) | **Adopted:** Qwen3.5-2B W8A8 decoder plus paired vision encoder for RKLLM 1.3.0; exact revision and SHA-256 values are pinned in the manifest |
| [`Qengineering/Qwen3-VL-2B-NPU`](https://github.com/Qengineering/Qwen3-VL-2B-NPU) and [`-4B-NPU`](https://github.com/Qengineering/Qwen3-VL-4B-NPU) | **Qwen3-VL multimodal, running on the RK3588 NPU.** Working deployments, both sizes |
| [`kamyarkazemi1373/Qwen3-4B-W8A8-RK3588`](https://huggingface.co/kamyarkazemi1373/Qwen3-4B-W8A8-RK3588) | Qwen3-4B, text only |
| [`Pelochus/ezrkllm-collection`](https://huggingface.co/Pelochus/ezrkllm-collection) | Broad collection, older runtime versions |
| `jamescallander/*_w8a8_g128_rk3588.rkllm` | Actively maintained, many models |
| Rockchip's own `rknn_model_zoo` | Pre-converted `.rknn` for YOLO and vision backbones |

**Adopted recommendation: Qwen3.5-2B on RKLLM 1.3.0.**

The selected Qengineering release supplies both required artifacts and has now been tested on the
actual board. The native C++ server pins the official runtime library/header to Rockchip commit
`878f9361fd3afa7e167b7079918918f78d2c1c2a`; the manifest pins the model repository revision and
both file checksums. Do not substitute a same-named file without updating and re-running the full
version-chain test.

The previous Qwen3-1.7B/RKLLM 1.2.1 model and image remain the rollback chain. Roll back decoder
and runtime together—never point the 1.3.0 image at the 1.2.1 artifact or vice versa. The agent's
OpenAI `base_url` remains unchanged across either chain.

⚠️ **The version-matching trap.** A `.rkllm` file is bound to the runtime version that produced it —
which is why community uploads put the version in the filename (`...-rk3588-rkllm-1.1.4`,
`SuperNova-Medius-rk3588-1.1.4`). **Three things must agree:**

```
   .rkllm file version   ⟷   librkllmrt.so in the container   ⟷   rknpu kernel driver on the host
```

A mismatch produces baffling failures rather than a clear error. Many published
models target older runtimes (1.1.4, 1.2.1), so pinning an **older** `librkllmrt.so` may be necessary
than the current 1.3.0 to use them. Pin all three in the model manifest and print all three at
container startup, so a mismatch is the first line in the log instead of a two-hour debugging
session.

**Route B — x86 emulation on Apple Silicon.** Both Macs are ARM, so there are two sub-routes with
opposite failure modes. Try them in this order.

**B1 — Docker Desktop with Rosetta (fast, may hit instruction gaps).** Enable *Use Rosetta for
x86_64/amd64 emulation* in Docker Desktop's settings, then:

```bash
docker run --rm --platform linux/amd64 -v "$PWD:/w" -w /w \
  python:3.10-slim bash -c "pip install rkllm_toolkit-*.whl && python convert.py"
```

Rosetta *translates* rather than emulates, so this is far faster than a full x86 VM. The catch:
Rosetta has historically not covered AVX instructions, and x86 PyTorch builds use them. An
An **illegal-instruction / SIGILL crash** indicates that limit, not a broken install. Fall
through to B2.

**B2 — Parallels x86 VM (slow, but complete).** Parallels Desktop 20.2+ added x86_64 emulation on
Apple Silicon, so an x86_64 Linux guest is genuinely possible. Two things to know before spending an
evening on it: it needs **Pro/Business/Enterprise** (not the standard edition), and Parallels
themselves ship it as a technology preview with heavy performance caveats. But it's full emulation,
so AVX works where Rosetta doesn't.

**The tradeoff in one line:** Rosetta is fast but may not run the code; Parallels runs the code but
slowly. Conversion is a one-off batch job, so slow-but-works is entirely acceptable.

**Route C — a cloud VM (the reliable one).**
Conversion is a one-off per model. A spot x86 VM at ~€0.50/hr, or an Oracle/GCP free-tier box, does
it in an hour. **Set this up even if Route A suffices today** — fine-tuning a SigLIP variant on
custom objects, or adopting a model before the community converts it, both require it. Depending on
third-party uploads for the entire model supply is a fragile position.

### 5.3 Perception model roles

Because Qwen3.5 is multimodal, everything *could* route through the VLM. It should not — far too slow for
anything reactive. Fast path and slow path:

| Job | Model | Rate | Why |
|---|---|---|---|
| Generic objects + **cat** | YOLO11n → RKNN | 20–30 Hz | COCO includes `cat` (class 15). The cat game's real-time tracker |
| "Is this the mug from my photo?" | SigLIP/CLIP encoder → RKNN | ~50 Hz on crops | Cosine similarity against the reference photo's embedding. Handles *specific instances*, which YOLO's 80 fixed classes cannot |
| Visual place recognition | Same embeddings, whole frames | 1 Hz | Powers topological navigation and "where am I?" |
| Depth (if going mono, §6) | Depth Anything V2 small → RKNN | 5–10 Hz | Relative depth → costmap, scaled by wheel odometry |
| Novel objects, scene understanding, chat, planning | **Qwen3.5-2B/4B** → RKLLM | 0.1–1 Hz | The slow, smart path — on candidates and on user requests |

**NPU contention:** RK3588 has three NPU cores. Pin vision to core 0 via RKNN's core mask, but do
not assume RKLLM can be assigned the other two: core allocation is compiled into the `.rkllm`
artifact. The adopted Qwen3.5-2B artifact reports three cores. With both running, detector
throughput measured 47.8 FPS while generation ran at 7.96 tokens/s, versus 10.44 tokens/s for
standalone text generation. Leave perception live for safety/awareness; suspend only detector inference when
minimum LLM latency is more important than continuous detections.

**CPU pinning matters too.** RK3588 is cores 0–3 (A55) + 4–7 (A76). Hardware measurements with
Whisper base.en q5_1 rejected the original A55 plan: 3.35 s of audio took 11.9 s on the A55s,
2.94 s on all A76s, and 3.69 s on A76 cores 5–7. Reserve core 4 for perception and give voice
cores 5–7; concurrent ASR only reduced measured detector throughput from 60.4 to 57.3 FPS.

### 5.4 "Find the thing in this picture"

1. Upload a photo in the app, optionally with a label ("my blue mug").
2. Encode it with SigLIP → one embedding. Also send it to Qwen3.5 once for a text description and
   likely COCO classes. Both go into a SQLite `targets` table with the image.
3. `find_object("my blue mug")` starts a search: navigate to unexplored areas, sweeping the pan
   servo at each waypoint.
4. Each frame, YOLO proposes regions; each crop is embedded and compared to the target.
5. Over threshold → **the VLM verifies**: crop plus reference image to Qwen3.5, "are these the same
   object?" This second opinion kills false positives, and it's affordable because it only fires on
   candidates.
6. Confirmed → approach, stop at ~50 cm, announce in the app with a photo, record the pose so
   "where's my mug?" is answerable later from memory.

Embeddings are fast but shallow; the VLM is capable but slow. Chaining them pays for the
smart one only when something interesting shows up.

---

## 6. Navigation — stereo is dead, here's what replaces it

### Why the €0 stereo plan doesn't work

Two independent blockers, either of which is fatal on its own:

**Blocker 1 — one CSI connector.** The ROCK 5B has **one four-lane
MIPI CSI** connector plus one DSI. (The 5B**+** is the variant with two CSI connectors; that's the
source of the confusion, and my rev 2 claim was wrong.)

Radxa's docs do note the 4-lane connector "can be split into 2× two-lane" — but they document no
overlay for it, a splitter adapter board would be required (community designs exist, none official), and
it would mean writing a custom device tree overlay for a dual-sensor configuration nobody publishes. That
is a research project with an uncertain ending, not a build step.

**Blocker 2 — mismatched optics.** Your instinct is correct and it's the deeper problem. Stereo
depth comes from triangulating *the same feature* across two images. That requires closely matched
focal length, field of view, and distortion. A "big lens" and a "tiny lens" mean different
intrinsics, which means:

- Only the overlapping FOV yields depth — a narrow camera paired with a wide one wastes most of the
  wide one's frame.
- Matching features across images at different scales is far less reliable.
- Rectification amplifies the difference in distortion profiles into systematic depth error.

Calibration handles *small* differences between units. It does not rescue genuinely different
lenses. Stereo wants two identical modules, ideally from the same batch.

### What to build instead

**Primary — monocular depth on the NPU (€0).** Run **Depth Anything V2 small** converted to RKNN on
the single MIPI camera. Relative depth per frame, scaled by wheel odometry, projected to a point
cloud, into an `octomap` or Nav2 costmap. Localisation from the EKF plus visual place recognition.

It won't close large loops and metric accuracy is mediocre, but it runs on the NPU rather than the
CPU, needs no calibration rig, and answers "is there a chair one metre ahead" perfectly well. It
also still sees table tops and chair seats — the 3D advantage that made stereo attractive.

**Add the lidar (€70) — this is now clearly worth buying.** `slam_toolbox` plus Nav2 gives real
metric SLAM, persistent maps, and localisation that works in the dark, against blank white walls,
and over plain carpet. Vision degrades badly in exactly those conditions, and a dim room with plain
walls is a normal room. With the €0 path gone, this is the best value purchase in the project.

The combination is genuinely good: **lidar for rock-solid localisation, mono depth for 3D obstacles
the lidar plane misses.** That covers most of what stereo would have provided.

**Build the topological place graph regardless** (named places, image embeddings, VLM descriptions,
rough edges between them). It's how the LLM should reason about locations no matter what's driving
underneath, and it degrades gracefully when metric SLAM loses tracking.

### Which camera goes on the robot

You have one CSI slot, so pick one — **measure, don't guess**:

> Tape a ruler to a wall. Photograph it with each module from exactly 1 m. Compare the visible
> width. Wider field of view wins for the robot; the other becomes the room-node camera.

FOV matters more than resolution here — the priority is seeing the chair leg alongside, not reading a book
across the room. Run whichever wins at 1280×720 @ 30 fps. The IMX415 (4K) also fits this slot but is
heavier, smears more under motion, and costs memory bandwidth the NPU wants.

### Adding stereo properly, later

In rough order of sanity: buy **two matched USB webcams** (~€30, no CSI limits, no overlay work — the
easy path); or buy an **OAK-D Lite** (~€150, does depth on-device and consumes no host CPU); or
buy two matched IMX219 modules plus a 2×2-lane splitter and write the overlay. Only the last one is
free, and it isn't really.

---

## 7. Voice — models, placement, and the phone bridge

### 7.1 Polish makes this a RAM problem, not a placement problem

Start on the Rock 5B. But **Polish may force voice onto the Zero 3W after all**, and it's worth
understanding why before building it, because it is a budget question rather than a latency one.

The reason: **Polish Whisper fine-tunes are distributed as f16 only.** The maintainer of the
whisper.cpp-ready builds notes that `q5_0` quantisation degraded Polish transcription to garbled
output, so quantised versions are not published. English `base` at q5 costs ~60 MB.
Polish `small` at f16 costs ~500 MB, and `medium` ~1.5 GB.

| Component | RAM |
|---|---|
| Qwen3.5-2B w8a8 + KV cache @ 4096 | ~2.2 GB text / ~3.2 GB with vision encoder |
| Whisper **small-pl f16** | ~0.5 GB |
| Piper TTS | ~0.1 GB |
| YOLO11n + Depth Anything small (RKNN) | ~0.3 GB |
| ROS 2 + Nav2 + costmaps | ~1.5 GB |
| OS + Docker + camera pipeline | ~1.0 GB |
| **Total** | **~6.1 GB of 8 GB** |

That fits. The 4B model (~5.0 GB with KV) brings the total to ~8.4 GB — **it does not fit.** Swap
in `medium-pl` too and it's hopeless.

**Decision rule:**

- **Qwen3.5-2B + whisper small-pl → both on the Rock 5B.** Simplest, lowest latency. Start here.
- **If the 4B model is required** (see §7.6 — Polish is a real argument for it) **or `medium-pl`
  accuracy → move voice to the Zero 3W.**

The key realisation: **whisper.cpp and Piper are pure CPU work.** The RK3566's lack of RKLLM support
is irrelevant to them. A Zero 3W with 2 GB holds whisper small-pl f16 plus Piper comfortably, and it
buys back ~600 MB on the Rock 5B for the bigger LLM.

The cost is one network hop (~20–50 ms on local WiFi), which is noise next to whisper's own
inference time. My earlier "keep ASR next to the LLM" reasoning was about latency and still holds —
it's just that Polish makes RAM the binding constraint instead.

On the Rock 5B, pin voice to A76 cores 5–7 and reserve core 4 for perception. If voice moves to the
Zero 3W, this local CPU split no longer applies.

### 7.2 The pipeline

```
  Phone browser mic ──WebSocket (Opus)──┐
  USB mic array (future) ──ALSA─────────┼──► [VAD] ─► [wake word] ─► [ASR] ─► text ─► agent
  Room node mic (future) ──WebSocket────┘                                              │
                                                                                       ▼
  Phone speaker ◄── WebSocket ◄──┐                                                  [reply]
  Robot speaker (future) ◄─ALSA──┴──────────────── [TTS] ◄──────────────────────────────┘
```

**Abstract audio I/O behind one interface.** Phone versus onboard mic is then a config flag, and
when the mic array arrives nothing above the `voice` container changes.

### 7.3 Which models

Confirmed Polish stack:

| Stage | Model | Size | Notes |
|---|---|---|---|
| VAD | **Silero VAD** | ~2 MB | Gates everything downstream. Cheap and excellent |
| Wake word | **openWakeWord** | ~5 MB | Custom phrase. Only needed for onboard mics — the app has push-to-talk |
| **ASR** | **`knightdave/whisper-polish-ggml-handy`** → `ggml-small-pl.bin` **f16** | ~500 MB | whisper.cpp-ready GGML builds of the **bardsai** Polish fine-tunes, which won the Polish category of HuggingFace's Whisper fine-tuning sprint. **Start with `small-pl`** |
| ASR (better) | same repo → `ggml-medium-pl.bin` f16 | ~1.5 GB | The maintainer recommends medium for quality/speed balance — but §7.1 shows it likely won't fit beside the LLM on one board |
| **TTS** | **Piper**, `pl_PL-gosia-medium` or `pl_PL-darkman-medium` | ~60 MB | Fast, good quality, native Polish voices. TTS is solved |
| DOA | *hardware* | — | Mic array DSP, not software — §7.4 |

⚠️ **Do not quantise the Polish ASR model.** `q5_0` garbles Polish output badly enough that the
maintainer stopped publishing quantised builds. Budget f16 and plan §7.1's RAM table around it. This
is the single biggest surprise Polish introduces.

**Always pass the language flag** (`-l pl`). These are fine-tunes, not multilingual models — they
need to be told, and accuracy drops noticeably without it.

A generic multilingual `whisper-small` also handles Polish, but a dedicated Polish fine-tune at the
same size will beat it comfortably. Use the fine-tune.

### 7.4 The mic array — buy USB, not a HAT

For a 4–6 mic array:

**Get a USB mic array with onboard DSP** — ReSpeaker USB Mic Array v2.0 (4 mics, hardware
beamforming, AEC, and direction-of-arrival over USB HID) or a MiniDSP UMA-8 (7 mics). ~€70–90.

**Avoid the ReSpeaker Pi HATs.** They need the `seeed-voicecard` kernel driver for their WM8960/AC108
codec, which is Raspberry Pi specific. Porting it to RK3588 is a real, unrewarding project. A USB
array is standard USB Audio Class — it just appears as an ALSA device, on any board, forever.

The DOA output is worth more than it sounds: the robot can **turn toward whoever spoke**, which is a
disproportionately large jump in how it feels to interact with.

Speaker: a PAM8403 amp plus a 4 Ω speaker, ~€8, mounted facing forward. A speaker firing into the
chassis sounds terrible.

### 7.5 Room nodes — what the Zero 3W is actually for

This is where the Zero 3W earns its place: a **fixed node in the corner of the room** with its CSI
camera and a cheap USB mic. It provides:

- A **second viewpoint** — the robot can find the cat even when the cat isn't in its own field of
  view. Directly useful for the cat game.
- A **second listening point**, so "hey robot" works from anywhere in the room rather than only
  within a few metres of the robot.
- Local YOLO on its own NPU, so it publishes *detections* over MQTT rather than streaming raw video
  and saturating the WiFi link.

The Pi Zero 2W can be a second, dumber node (stream only, no NPU). This is a Phase 8+ project, but
it's the right home for both boards.

### 7.6 The LLM also has to speak Polish — and that's the harder half

ASR and TTS are solved. The model is the uncertain part, and it's worth planning for.

**Multilingual ability is one of the first things sacrificed at small scale.** A 2B model's Polish
is meaningfully worse than its English — expect clumsy grammar, occasional English leakage, and
degraded instruction-following *in* Polish even when the same instruction works fine in English.
Qwen models are also Chinese/English-centric by training priority; Polish is present but not a
focus. **Gemma is generally stronger multilingually**, and RKLLM supports Gemma 4 — so add a Gemma
variant to the eval in §5.1 specifically to compare Polish output. That comparison may matter more
than any benchmark score.

**Three mitigations, cheapest first:**

1. **Keep the system prompt and tool schema in English; let the model reply in Polish.** Models
   handle this split far better than they handle reasoning in a weaker language. It also keeps tool
   names and arguments stable, which is where a confused small model does the most damage — a
   mistranslated tool argument is a robot driving the wrong way.
2. **Use the 4B model if 2B's Polish disappoints.** This is a genuine argument for 4B over 2B, and
   it's what §7.1's RAM table exists to resolve: 4B plus Polish ASR doesn't fit on one board, so
   picking 4B means moving voice to the Zero 3W.
3. **Route conversation to the cloud, tool calls to local.** The `base_url` indirection already
   supports this. Local 2B for "drive to the kitchen"; a frontier model for actual conversation when
   WiFi is up. Best quality, needs connectivity.

**Test this early — in Phase 6, not Phase 8.** If the local model's Polish is unusable, it changes
model choice, RAM budget, and board layout. That is an expensive thing to discover late.

---

## 8. Software: containers, topics, updates

### 8.1 Host OS — keep Radxa's Debian, drop the desktop

Running **`rock-5b_bookworm_kde`: Debian 12, Radxa BSP kernel 6.1.84**. Keep it. Two changes.

**Why keep it.** ROS 2's Tier 1 platform for Jazzy is Ubuntu 24.04, and there are no official
Jazzy binaries for Debian 12 — which sounds like a problem and isn't, because **ROS 2 runs in a
container here anyway**. The container is Ubuntu 24.04 inside; the host only has to provide a
kernel, Docker, and device nodes. That was already the architecture, and this is the payoff: the
host distro is very nearly irrelevant.

What the host *does* have to provide is the hard part, and Radxa's image already provides it:

| Needs the vendor BSP | Why mainline won't do |
|---|---|
| **NPU** (`rknpu` driver) | The whole project depends on it. Mainline RK3588 support does not include a usable NPU driver |
| **MIPI CSI** camera overlays | Sensor drivers and device tree overlays are vendor-supplied |
| **Hardware H.264 encode** (`rkmpp`) | Needed for the app video stream without burning CPU |
| **Mali GPU** (`libmali`) | OpenCL, for SGBM depth on the GPU |

So the 6.1.84 BSP kernel is not a compromise here — it is **the correct kernel** for this hardware,
and swapping to a mainline-kernel Ubuntu image would cost the NPU. Do not churn the OS.

**Change 1: drop the desktop.** This one is quantified rather than stylistic. KDE Plasma plus
SDDM costs roughly **0.8–1.5 GB of RAM**, and §7.1's budget already lands at ~6.1 GB of 8 GB with
the 2B model and Polish ASR resident. A desktop pushes that to ~7.5 GB, at which point the OOM
killer becomes the usual cause of a failed mission.

```bash
sudo systemctl set-default multi-user.target
sudo systemctl disable sddm
sudo reboot
# verify: free -h should show >7 GB available
```

Reversible any time with `systemctl set-default graphical.target`, so there is no downside to a
headless robot reached over SSH and the web app. Run `rviz2` or Foxglove on a workstation against
the robot's `foxglove_bridge` — that is the better workflow regardless.

**Change 2: verify the NPU driver version before Phase 4.** This is the single most likely source
of baffling failures later, and it costs one command now:

```bash
sudo cat /sys/kernel/debug/rknpu/version    # e.g. "RKNPU driver: v0.9.8"
dmesg | grep -i rknpu
```

Write that number in `models/manifest.yaml`. The `librknnrt.so` shipped inside the perception and
LLM containers must be compatible with **this** driver, and a `.rkllm` file adds a third version to
the chain (§5.2). Three things must agree, and only one of them lives inside a container.

**Other Debian-12-specific notes:**

- Docker: use Docker's own `docker-ce` repo, not Debian's `docker.io` package — the packaged
  version lags behind current `compose` v2.
- Python 3.11 on the host. Irrelevant to the bridge, which runs in a container, but relevant when
  run `tools/gen_protocol.py` or `esptool` directly on the robot.
- `udev`, `dialout` group, and the rules in `scripts/99-kobold.rules` work identically to Ubuntu.
- Radxa images ship `rknn_server` / `restart_rknn.sh` for NPU debugging. Handy in Phase 4.

### 8.2 Containers

Everything `network_mode: host` — ROS 2 DDS discovery across bridged Docker networks is a
long-running source of misery.

| Service | Contents | Notes |
|---|---|---|
| `base` | robot_state_publisher, URDF, EKF, TF | Everything else assumes it's running |
| `serial-bridge` | Both ESP32 protocols → ROS 2 topics | §8.3 |
| `nav` | Nav2 + RTAB-Map (or slam_toolbox) | §6 |
| `camera` | CSI capture, rkmpp H.264 encode | Needs `/dev/video*` |
| `perception` | RKNN: YOLO, SigLIP, depth, place recognition | Needs `/dev/dri` + rknpu |
| `llm` | Native C++ RKLLM 1.3 server, OpenAI-compatible text/tools/SSE on :8080 | Needs `/dev/dri` + rknpu; paired VLM encoder is validated but not yet HTTP-wired |
| `agent` | Hermes Agent + robot MCP server | Points at `llm` via `base_url` |
| `voice` | Silero VAD, openWakeWord, whisper.cpp, Piper | **Pin to A76 cores 5–7** |
| `app` | FastAPI + PWA: chat, photo upload, video, teleop, e-stop, map | |
| `bridge` | foxglove_bridge | Debugging |
| `updater` | Image pulls + esptool firmware flashing | §8.4 |

`/data` on the NVMe holds models, maps, the targets DB, and a ring-buffered rosbag.

⚠️ **Number one gotcha:** `librknnrt.so` / `librkllmrt.so` in the container must match the `rknpu`
kernel driver on the host **and** the version the `.rkllm` file was converted with. Print all three
at startup.

### 8.3 ESP32 ↔ ROS 2

**Plain framed serial plus a bridge node — not micro-ROS.** micro-ROS couples ESP-IDF to a specific
ROS 2 distro, upgrades are brittle, and a failure means debugging XRCE-DDS instead of the
robot. A versioned CRC-framed protocol is a few hundred lines and debuggable with a serial monitor.

```
  From drive:  /wheel_ticks  /imu/data_raw  /battery_state  /cliff/{fl,fr,bl,br}
               /drive/status  /firmware_version
  To drive:    /cmd_vel  /estop  /motor_enable  /mode  (normal | table)
  From sense:  /range/{front,back,left,right}  /ir/{front,back}  /sense/status
  To sense:    /buzzer  /oled/text
```

**udev rules keyed to VID/PID** — note that CH340 clones often share serial numbers, so it may
need to match physical USB port paths and always plug each board into the same port. Label the ports
(WIRING §5). The failure mode — motor commands sent to the ultrasonic board — is as bad as it sounds.

### 8.4 Updates

Your Macs build **arm64 natively** — no QEMU, no slow builds. Use a Mac as the build machine.

```
  Mac (arm64 native)     ──push──►  registry on Odroid N2  ──pull──►  robot
  HuggingFace / x86 VM   ──models──────────────────────────────────►  /data/models
```

- Everything in git; **pin images by digest** in `versions.env` (tags move, digests don't).
- Keep the previous digest set in `versions.prev.env` plus a one-command `rollback.sh`.
- **Never auto-update.** Trigger from the app, and refuse while `/cmd_vel` is non-zero or the robot
  isn't docked. Watchtower pulling a broken image at 3 am while the robot is mid-room is a bad night.
- **Firmware:** the bridge reads `/firmware_version` at startup; mismatch against the manifest →
  the updater flashes with `esptool` over USB. Keep the last known-good binary on disk. Compile
  **ArduinoOTA in as a backup path** — these are WiFi-capable ESP32s and USB flashing will fail at
  an inconvenient moment eventually.

---

## 9. The agent layer

**Hermes Agent** connects to any OpenAI-compatible endpoint via `base_url`, supports **MCP servers**,
and persists learned procedures as skills across sessions — "how to find the mug" can become a saved
skill.

Write a **ROS 2 MCP server** exposing the robot as tools. Hermes connects over stdio and discovers
them at startup.

Note: as of July 2026 Hermes reads manual MCP definitions from top-level `mcp_servers:`, not
`mcp: servers:`.

```yaml
model:
  base_url: http://localhost:8080/v1     # RKLLM; swap for Odroid/cloud when online
  name: qwen3.5-2b
mcp_servers:
  robot:
    command: python
    args: ["-m", "kobold_mcp.server"]
```

```
  # Motion
  navigate_to(place | x, y)      explore(duration_s)      stop()
  look_around(pan_range_deg)     set_mode(normal | table)

  # Perception
  what_do_you_see()              find_object(target | query)
  remember_place(name)           where_am_i()

  # Play
  start_cat_game()               stop_cat_game()

  # State & expression
  battery_status()   say(text)   show_face(emotion)   beep(pattern)
```

Keep tools **coarse-grained**. `navigate_to("kitchen")` is a good tool; `set_left_motor_pwm(180)` is
a terrible one — it puts a 1 Hz model inside a 100 Hz loop. Every tool should take seconds, report a
result, and be safe to call in any state.

Give the agent a **memory file** of places, targets, and observations. "The mug was on the desk at
14:20" is what makes the robot feel like it has continuity.

---

## 10. The cat game

A state machine (ROS 2 node or BehaviorTree.CPP), **not** the LLM. The LLM only calls
`start_cat_game()` and comments afterwards.

```
  SEEK      ── rotate + patrol, YOLO looking for COCO class 15
    │ cat detected
  APPROACH  ── move toward the cat at ≤0.3 m/s, stop at 1.0 m
    │ arrived
  TAUNT     ── short beep, wiggle, back off 20 cm, OLED face
    │ cat closes to <0.6 m OR bbox growing fast
  FLEE      ── reverse and run at ≤0.5 m/s, full obstacle avoidance
    │ cat unseen 5 s, or wall reached
  RESET     ── pause 3 s → SEEK
```

**Non-negotiable safety rules:**

- Hard cap 0.5 m/s. On a table, 0.15 m/s and FLEE is disabled entirely — a fleeing robot on a table
  is a robot on the floor.
- Rear cliff and rear ultrasonic always active before reversing. FLEE reverses constantly.
- **Never corner the cat.** Large bbox plus walls on multiple sides → exit to RESET and back away.
- **Wheel guards** before the first session. Exposed wheels, paws, tails.
- Buzzer quiet — piezo resonance is unpleasant at feline hearing range.
- Supervise the first several sessions. Some cats engage and some are frightened, and the difference
  know within a minute.

---

## 11. Build phases

| Phase | Goal | Done when | Est. |
|---|---|---|---|
| **0** | Rock 5B: **drop the desktop** (§8.1), NVMe, Docker, WiFi + fallback AP, TVS/cap on the power rail, 30-min load test | Survives `stress-ng` + NPU + motors stalling, no resets; `free -h` shows >7 GB available | 1 wk |
| **1** | Drive ESP32: motors, encoders, IMU, serial protocol, watchdog | Teleop works; motors stop 300 ms after the bridge is killed | 1–2 wk |
| **2** | **Cliff safety + sense ESP32:** cliff reflex, safety line, full sensor ring, power rails, fuses | **Robot drives to a table edge and stops, ten times out of ten** | 1–2 wk |
| **3** | Odometry + EKF: gyro/encoder fusion, track-width calibration | Commanded 360° turn lands within ~10° | 1 wk |
| **4** | Camera + NPU: FOV test, RKNN in Docker, YOLO at 20+ Hz, video in the app | App shows live video with bounding boxes | 1–2 wk |
| **5** | Navigation: lidar + slam_toolbox + Nav2, mono depth into the costmap | "Go to the kitchen corner" works twice in a row | 2–3 wk |
| **6** | LLM + agent + voice: RKLLM server, MCP tools, Hermes, Polish ASR/TTS via phone. **Test Polish quality first** | You ask "co widzisz?" out loud and it answers out loud | 2 wk |
| **7** | Object search: SigLIP, targets DB, two-stage verification | Upload a photo of a mug, robot finds the mug | 1–2 wk |
| **8** | Cat game, room nodes, mic array, update pipeline | The cat has an opinion | 2–3 wk |

Roughly 3–4 months of evenings. **Phase 2 is the gate for table driving** — don't put the robot on a
table until the cliff reflex has been tested to destruction on the floor first (drive at a taped
line and verify it stops). Phases 1–3 determine whether any of this works; a robot with flawless AI
and bad odometry gets stuck under the sofa.

---

## 12. Honest risks

| Risk | Likelihood | Mitigation |
|---|---|---|
| RKLLM version mismatch (file ⟷ runtime ⟷ kernel driver) | **High** | Pin all three, assert at startup (§5.2). Expect to pin an *older* runtime to match published models |
| Community Qwen3.5 artifact disappears or changes | Low, high impact | Exact HF revision and SHA-256 values are pinned; keep local copies and the Qwen3-1.7B/1.2.1 rollback chain |
| 4B doesn't fit alongside ROS 2 in 8 GB | **High** | Start at 2B, cap context at 4096 |
| **Robot drives off a table** | **High** without Phase 2 discipline | Hardware cliff reflex, 0.15 m/s table mode, test on floor first |
| L293D thermal shutdown mid-session | **Medium-high** | Heatsink now; TB6612 properly |
| Buck failure puts 12.6 V on the Rock 5B | Low, catastrophic | TVS diode. €0.30 |
| Mono depth insufficient for reliable navigation | **Medium-high** | This is what the €70 lidar is for. Stereo is not available (§6) |
| CH340 boards share a serial → wrong device gets motor commands | Medium | USB-C board as drive (distinct VID/PID); port-path udev; label ports |
| **Local model's Polish is too weak to be usable** | **Medium-high** | Test in Phase 6, not Phase 8. English system prompt; eval Gemma 4; 4B or cloud fallback (§7.6) |
| Polish ASR f16 + 4B model don't fit in 8 GB | **Confirmed** | 2B on-robot, or move voice to the Zero 3W (§7.1) |
| Rosetta SIGILLs on AVX during model conversion | Medium | Parallels x86 VM, or a cloud VM (§5.2 B2/C) |
| Side blind spot (all corner IR face down) | Medium | Slow turns, or 2 more IR modules (€4) |
| NPU contention slows generation and vision | Medium | Pin RKNN vision to Core0; keep perception live by default, or pause detector inference for latency-critical LLM turns. RKLLM core count is model-compiled |
| Scope creep | **High** 🙂 | The phase table |

---

## 13. Repo layout

```
kobold/
├── PROJECT_PLAN.md · COMPONENTS.md · WIRING.md · SHOPPING_LIST.md
├── firmware/
│   ├── drive/               motors, encoders, IMU, PID, cliff reflex, watchdog
│   ├── sense/               ultrasonic, IR, buzzer, OLED, safety line
│   └── protocol/            shared framed-serial definition (versioned)
├── ros2_ws/src/
│   ├── kobold_bringup/     launch, params, URDF
│   ├── kobold_bridge/      serial ↔ ROS 2
│   ├── kobold_perception/  RKNN: YOLO, SigLIP, depth, place recognition
│   ├── kobold_behaviors/   search, patrol, cat game
│   └── kobold_msgs/
├── agent/{hermes,kobold_mcp}/
├── app/                     FastAPI + PWA
├── voice/                   VAD, wake word, ASR, TTS
├── models/manifest.yaml     versions + checksums + rkllm runtime version
└── docker/{compose.yaml,versions.env,rollback.sh}
```

---

## 14. Open questions

1. ~~Voice language?~~ **Polish.** Drives f16 ASR models and the RAM budget (§7.1, §7.3).
2. ~~Is either Mac Intel?~~ **No — both ARM.** Routes B1/B2/C in §5.2.
3. ~~Which Qwen3.5 size is usable now?~~ **Qwen3.5-2B W8A8 plus its paired vision encoder, pinned
   to RKLLM 1.3.0.** The 4B remains unevaluated and is not the default.
4. **Which camera has the wider FOV?** Measure both against a ruler at 1 m (§6). One CSI slot, so
   the winner goes on the robot and the loser becomes the room-node camera.
5. **Motor voltage rating** on the chassis motors — determines whether 8 V is abuse (§3.1).
6. **Encoder slots per revolution** — count the slots on the disc; it sets the odometry scale.
7. **Is the local 2B's Polish good enough?** Test in Phase 6. Drives model size, RAM, and whether
   voice moves to the Zero 3W (§7.6).

---

*Sources:*
[Qwen3.5 small models](https://artificialanalysis.ai/articles/qwen3-5-small-models) ·
[airockchip/rknn-llm](https://github.com/airockchip/rknn-llm) ·
[RKLLM on Radxa](https://docs.radxa.com/en/rock5/rock5b/app-development/ai/rkllm-usage) ·
[ROCK 5B power](https://docs.radxa.com/en/rock5/rock5b/getting-started/power-supply) ·
[Hermes Agent](https://hermes-agent.nousresearch.com/docs/) ·
[Hermes MCP](https://hermes-agent.nousresearch.com/docs/user-guide/features/mcp/) ·
[rknn-toolkit2](https://pypi.org/project/rknn-toolkit2)
