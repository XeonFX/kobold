# kobold

A 4WD indoor robot that navigates a room (and a table), finds objects you show
it as photos, plays chase-and-flee with a cat, and talks to you through a phone
app. Everything on the robot runs in Docker; the microcontrollers are flashed
over USB from the SBC.

> **Status: early.** The protocol, firmware and ROS 2 bridge are written and
> tested. Perception, navigation, voice and the agent layer are planned but not
> yet implemented — see [the roadmap](#roadmap).

```
        Radxa Rock 5B (RK3588S, 8 GB, 6 TOPS NPU)
        Debian 12 (Radxa BSP, 6.1) · Docker · ROS 2 Jazzy · RKNN/RKLLM · Hermes
                    │
        ┌───────────┴───────────┐
        │ USB                   │ USB
   ESP32 "drive"           ESP32 "sense"
   motors, encoders,       ultrasonics, IR,
   IMU, battery,           buzzer, OLED
   cliff reflex                │
        └────── safety line ───┘
```

## Design in one section

**The LLM is never in the control loop.** It picks goals; it does not steer.
Five tiers, each an order of magnitude slower than the one below, each able to
fail without taking the tier below it down:

| Tier | Where | Rate | Job | Fails safe by |
|---|---|---|---|---|
| −1 Hardware | direct wire | <1 ms | Cliff → motor coast; safety line → motor coast | Physics |
| 0 Reflex | ESP32 drive | 100–200 Hz | PWM, per-side PID, encoders, watchdog | Stopping motors |
| 0b Sensing | ESP32 sense | 20–50 Hz | Ranging, IR, buzzer, display | Asserting the safety line |
| 1 Motion | Rock 5B | 10–30 Hz | Nav2, costmaps, EKF | Zeroing cmd_vel |
| 2 Perception | Rock 5B NPU | 5–30 Hz | YOLO, embeddings, depth | Reporting no detections |
| 3 Cognition | Rock 5B NPU | 0.1–1 Hz | VLM planning, conversation, tools | Falling back to idle |

Tier −1 is what makes it safe to put a language model in charge of a physical
object. The emergency-stop path contains **no software above the two
microcontrollers**: the SBC can be rebooting, Docker can be pulling images, the
model can be confidently wrong, and the robot still stops at the table edge.

## Quick start — no hardware required

The firmware simulator creates fake drive and sense boards on real pseudo-terminals,
so the whole stack runs before anything is soldered.

```bash
git clone https://github.com/XeonFX/kobold.git && cd kobold
make test                 # framing codec tests, both languages
make sim                  # prints two device paths
```

Then in another shell:

```bash
ros2 launch kobold_bringup bringup.launch.py drive_port:=/dev/pts/3 sense_port:=/dev/pts/4
```

Stage failures that are tedious to reproduce physically:

```bash
python3 -m kobold_bridge.sim --cliff-after 5     # trip a cliff fault
python3 -m kobold_bridge.sim --bad-version       # exercise the version refusal
```

## With hardware

```bash
make udev            # stable /dev/robot-drive and /dev/robot-sense (Linux)
make firmware        # build both boards
make flash           # flash over USB
make bringup
```

Driving on a table? Use `table_mode:=true` — and read
[the cliff-safety notes](docs/WIRING.md#21-cliff-sensors-live-here-not-on-the-sense-board)
first. Test the reflex against a taped line on the floor before trusting it at
a real edge.

## Repository layout

```
protocol/protocol.yaml     Single source of truth for the wire protocol
tools/gen_protocol.py      Generates the C++ and Python bindings from it

firmware/                  PlatformIO, three ESP32 DevKit V1 boards
  lib/kobold_protocol/    Framing codec — COBS + CRC16, platform-independent
  src/drive/               Motors, encoders, IMU, battery, cliff reflex
  src/sense/               Ultrasonics, IR, buzzer, OLED, safety line
  src/head/                Pan/tilt servos, PIR  (Phase 5)
  test/native/             Codec tests that run on your laptop

ros2_ws/src/
  kobold_bridge/          Serial ↔ ROS 2, version handshake, simulator
  kobold_bringup/         Launch files, URDF, EKF config
  kobold_perception/      RKNN: YOLO, embeddings, depth   (Phase 4)
  kobold_behaviors/       Search, patrol, cat game        (Phase 7)

agent/                     Hermes Agent + ROS 2 MCP server (Phase 6)
app/                       FastAPI + PWA: chat, photos, video, teleop
voice/                     VAD, wake word, whisper ASR, Piper TTS  (Phase 6)
models/manifest.yaml       Model versions + checksums
docker/                    Compose stack, pinned by digest
docs/                      Plan, components, wiring, shopping list
```

## The protocol is generated, not hand-written

`protocol/protocol.yaml` is the only place message layouts are defined. Both
bindings are generated from it and committed:

```bash
make protocol         # regenerate
make protocol-check   # CI gate: fails if the committed files are stale
```

Every frame carries a protocol version byte. **The bridge refuses to send a
single motor command to a board whose version disagrees** — that check is what
makes remote firmware updates safe rather than exciting.

Wire format is COBS-framed with a CRC16, so resynchronising after a dropped byte
is "discard until the next `0x00`". The C++ and Python implementations are
pinned to byte-identical test vectors in
`firmware/test/native/test_codec.cpp` and
`ros2_ws/src/kobold_bridge/test/test_link.py`; if they ever drift apart, one
of those suites goes red long before a robot goes quiet on the bench.

## Documentation

| Document | What's in it |
|---|---|
| [Project plan](docs/PROJECT_PLAN.md) | Architecture, model stack, power, navigation, phases, risks |
| [Components](docs/COMPONENTS.md) | Every part, its role, and what's deliberately shelved |
| [Wiring](docs/WIRING.md) | Pin maps, power rails, the safety line, build notes |
| [Shopping list](docs/SHOPPING_LIST.md) | What to buy, in priority order |

## Roadmap

- [x] **0** Protocol, framing codec, cross-language tests
- [x] **1** Drive firmware — motors, encoders, IMU, PID, watchdog
- [x] **2** Sense firmware + hardware cliff/safety reflex
- [x] **3** ROS 2 bridge, EKF config, URDF, simulator
- [ ] **4** Camera + NPU: RKNN runtime, YOLO, video in the app
- [ ] **5** Navigation: lidar + Nav2 + SLAM, mono depth into the costmap
- [ ] **6** LLM + agent + voice (English)
- [ ] **7** Object search from a photo
- [ ] **8** Cat game, room nodes, mic array

## Hardware

Radxa Rock 5B (8 GB) · 3× ESP32 DevKit V1 · botland 4WD chassis with encoders ·
4× HC-SR04 · 6× IR · MPU-6050 · 2× 3S 18650 packs (~77 Wh, ~2.8 h) ·
IMX219 camera. Full inventory and reasoning in [docs/COMPONENTS.md](docs/COMPONENTS.md).

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md). The short version: run `make check`
before opening a PR, and if you change `protocol.yaml`, run `make protocol` and
commit the generated files.

## Safety

This is a wheeled robot that shares space with animals and drives on surfaces it
can fall off. Three rules that are not negotiable:

1. **Test the cliff reflex on the floor** — drive at a taped line and confirm it
   stops — before putting the robot on a table.
2. **Fit wheel guards** before the first session with a cat.
3. **Keep the motor-rail kill switch reachable.** It cuts the motors while
   leaving the computer running, so you can see what happened.

## License

MIT — see [LICENSE](LICENSE).
