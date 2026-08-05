# Shopping List

*Rev 2 — reprioritised after hardware verification. The PD source module is gone (the GPIO power
path works), and the lidar is demoted from essential to optional now that a stereo rig costs €0.*

Prices are rough EU/botland estimates. Tiers are ordered so that stopping at any point still leaves
a working robot at that level of capability.

---

## Tier 0 — buy this week (~€35)

Small, cheap, and each one prevents a specific failure the build is currently exposed to.

| Item | Qty | ~Price | Why |
|---|---|---|---|
| **TVS diode, 5.6–6.0 V** (SMBJ5.0A or similar) | 2 | €1 | Your GPIO power path bypasses the Rock 5B's input protection. If the XL4016's high-side switch fails short, 12.6 V lands on a €150 board. **Cheapest insurance in this document** |
| **Low-ESR electrolytic, 1000 µF 16 V** | 4 | €3 | One at the Rock 5B header, one at each motor driver. Prevents brownout resets when motors stall |
| **Blade fuse holders + fuses** (2/3/7.5/10 A) | 1 set | €5 | Non-optional on a lithium build |
| **Main kill switch, 15 A+** | 1 | €4 | On the motor rail. The control to reach for when the robot does something unexpected |
| **XT30/XT60 connectors + 18 AWG silicone wire** | 1 set | €10 | Dupont jumpers on a motor rail melt connectors |
| **Small heatsinks** (for the L293Ds, while they last) | 2 | €2 | You're running them at 8 V. Measure chip temp after 10 min of driving — above ~70 °C they're already throttling |

---

## Tier 1 — makes the robot work as designed (~€20)

| Item | Qty | ~Price | Why |
|---|---|---|---|
| **TB6612FNG motor driver** | 2 | €6 | Same wheel voltage at 6.5 V that the L293D needs 8 V to reach, a quarter of the waste heat, double the current headroom, and 3.3 V logic so no level shifters. Your 8 V workaround is treating the symptom |
| **2D lidar — LD19/LD06 or RPLIDAR C1** | 1 | €70–90 | **Promoted back to essential.** The €0 stereo plan is dead — the ROCK 5B has one CSI connector, and the two IMX219s have mismatched lenses. Mono depth on the NPU covers 3D obstacles, but localisation needs this. Lidar works in the dark, against blank walls, and over plain carpet, and needs no calibration |
| **IR Flying-Fish modules** (left + right horizontal) | 2 | €4 | All four corner sensors now point down for table driving, leaving the **sides with no close-range coverage**. Skid steer sweeps sideways in every turn |
| **INA226 current/voltage sensor** | 1 | €3 | Real battery telemetry over I²C. The agent says "15%, heading to the dock" instead of dying mid-room |
| **Heat-shrink, sleeving, zip ties, standoffs** | — | €10 | Cable management on a moving robot isn't cosmetic — a loose wire in a wheel ends the session |

---

## Tier 2 — optional camera and stereo upgrades

Only after the lidar is working. None of this is needed for the planned build.

| Item | ~Price | Why |
|---|---|---|
| **Wide-angle M12 lens for IMX219** | €10 | Only if both modules prove narrow after the FOV test (PROJECT_PLAN §6). Wider FOV directly improves visual place recognition |
| **2× matched USB webcams** | €30 | The straightforward route to stereo — no CSI limits, no device tree overlay work. Rolling shutter and no hardware sync, but it works |
| **OAK-D Lite** | €150 | The expensive route to stereo. Does depth on-device, costing no RK3588 CPU. Genuinely good, but lidar plus mono depth covers most of the same ground for half the price |

---

## Tier 3 — voice hardware (~€80–100)

Deferred; the phone app covers it. Nothing above the `voice`
container changes — same pipeline, different audio source.

| Item | ~Price | Why |
|---|---|---|
| **ReSpeaker USB Mic Array v2.0** (4-mic) | €70 | **Get USB, not a Pi HAT.** HAT versions need the `seeed-voicecard` kernel driver for their WM8960/AC108 codec, which is Raspberry Pi specific — porting it to RK3588 is a real, unrewarding project. USB Audio Class just appears as an ALSA device on any board. Onboard DSP does beamforming, AEC, and **direction-of-arrival**, so the robot can turn toward whoever spoke |
| — or **MiniDSP UMA-8** (7-mic) | €90 | Same argument, more mics |
| **PAM8403 amp + 4 Ω speaker** | €8 | Piper TTS output. Mount facing forward — a speaker firing into the chassis sounds terrible |

---

## Tier 4 — later

| Item | ~Price | Why |
|---|---|---|
| **Charging dock** (contacts + IP2721 + buck + Zero 3W) | €20 | Uses parts already owned. Self-docking is the difference between a project and a robot that lives in a house |
| **Room node kit** (mount + USB mic for the Zero 3W) | €15 | Second viewpoint and second listening point. Directly useful for the cat game — the robot finds the cat even when the cat isn't in its own FOV |
| **Wheel guards** | — | Print or cut. **Before the first cat session** — exposed wheels, paws, and tails |
| **Second 2 TB NVMe** | — | When the rosbag archive on the Odroid fills up. It will |
| **IMU upgrade — BNO055 / ICM-20948** | €25 | Onboard fusion with a magnetometer. Only if MPU-6050 yaw drift is a real problem after EKF tuning |
| **Depth camera** (OAK-D Lite) | €150 | Only if both stereo *and* lidar disappoint. Unlikely |

---

## Explicitly not buying

| Item | Why not |
|---|---|
| ~~12.6 V CC/CV charger~~ | **Resolved — already covered.** The KORAD KA3005D bench supply *is* a CC/CV source, with an adjustable current limit and a live current readout no €10 charger provides. See COMPONENTS §11 |
| ~~USB-C PD source module~~ | **Resolved** — the 5.1 V GPIO header path works. Add the TVS |
| ~~M.2 2242→2280 adapter~~ | **Resolved** — PM991 is 2280 |
| ~~MHF4 antennas~~ | **Resolved** — the A8 shipped with them |
| ~~3S balance charger~~ | **Resolved** — the BMS boards balance |
| A more powerful SBC | The Rock 5B's 6 TOPS NPU suits a 2–4B model well. Compute isn't the bottleneck |
| A replacement for the NCS2 | Covered by the RK3588 NPU. The stick is shelved, not replaced |
| Motor upgrades | The chassis motors are fine. Fix the *driver*, not the motors |
| A CSI splitter + second IMX219 for MIPI stereo | One CSI connector *can* split to 2×2-lane, but there's no official adapter, no documented overlay, and it would mean writing a dual-sensor device tree from scratch. Research project, not a build step |
| An x86 PC for model conversion | Pre-converted HuggingFace models first, then Rosetta Docker, then Parallels x86, then a €0.50/hr cloud VM. Don't buy a computer to run a conversion script |

---

## Buying exactly one thing

The **TVS diode**. Thirty cents, and it is the only thing standing between a shorted buck regulator
and a dead Rock 5B on a power path that has no input protection.

## Buying exactly five things

**TVS diode**, **2× TB6612FNG**, fuses + kill switch, **bulk caps**, **lidar**. About €95, and it
moves the build from "works on the bench" to "navigates a room safely."

---

*See [COMPONENTS.md](COMPONENTS.md) for the existing inventory and
[PROJECT_PLAN.md](PROJECT_PLAN.md) for how it fits together.*
