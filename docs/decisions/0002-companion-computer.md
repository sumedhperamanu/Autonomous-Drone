# 0002: Companion computer

Date: 2026-11-04 · Updated: 2026-11-15 · Owner: P2 · Status: accepted in the Jetson build parts list (Nov 14); freeze at Gate 1

## Decision
**Jetson Orin Nano Super Developer Kit (8 GB), flown as-is on its own reference carrier board.** The same unit is used on the bench (NVIDIA's 19 V adapter) and on the drone (12 V from the BEC). No separate flight carrier is bought.

- **Price:** US$399.99 (about $560 CAD) from NVIDIA, after NVIDIA's July 2026 increase from US$249. It ships from the US, so expect duties and brokerage. Before ordering, check Canadian sellers, a used Orin Nano dev kit (same hardware; "Super" is a free JetPack update) and NVIDIA's education discount.
- **What's in the box:** module, carrier board, heatsink and fan, a Wi-Fi + Bluetooth card with antennas, and a 19 V bench adapter.
- **Why the dev kit's carrier:** $0 extra, guaranteed JetPack 7.2 / Ubuntu 24.04 support from NVIDIA, 176 g, 100 × 79 mm, and its barrel jack accepts 9–20 V.
- **Power:** Matek BEC12S-PRO set to **12 V**, dedicated to the Jetson, through a 5.5 × 2.5 mm right-angle barrel pigtail (22 AWG or thicker), centre positive. 60 W capacity against ~35 W worst-case draw (Jetson up to 25 W plus the OAK-D Lite, D500 and cameras it powers). Needs ≥ 14 V in for a steady 12 V; near the end of a 4S pack its output sags to ~11 V, still inside the 9–20 V range.
- **Storage:** used 500–512 GB NVMe SSD, M.2 2280, 95%+ health (era.ca, $47), in the dev kit's M.2 slot. Install JetPack 7.2 (Ubuntu 24.04) straight onto it with NVIDIA's unified installer ISO from a USB stick (8 GB or larger); the Orin Nano dev kit no longer has an SD card image. No microSD is bought.
- **Telemetry:** the dev kit's Wi-Fi replaces the SiK radio (see Ports).
Fallback, in order:
1. Reduce load in software: smaller images, lower frame rates, fewer topics recorded in flight, side-camera detection on the GPU, or cuVSLAM on the GPU instead of RTAB-Map odometry (see 0001).
2. Orin NX 16 GB, only if Day 6 measurements show the Nano Super can't carry the load **and** the team approves the extra cost (module US$999 at NVIDIA's bulk price since July 2026).
Replaces the Raspberry Pi 5 in the Nov 10 "cheaper build with Pi 5" parts list.

## Requirements
Load estimate for everything running at once in flight, at **full settings**. Built from P3's measurements (PR #8, `bench/pi5`). The Jetson's cores are the same speed as the Pi 5's (Geekbench 6 single-core 778 vs 774), so per-core numbers carry over 1:1. Replace with measured Jetson values on Day 6.

| Program running in flight | Owner | Jetson cores (of 6) | Source |
|---|---|---|---|
| uXRCE-DDS agent + D500 driver | P2/P3 | 0.1–0.2 | Estimate |
| mavlink-router (MAVLink to QGC over Wi-Fi) | P2 | < 0.05 | Estimate |
| Mission state machine, planner, offboard control | P2 | 0.1–0.2 | Estimate |
| OAK-D Lite driver (depth on camera) | P3 | 0.1–0.3 | Estimate |
| Receiving full-size depth + mono images at 30 Hz | P3 | 0.45–0.65 | Measured (PR #8) |
| Obstacle perception: 640 × 480 depth + lidar → tracked objects → dodge decision | P3/P2 | ~0.75 | Measured (PR #8): 24.7 ms a frame at 30 Hz |
| Side cameras ×2 at 1640 × 1232, 30 fps: hardware ISP capture + motion/looming detection on the GPU | P3 | 0.3–0.5 | Estimate; to be added to P3's benchmark |
| SLAM odometry: RTAB-Map 320 × 240 at 15 Hz (or cuVSLAM on the GPU) | P2 | 0.2–0.6 | Published Pi 5 measurement |
| SLAM map update, once a second | P2 | 0.1–0.4 | Published Pi 5 measurement, worst case |
| Linux, ROS 2, logging (no raw images recorded in flight) | — | 0.2–0.4 | Estimate |
| **Total** | | **about 2.3–4.0 of 6** | 2–3.7 cores spare |

Memory: about 3.5–5.5 GB of 8 GB (shared with the GPU), growing with the map. Measure on Day 6.

No marker (AprilTag) detection: dropped from the project.

Minimum spec:
- **CPU:** enough cores for the full-settings load above with at least 0.75 core spare (P3's pass mark). That needs more than the Pi 5's 4 cores.
- **Memory:** 8 GB minimum.
- **GPU (CUDA):** side-camera detection at full resolution, the cuVSLAM backup, and any neural-network detector added later (for example, person following).
- **Ports:**
| Device | Jetson dev kit connection |
|---|---|
| OAK-D Lite | USB 3 Type-A, short right-angle USB-A to USB-C cable rated USB 3 (5 Gbps) |
| D500 lidar | USB Type-A via its CP2102 adapter board (micro-USB; 20 cm cable included). Tie the micro-USB plug down |
| Side cameras ×2 | Both CSI camera ports (22-pin to 15-pin cables), enabled with `jetson-io` |
| Flight controller TELEM1 (ROS 2, 921600 baud) | UART on the 40-pin header, `/dev/ttyTHS1`. FC TX → pin 10, FC RX → pin 8, GND → pin 6, FC 5 V left unconnected. Check whether TELEM1 is JST-GH or JST-SH before buying the cable |
| Flight controller USB (MAVLink) | USB Type-A, short USB-A to USB-C cable (USB 2 is fine) |
| Storage | M.2 NVMe slot |
| Telemetry + ground debugging | Built-in Wi-Fi on **5 GHz** (keeps clear of the 2.4 GHz ELRS link). `mavlink-routerd` forwards the FC's MAVLink to QGC on UDP 14550 |

- **Software:** Ubuntu 24.04 (JetPack 7.2), so ROS 2 Jazzy installs the same way as in the dev container.
## Options

| | Raspberry Pi 5, 8 GB | RK3588 board, 8 GB (e.g. ROCK 5B+) | Jetson Orin Nano Super, 8 GB | Jetson Orin NX, 16 GB |
|---|---|---|---|---|
| CPU | 4 × Cortex-A76, 2.4 GHz | 4 × A76 + 4 × A55 | 6 × Cortex-A78AE, 1.7 GHz | 8 × Cortex-A78AE, 2.0 GHz |
| Geekbench 6 single / multi | 774 / about half the Jetson | ~826 / ~2,900 | 778 / ~3,350 | — |
| GPU we can use (CUDA) | No | No (Mali GPU + 6 TOPS NPU, Rockchip tools only) | Yes, 1024-core Ampere | Yes, 1024-core Ampere |
| Ubuntu 24.04 | Official | Community image only | Official, JetPack 7.2 | Official, JetPack 7.2 |
| IMX219 on CSI | Supported | Hit-or-miss by kernel version | Supported | Depends on carrier |
| Power (approx.) | 5–12 W | 5–15 W | 7–25 W | 10–40 W |
| Flight mass (approx.) | 50–70 g | ~60–80 g | 176 g as the dev kit | ~160 g on a compact carrier |
| Price (USD list, Nov 2026) | $175 | ~$130 (Apr 2026, often sold out) | $399.99 dev kit | $999 module (bulk price) + carrier |

Prices rose sharply in 2026 with memory costs; confirm before ordering.

### Gates

| Must-have | Pi 5 | RK3588 | Orin Nano Super | Orin NX 16 |
|---|---|---|---|---|
| Ubuntu 24.04 + Jazzy | Pass | Community only | Pass | Pass |
| 8 GB memory or more | Pass | Pass | Pass | Pass |
| Full-settings load with ≥ 0.75 core spare | **Fail** (fits only reduced settings) | Marginal | Pass | Pass |
| USB 3 + UART | Pass | Pass | Pass | Pass |
| Two working CSI camera ports | Pass | **Risk** | Pass | Depends on carrier |

### Why the Jetson over the Pi 5
P3's benchmark (PR #8) shows the Pi 5 fits a **reduced** setup: 1.8–2.9 of 4 cores for front depth, lidar and SLAM at 320 × 240, before the side cameras. Adding the side cameras (0.5–1.5 cores) gives 2.3–4.4, against a limit of 3.25 cores to keep 0.75 spare. It fits only with half-resolution side cameras and simple motion detection, and P3's ×5 scaling case adds ~0.4 core.

| | Pi 5 (reduced setup) | Jetson (full setup) |
|---|---|---|
| Front depth | 320 × 240 | 640 × 480 |
| 7 cm object, front: seen at / speed limit | ~2.6 m / ≤ ~1.5 m/s | ~5.2 m / ≤ 3 m/s |
| 7 cm object, sides | ~1.75 m / ≤ ~1 m/s | ~3.5 m / ≤ 2 m/s |
| 20 cm object, front / sides | ≤ ~3.5 / ~2.5 m/s | ≤ 5 / 5 m/s |
| Spare CPU | ~0–1 core | ~2–3.7 cores |
| SLAM backup | None | cuVSLAM on the GPU |
| Cost difference in the parts list | — | about $312 more (~$78 each for 4, ~$104 for 3) |
| Mass / hover time | — | ~120 g heavier, ~1.5 min less hover |

Speed limits scale with detection distance (v = x / T, Dodge Analysis); approximate. Only the Jetson meets the proposed P-13 limits.

### Other options considered
- **RK3588 board** (~$210 CAD if in stock): ~87% of the Jetson's CPU at Pi 5 cost, but no CUDA (no cuVSLAM, no GPU side-camera detection), community-maintained Ubuntu, and unreliable IMX219 support. Rejected on software and camera risk.
- **Waveshare Orin Nano/NX base board** (US$90, no module): the module alone is ≥ US$399, so board + module + fan costs more than the dev kit. Only worth it with a cheap module.
- **ARK Just A Jetson** (~$950 CAD): lighter and takes battery voltage directly; not affordable.
- **Compact third-party carriers** (Seeed J401, Waveshare): ~US$100–200 plus a module; JetPack 7.2 support depends on the vendor.
## Effects on other people's work
- **P1, power:** BEC12S-PRO at 12 V, dedicated to the Jetson. Measure its output and the pigtail's polarity (centre positive) with a multimeter before connecting; the dev kit has no reverse-polarity protection.
- **P1, mass:** the 1,383 g estimate assumed a 567 g battery, GPS and the SiK radio. The build now has one Powerhobby 4S 5200 mAh 50C pack (listed as 473 g or 567 g; weigh it), no SiK (~15–25 g less) and GPS optional. Rerun eCalc at the measured weight with ~2–2.5 A electronics; update sim body mass with the 320 mm spacing and 4S inertia changes.
- **P1, parameters (Wi-Fi telemetry):** set `NAV_DLL_ACT` to disabled or hold, so a Wi-Fi drop doesn't trigger a failsafe; the RC-loss failsafe (`NAV_RCL_ACT`) covers the real emergency. Set `COM_OBL_RC_ACT` to land or hold, in case the Jetson stops sending offboard commands.
- **P1, ports:** TELEM1 ROS 2 and USB MAVLink unchanged; TELEM2 is now free. Rename "Pi 5" to "companion" in params/README. The Jetson's UART is `/dev/ttyTHS1` (P2, Day 8).
- **P3, benchmark:** add the two side cameras to `perception_load.py`, report detection range alongside timing, and run it on the Jetson on Day 6.
- **P1/P3, frame:** the dev kit needs a custom top plate on dampers with fan airflow; it is larger than the 30.5 mm stack mounting. Mount the Wi-Fi antennas off the carbon plates, angled differently, ≥ 5 cm from the ELRS antenna.
## Risks
- **Jetson stock and import costs:** dev kits have been out of stock in many places since the price rise. Mitigation: order early; compare Canadian sellers, used kits and the education discount.
- **Wi-Fi telemetry depends on the Jetson:** if it crashes or loses power, telemetry stops. Mitigation: RC link stays independent; ELRS telemetry shows battery, link and flight mode on the transmitter; failsafe parameters above.
- **Load higher than estimated:** the side cameras aren't measured yet, and real rooms have more features than the sim. Mitigation: 2+ cores spare, GPU offload, measure on Day 6, software fallback before any hardware swap.
- **8 GB shared with the GPU:** GPU detection and any cuVSLAM use eat into the same memory. Measure on Day 6.
- **Barrel plug or micro-USB vibrating loose** cuts power or the lidar mid-flight. Mitigation: right-angle plug, strain relief, a dab of hot glue once tested; soft-mount the board.
- **BEC sag near pack end** (to ~11 V) is within the 9–20 V input, but a damaged pack or a mis-set BEC is not. Mitigation: measure under load on the bench.
- **Prices move.** Confirm all prices against sellers before ordering (see "Parts list: Jetson build").
