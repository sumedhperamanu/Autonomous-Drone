# 0002: Companion computer

Date: 2026-11-04 · Owner: P2 · Status: proposed (freeze at Gate 1)

## Decision
**Jetson Orin Nano Super 8 GB.** Developer kit on the bench now; compact flight carrier for the drone (must have USB 3 and a UART).

Fallback, in order:
1. Reduce load in software: camera at 15 fps instead of 30, fewer topics recorded in flight, or move SLAM to cuVSLAM on the GPU (see 0001).
2. Orin NX 16 GB, only if Day 6 measurements show the Nano Super can't carry the load **and** the team approves the extra cost (module alone is roughly $1,000+ CAD).

Replaces the Raspberry Pi 5 4 GB in P1's Day 3–4 build list.

## Requirements
Load estimate for everything running at once in flight (640 × 480 at ~30 fps). Rough starting numbers; replace with measured values on Day 6.

| Program running in flight | Owner | CPU cores | Memory |
|---|---|---|---|
| uXRCE-DDS agent + PX4 link | P2 | 0.1 | 50 MB |
| Mission state machine + offboard control | P2 | 0.2 | 100 MB |
| OAK-D Lite driver (depth on camera) | P3 | 0.5–1 | 300 MB |
| LD19 driver | P3 | 0.1–0.2 | 50 MB |
| SLAM: RTAB-Map odometry + mapping | P2 | 1.5–2.5 | 1–2 GB, grows with the map |
| Moving-obstacle tracking | P2 | 0.3–0.5 | 200 MB |
| Marker (AprilTag) detection | P3 | 0.5–1 | 200 MB |
| Depth pipeline: crop, downsample, nearest obstacle | P3 | 0.5–1 | 300 MB |
| Flight data recording (rosbag) | P2 | 0.3 | 200 MB + fast storage |
| Ubuntu + ROS 2 overhead | — | 0.5 | 1 GB |
| **Total** | | **about 4.5–7 cores** | **about 3.5–5.5 GB** |
| **With 30% headroom** | | **about 6–9 cores** | **about 4.5–7 GB** |

Minimum spec:
- **CPU:** 6+ modern ARM cores (Cortex-A76/A78 class). 8 preferred if Day 6 measurements land near the high estimate.
- **Memory:** 8 GB minimum.
- **GPU:** not required for RTAB-Map; required for the cuVSLAM backup or a neural-network detector.
- **Ports:** USB 3 (OAK-D Lite); USB (LD19); UART to flight controller TELEM1 at 921600 (ROS 2 over uXRCE-DDS); USB to flight controller (MAVLink to QGC); Wi-Fi for ground debugging.
- **Storage:** 64 GB+, ideally NVMe SSD.
- **Software:** Ubuntu 24.04, so ROS 2 Jazzy installs the same way as in the dev container.

## Options

| | Raspberry Pi 5, 8 GB | Jetson Orin Nano Super, 8 GB | Jetson Orin NX, 16 GB |
|---|---|---|---|
| CPU | 4 × Cortex-A76, 2.4 GHz | 6 × Cortex-A78AE, 1.7 GHz | 8 × Cortex-A78AE, 2.0 GHz |
| GPU | None | 1024-core Ampere | 1024-core Ampere |
| Memory | 8 GB | 8 GB, shared with GPU | 16 GB, shared with GPU |
| Ubuntu 24.04 | Yes | Yes, JetPack 7.2 | Yes, JetPack 7.2 |
| Power (approx.) | 5–12 W | 7–25 W | 10–40 W |
| Flight mass (approx.) | 50–70 g | 120–180 g on a carrier | ~160 g on a carrier |
| Price (USD) | ~$130 | $249 dev kit + flight carrier | ~$700–1,000 module; ~$1,500+ as a kit |

### Gates

| Must-have | Pi 5 | Orin Nano Super | Orin NX 16 |
|---|---|---|---|
| Ubuntu 24.04 + Jazzy | Pass | Pass | Pass |
| 8 GB memory or more | Pass | Pass | Pass |
| 6+ modern CPU cores | **Fail (4)** | Pass | Pass |
| USB 3 + UART/Ethernet | Pass | Pass | Pass |

### Scores (1–5)

| Criterion | Weight | Orin Nano Super | Orin NX 16 |
|---|---|---|---|
| Compute headroom for SLAM + perception | 30% | 4 | 5 |
| Software fit (JetPack 7.2, Jazzy, Isaac ROS option) | 20% | 4 | 4 |
| Mass | 15% | 3 | 3 |
| Power draw | 10% | 3 | 2 |
| Cost | 15% | 4 | 1 |
| Ports and integration | 10% | 4 | 4 |
| **Weighted total** | 100% | **3.75** | **3.50** |

Sensitivity: moving 10 points from cost to headroom (40% / 5%) gives Nano Super 3.75, NX 3.90, so the winner flips. The choice rests on cost. With a realistic budget of $1,200–1,500 CAD for the whole drone, cost decides it for the Nano Super.

## Effects on other people's work
- **P1, power:** the Matek BEC12S-PRO (5.2 V, 5 A) was sized for the Pi. The Jetson draws up to 25 W, plus the OAK-D Lite and LD19 on its USB (~30 W total). Regulator to be chosen to match the flight carrier's input voltage.
- **P1, mass:** about +70–110 g over the Pi, so ~1,420 g all-up. Rerun eCalc at ~1,420 g with ~2–2.5 A electronics; update sim body mass.
- **P1, ports:** unchanged (TELEM1 ROS 2, USB MAVLink). Rename "Pi 5" to "companion" in params/README.

## Risks
- **Load near the top of the estimate.** Moving-obstacle tracking pushes the high case past 6 cores with headroom. Mitigation: measure on Day 6; apply the software fallback before any hardware swap.
- **8 GB shared with the GPU.** Measure memory on Day 6, especially if cuVSLAM or a neural detector is used.
- **Power regulator not chosen yet.** A brown-out resets the Jetson mid-flight. Owner: P1.
- **Flight carrier not chosen yet.** Must have USB 3, a UART, and acceptable mass. Owner: P2.
- **Prices move.** Confirm all prices against sellers before the Day 6 BOM.
