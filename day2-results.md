# Day 2 results (P1, 2026-10-29)

## Design (phase 1, budget build)

| Item | Value |
| --- | --- |
| All-up weight | 840 g (764 g + 10% margin) |
| Frame | 220 mm motor to motor (5") |
| Motors | 2207, about 1750 KV |
| Props | 5.1x3x3 (tri-blade) |
| Battery | 6S 1800 mAh 80C |
| ESC | 50 A 4-in-1, 250k eRPM or more |

## eCalc (840 g)

| Result | Value |
| --- | --- |
| Thrust-to-weight | 4.5 |
| Hover throttle | 36% |
| Flight time | 10.0 min hover, 5.1 min mixed |
| Max current | 85 A |

At 965 g (+15%): T/W 3.9, hover 38%, 8.3 / 4.7 min.

## Sim airframe numbers (`sim/fiveinch`)

| Number | Value |
| --- | --- |
| Mass | 0.84 kg |
| Inertia Ixx / Iyy / Izz | 0.0016 / 0.0023 / 0.0035 kg·m² |
| Arm (rotor offset) | ±0.078 m in x and y |
| motorConstant | 1.05e-06 |
| maxRotVelocity | 3121 rad/s |
| Rate gain scale | MC_ROLLRATE_K / MC_PITCHRATE_K = 0.2 |

## Sim test results

| Test | Result |
| --- | --- |
| Spawns and takes off to 2.5 m (`gz_fiveinch`) | Pass |
| Hover motor output | 0.349 on all 4 (eCalc: 0.36) |
| Stable hover | Pass after scaling rate gains to 0.2 (shook with x500 gains) |
| 360° lidar (`obstacle_distance`) | Pass: 72 sectors, walls seen at 4.5 m |
| Downward range sensor | Pass: 0.07 m on the ground |
| Optical flow | Pass: quality 255 |
| Position hold without GPS | To do |
| Stop 1 m before a wall | To do (needs QGroundControl connected) |

## Fixes found in sim

- Prop inertia was physically impossible (Ixx + Iyy < Izz), so Gazebo refused to spawn the model. Fixed to 3.1e-06.
- NAV_DLL_ACT copied from x500 blocked arming without a ground station. Set to 0 for the sim.
- x500 rate gains made the 5" oscillate. Scaled to 20%. The real drone still needs its own tuning (autotune, start low).
