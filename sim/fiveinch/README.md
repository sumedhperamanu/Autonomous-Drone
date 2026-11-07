# 5" model

How to fly it is in `sim/README.md`. This file is the numbers inside the model and where they came from.

| Airframe | make target | What it is | Hover output |
| --- | --- | --- | --- |
| 22050 | `gz_fiveinch` | Bare 5" with GPS, for checking the physics | 0.35 |
| 22051 | `gz_fiveinch_lidar_flow` | Phase 1 sensors: 360 lidar on a mast, downward flow + range, no GPS | 0.35 |

The motor numbers are from the Day 2 design (2207 1750 KV, 5.1x3x3, 6S 1800). On Nov 3 I ran the 5" on 4S in eCalc
(V2208 2450 KV, 4S 2700) and it came out close: thrust-to-weight 4.1 against 4.5, same 10.0 min hover at 840 g.
So the sim stays on the Day 2 numbers.

| Number | Value | File | Where it came from |
| --- | --- | --- | --- |
| Mass | 0.84 kg | `models/fiveinch_base` | Day 2 parts list + 10% |
| Inertia Ixx / Iyy / Izz | 0.0019 / 0.0022 / 0.0025 kg m^2 | `models/fiveinch_base` | Same parts-as-boxes method as the 7" (`sim/calc/inertia.py`) |
| Rotor positions | +-0.078 m (220 mm motor to motor) | SDF rotor poses, `CA_ROTORn_PX/PY` | Day 2 frame |
| maxRotVelocity | 3121 rad/s (29,805 RPM) | `models/fiveinch`, `SIM_GZ_EC_MAX1..4` | Day 2 eCalc |
| motorConstant | 1.05e-06 | `models/fiveinch` | Max thrust per motor / maxRotVelocity^2 |
| momentConstant | 0.016 | `models/fiveinch` | Prop torque / thrust |
| Motor spin up / down | 0.02 / 0.025 s | `models/fiveinch` | My estimate |
| rotorDragCoefficient | 6.0e-05 | `models/fiveinch` | Prop drag (the x500's value scaled down for 5" props) plus 0.025 m^2 of frame drag, matched at 10 m/s |
| Max throttle | 0.8 | `MPC_THR_MAX` | Keeps peak current inside the 6S 1800 pack's rating |
| Rate gains | `MC_ROLLRATE_K` = `MC_PITCHRATE_K` = 0.4 | `airframes/22050_gz_fiveinch` | Day 4 sweep: smooth at 0.7, shook at 0.85 |

The lidar version has an LD19-style scanner (12 m, 10 Hz, 72 readings) on a mast 75 mm up so it sees over the props,
Collision Prevention at 1 m, and P3's speed limits (1.5 m/s max, 2 m/s^2 braking). It passed the sensor check on
Oct 29: 72 distances, walls at about 450 cm, ground distance 0.07 m, flow quality 255.

## What changed since the 5-inch-sim branch

- Inertia recomputed the same way as the 7" (it was hand estimates: 0.0016 / 0.0023 / 0.0035). Roll inertia went up 19%.
- Rate gains 0.2 -> 0.4. The new inertia moved the shake point up, and 0.4 is about half of it.
- `MPC_THR_HOVER` 0.36 -> 0.35, the value PX4 learned in sim.
- `CA_ROTORn_KM` +-0.05 -> +-0.016 so it matches the model. PX4 rescales yaw anyway, so flight doesn't change.
- Speed limits in the lidar version: 1.5 m/s (was 3), from P3's requirements.
- `rotorDragCoefficient` 8.06e-05 (the x500's, for 13" props) -> 6.0e-05, so wind pushes the 5" and the 7" in a fair
  way. See "Wind" in `sim/README.md`. It doesn't change hover or the step tests.

The real LD19 has no PX4 driver. In the sim Gazebo hands the scan straight to PX4. On the real drone the Pi will do that.
