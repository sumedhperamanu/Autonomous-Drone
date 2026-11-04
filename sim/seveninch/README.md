# 7" model

How to fly it is in `sim/README.md`. This file is the numbers inside the model and where they came from.

Three airframes use it:

| Airframe | make target | Motors and battery | Top motor speed | Hover output |
| --- | --- | --- | --- | --- |
| 22054 | `gz_seveninch_4s` | 2807 1700 KV, 4S 5000 (the pick) | 1964 rad/s | 0.39 |
| 22053 | `gz_seveninch_lidar_flow` | same 4S setup + lidar, flow sensor, no GPS | 1964 rad/s | 0.39 (not flown yet) |
| 22052 | `gz_seveninch` | 2807 1300 KV, 6S 4000 (dropped, kept to compare) | 2213 rad/s | 0.33 |

All three share one body. It's 1.32 kg, which is the 4S drone with the phase 2 sensors (1.31 kg in eCalc).

| Number | Value | File | Where it came from |
| --- | --- | --- | --- |
| Mass | 1.32 kg | `models/seveninch_base` | Parts list: payload, frame, motors, props, ESC, battery, plus 10% |
| Inertia Ixx / Iyy / Izz | 0.0040 / 0.0054 / 0.0074 kg m^2 | `models/seveninch_base` | Each part placed as a box or point mass (`sim/calc/inertia.py`). Worked out with the 6S 4000 pack; the 4S 5000 pack we're buying is about 5% more on roll and yaw, 2% on pitch |
| Rotor positions | +-0.104 m (295 mm motor to motor) | SDF rotor poses, `CA_ROTORn_PX/PY` | The eCalc frame size |
| maxRotVelocity | 2213 rad/s on 6S, 1964 on 4S | `models/seveninch`, `SIM_GZ_EC_MAX1..4` | Full-throttle RPM from my motor model (`sim/calc/four.py`) |
| motorConstant | 3.661e-06 | `models/seveninch` | Max thrust per motor / maxRotVelocity^2 |
| momentConstant | 0.012 | `models/seveninch` | Prop torque / thrust for a 7x3.5 two-blade |
| Motor spin up / down | 0.03 / 0.04 s | `models/seveninch` | Heavier prop than the 5" (0.02 / 0.025 s). My estimate |
| Idle speed | 15% of max | `SIM_GZ_EC_MIN1..4` | Same ratio as the x500 and the 5" |
| rotorDragCoefficient | 1.47e-04 | `models/seveninch` | Prop drag (the x500's 8.06e-05) plus 0.04 m^2 of frame drag area, matched at 10 m/s. Only matters in wind or when moving |
| Rate gains | `MC_ROLLRATE_K` = `MC_PITCHRATE_K` = 0.5 | `airframes/22052_gz_seveninch` | 6S sweep: smooth up to 0.85, shook at 1.0. The 4S airframes use the same gains; the 4S drone never shook in its sweep, even at 1.0 |

The motor model was fitted to the Hobbywing XRotor 2807SL thrust table (within about 5%), then switched to a two-blade
prop. My eCalc runs on Nov 3 matched it within 15% on thrust and hover throttle, so I didn't change the model.
eCalc's hover time came out about 19% shorter than my model's, so trust eCalc for flight time.

## Not done yet

- The TBS Source One V5 frame we picked is 320 mm motor to motor, not 295. That moves the rotors from +-0.104 to
  +-0.113 m (SDF poses and `CA_ROTORn_PX/PY`). Small effect, but it should match before tuning.
  Update the inertia to the 4S pack at the same time (numbers in `sim/calc/inertia.py`), then re-fly the step tests.
- The sensor version (22053) needs a GPU for the lidar and flow camera, so I haven't flown it headless.

## Changing the motor numbers

If we swap motors or props, take these from eCalc: all-up weight, full-throttle RPM, max thrust per motor.

1. maxRotVelocity = RPM x 2 x pi / 60. Put it in `models/seveninch/model.sdf` (all 4 motors) and `SIM_GZ_EC_MAX1..4`.
2. motorConstant = max thrust per motor in N / maxRotVelocity^2.
3. `SIM_GZ_EC_MIN1..4` = 0.15 x maxRotVelocity.
4. Body mass = all-up weight - 4 x prop mass. Scale the inertia by the same ratio.
5. `MPC_THR_HOVER` = (sqrt(weight in N / 4 / motorConstant) - idle) / (max - idle).

Then run `bash sim/install.sh` again and fly it. The hover motor output should come out close to step 5.

## How the 7" model files are made

The files in `models/` are generated from the 5" models by `sim/calc/gen7.py`. It scales the frame, swaps the
prop meshes and sets every 7" number (mass, inertia, motor numbers) by name, so nothing from the 5" can carry over.
If you change a 7" number, change it in `gen7.py` and regenerate. To check the committed files still match the script:

```bash
python3 sim/calc/gen7.py sim/fiveinch /tmp/gen7 && diff -r /tmp/gen7/models sim/seveninch/models
```

No output from `diff` means they match.
