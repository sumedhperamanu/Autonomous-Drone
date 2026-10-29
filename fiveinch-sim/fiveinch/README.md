# 5-inch quad sim model (phase 1 design)

This folder adds the team's 5" design to PX4 v1.17 + Gazebo Harmonic. It's built from the stock x500 model, with the eCalc numbers from 2026-10-29.

| Number | Value | Where |
| --- | --- | --- |
| Total mass | 0.84 kg | `models/fiveinch_base` (body 0.819 kg + 4 props) |
| Inertia Ixx / Iyy / Izz | 0.0016 / 0.0023 / 0.0035 kg·m² | `models/fiveinch_base` |
| Rotor positions | ±0.078 m (220 mm motor to motor) | SDF rotor poses + `CA_ROTORn_PX/PY` |
| `maxRotVelocity` | 3121 rad/s (29,805 RPM) | `models/fiveinch` |
| `motorConstant` | 1.05e-06 | `models/fiveinch` |
| Motor spin-up / down | 0.02 / 0.025 s | `models/fiveinch` |
| Hover throttle | 0.36 | `airframes/22050_gz_fiveinch` |
| Max throttle | 0.8 | `airframes/22050_gz_fiveinch` |

Two vehicles:

- **`gz_fiveinch`**: the bare drone with GPS, for checking that the physics fly right.
- **`gz_fiveinch_lidar_flow`**: the phase 1 build. It has a 360° 2D lidar (LD19 spec: 12 m, 10 Hz) on a mast, a downward optical flow and range sensor, and **no GPS**. Collision Prevention is set to 1.0 m and max speed to 3 m/s.

## Install (once)

Inside the dev container, from the kit folder:

```bash
bash sim/fiveinch/install.sh
```

This copies the models and airframes into `third_party/PX4-Autopilot`. Running it twice is safe.

## Test 1: does it fly like our design?

```bash
scripts/sim_up.sh gz_fiveinch
tmux attach -t sim          # Ctrl+B then a window number to switch; the px4 window has the pxh> console
```

The first build after installing takes a few minutes. Then, in the `pxh>` console:

| Step | Type | What you should see | If not |
| --- | --- | --- | --- |
| 1 | (wait) | A small quad sitting level on the ground in Gazebo. The console says `Ready for takeoff!` | Falls through the ground or tips over: send me the console output |
| 2 | `commander takeoff` | Climbs to about 2.5 m and holds still. Tiny wobble is fine | Flips right away: a rotor position or direction is wrong. Won't lift: motor numbers are wrong |
| 3 | `listener actuator_motors` | All 4 values close to each other, about **0.33 to 0.40** | Much higher means the sim drone is heavier or weaker than the eCalc design |
| 4 | `listener vehicle_local_position` | `z` steady near -2.5, `vx`/`vy` near 0 | Drifting more than about 0.5 m: send me the output |
| 5 | In QGroundControl, switch to Position mode and fly with the virtual joystick | Moves quickly and stops when you let go, with no growing shake | Shakes or buzzes: the x500's controller gains are too strong for this small drone (see Tuning) |
| 6 | `commander land` | Lands softly and disarms by itself | |

**Pass means:** steps 1 to 6 work and the motor values in step 3 are near 0.36. That's the sim agreeing with eCalc.

## Test 2: phase 1 sensors and obstacle stopping

```bash
scripts/sim_down.sh
scripts/sim_up.sh gz_fiveinch_lidar_flow walls
```

| Step | Type / do | What you should see |
| --- | --- | --- |
| 1 | (wait) | Lidar rays drawn around the drone in Gazebo. `Ready for takeoff!` with no GPS |
| 2 | `listener obstacle_distance` | 72 distances (one per 5°), in cm, smaller toward the nearest walls |
| 3 | `listener distance_sensor` | Downward distance about 0.03 to 0.1 m on the ground, about 2.5 m after takeoff |
| 4 | `listener sensor_optical_flow` | Messages arriving, with quality above 0 |
| 5 | `commander takeoff`, then Position mode in QGroundControl | Holds position without GPS, drifting less than about 0.5 m |
| 6 | Fly slowly at a wall | Slows and stops about **1 m** from it. QGroundControl shows a Collision Prevention message |
| 7 | `commander land` | Lands where it is (no GPS means no return-home) |

If the drone drifts in step 5, the floor may be too plain for the flow camera. Try the `lawn` or `baylands` world instead of `walls`.

## Tuning (only if it shakes in Test 1, step 5)

This small drone turns much faster than the 2 kg x500 its gains came from. In `pxh>`:

```
param set MC_ROLLRATE_P 0.10
param set MC_PITCHRATE_P 0.10
param set MC_ROLLRATE_D 0.002
param set MC_PITCHRATE_D 0.002
```

Lower them further in small steps until the shake stops, then write the values into `airframes/22050_gz_fiveinch`.

## Notes

- The models were generated from the PX4-gazebo-models `main` branch x500 files. If a newer x500 looks different from the one in your PX4 checkout, tell me.
- The sim lidar is drawn on a mast 75 mm above the body, so its scan clears the props.
- The real LD19 has no PX4 driver. In the sim, Gazebo feeds the scan to PX4 directly. On the real drone the Pi does that job.
