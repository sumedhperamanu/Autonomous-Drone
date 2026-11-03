# Day 3-4 results (P1, Nov 2-3 2026)

Short version: we're going 7" on 4S. It's the only option that meets our written spec (1.2 to 1.8 kg, at least
2:1 thrust, 10+ min hover, 4S). With the phase 2 sensors on, eCalc gives it 14.6 min of hover against 8.7 for the 5".
Both drones pass the Day 4 step tests in sim. To fly them yourself, see `sim/README.md`.

## The 7" build

| Part | Pick |
| --- | --- |
| Frame | TBS Source One V5 7" (320 mm motor to motor, 123.5 g) |
| Motors | 2807, about 1700 KV |
| Props | 7x3.5 two-blade |
| Battery | 4S 5000 mAh, 50C or more |
| Flight controller | MicoAir743 V2 (PX4 v1.17 target, microSD logging) |
| ESC | MicoAir 55A AM32 4-in-1, DShot |
| 5 V for the Pi | Matek BEC12S-PRO, 5.2 V 5 A |
| RC | RadioMaster RP1 V2 receiver + Pocket ELRS transmitter |
| Companion computer | Raspberry Pi 5, 4 GB |
| Phase 1 sensors | LD19 360 lidar, MTF-01P flow + range |
| Weight | 1,230 g phase 1, 1,310 g with the OAK-D Lite and GPS (phase 2) |

## eCalc (Nov 3)

All runs on 4S, field elevation 330 m, 10 C, 1.2 A of electronics.

| | 5" at 840 g | 5" at 935 g | 7" at 1,230 g | 7" at 1,310 g | Spec |
| --- | --- | --- | --- | --- | --- |
| Motor | T-Motor V2208 2450 KV | same | T-Motor F80 1900 KV (stand-in) | same | |
| Battery | 4S 2700 45C | same | 4S 5000 80C | same | |
| Thrust-to-weight | 4.1 | 3.6 | 4.4 | 4.1 | 2 or more |
| Hover throttle | 37% | 40% | 36% | 38% | 50% or less |
| Hover time | 10.0 min | 8.7 min | 15.9 min | 14.6 min | 10 min or more |
| Mixed flying | 5.9 min | 5.4 min | 9.2 min | 8.7 min | |
| Power at hover | 186 W | 217 W | 219 W | 241 W | |
| Lift per watt | 4.62 g/W | 4.43 g/W | 5.69 g/W | 5.52 g/W | |
| Peak current | 27 A per motor, 107 A | same | 38 A per motor, 153 A | same | under the 55 A ESC |
| Battery load at full throttle | 40C | 40C | 31C | 31C | under the pack rating |

Notes:

- The 2807 1700 KV we'd actually buy should pull about 32 A per motor instead of the F80's 38 A.
- My first 7" run used a 2812 1155 KV motor. That's a 6S motor, and on 4S it only made 2.0:1 thrust at 60% hover
  throttle. Motor speed is about KV x volts, so KV has to match the battery.
- At mission weight the 7" lifts 40% more weight on 11% more power. Prop theory (ideal power = T^1.5 / sqrt(2 rho A))
  says 37% more lift per watt at equal weight, about 16% at these two weights. eCalc gives 25%.
- My own 7" motor model (in `sim/calc`) was within 15% of eCalc on thrust and hover throttle, but about 19% long on
  hover time (17.4 min against 14.6). The sim uses my model; quote eCalc for flight time.

## Sim models

| | 5" (22050) | 7" on 4S (22054) | 7" on 6S (22052) |
| --- | --- | --- | --- |
| Mass | 0.84 kg | 1.32 kg | 1.32 kg |
| Inertia Ixx / Iyy / Izz (kg m^2) | 0.0019 / 0.0022 / 0.0025 | 0.0040 / 0.0054 / 0.0074 | same |
| Rotor offset | +-0.078 m | +-0.104 m | +-0.104 m |
| Max motor speed | 3121 rad/s | 1964 rad/s | 2213 rad/s |
| motorConstant | 1.05e-06 | 3.661e-06 | 3.661e-06 |
| momentConstant | 0.016 | 0.012 | 0.012 |
| Motor spin up / down | 0.02 / 0.025 s | 0.03 / 0.04 s | 0.03 / 0.04 s |
| Rate gain (MC_ROLLRATE_K, MC_PITCHRATE_K) | 0.4 | 0.5 | 0.5 |
| MPC_THR_HOVER | 0.35 | 0.39 | 0.32 |

Details and how to change them: `sim/fiveinch/README.md`, `sim/seveninch/README.md`.

## Step tests at the baseline gains

Scripted with `sim/tests/` (same flight every time): take off to 2 m, hover, 2 -> 3 -> 2 m, 10 deg roll step,
10 deg pitch step, land. Headless Gazebo Harmonic, PX4 v1.17.0, lockstep.

Re-flown Nov 3 with the fixed hover-thrust measurement:

| | 5" (K 0.4) | 7" on 4S (K 0.5) | Pass |
| --- | --- | --- | --- |
| Hover motor output | 0.349 | 0.391 | under 0.5 |
| Altitude 2 -> 3 m: rise, overshoot, settle | 1.98 s, 0%, 3.0 s | 2.07 s, 0%, 3.3 s | overshoot under 10%, settle under 5 s |
| Roll 10 deg: rise, overshoot | 0.58 s, 0% | 0.48 s, 0% | rise under 1 s, overshoot under 15% |
| Pitch 10 deg: rise, overshoot | 0.57 s, 0% | 0.47 s, 0% | same |
| Rate-tracking lag, roll / pitch | 34 / 42 ms | 46 / 54 ms | under 60 ms |
| Hover wobble | 0.19 deg/s | 0.19 deg/s | under 1 deg/s |

First runs (Nov 3, before the fix), including the 6S 7":

| | 5" (K 0.4) | 7" on 6S (K 0.5) | 7" on 4S (K 0.5) |
| --- | --- | --- | --- |
| Hover motor output | 0.349 | 0.326 | 0.386 |
| Altitude 2 -> 3 m: rise, overshoot, settle | 2.2 s, 2%, 3.2 s | 2.1 s, 1%, 3.1 s | 2.1 s, 0%, 3.4 s |
| Roll 10 deg: rise, overshoot | 0.58 s, 0% | 0.48 s, 0% | 0.47 s, 0% |
| Pitch 10 deg: rise, overshoot | about 1 s, 0% | 0.47 s, 0% | 0.45 s, 0% |
| Rate-tracking lag, roll / pitch | 40 / 44 ms | 48 / 46 ms | 50 / 46 ms |
| Hover wobble | 0.25 deg/s | 0.20 deg/s | 0.19 deg/s |

The old script measured hover thrust from queued climb messages, about 20% low. That made the drone sink during
the angle steps (about 0.4 m), and the 5" pitch step looked slow. With the fix the altitude stays flat through both
steps and the 5" pitch step is 0.57 s, same as its roll. Plots: `docs/p1/plots/` (both are the re-flown runs).

Lag changes by about 6 ms between identical runs (three 7" runs gave 36, 40 and 48 ms), so small differences don't
mean anything. The angle steps settle at about 90% of the target inside the 1 s window: rotor drag pulls back a bit as
the drone speeds up sideways, until the integrator catches up.

## Rate gain sweep

Hover wobble in deg/s at each `MC_ROLLRATE_K` = `MC_PITCHRATE_K`. Shaking runs marked with *.

| K | 5" (old hand inertia) | 5" (computed inertia) | 7" on 6S | 7" on 4S |
| --- | --- | --- | --- | --- |
| 0.2 | 0.22 to 0.32 | | 0.36 | |
| 0.3 | 0.33 | | 0.24 | |
| 0.4 | 0.24 | 0.25 | 0.18 | |
| 0.5 | 0.22 | | 0.16 to 0.20 | 0.19 |
| 0.6 | 0.24 | | | |
| 0.7 | 63 * | 0.17 | 0.24 | 0.19 |
| 0.85 | | 71 * | 0.34 | |
| 1.0 | 48 * | 25 * | 54 * | 0.27 |

The jump from under 1 to 25+ deg/s is the shake point. Baseline gain = about half of it (gain margin of about 2).
Recomputing the 5" inertia the same way as the 7" raised its roll inertia 19% and moved its shake point up.

## Wind test (Nov 3)

`sim/wind.py` with Waterloo numbers: `typical` is 5 m/s gusting to about 8, `windy` is 9 m/s gusting to about 15.
Headless, 45 s each, GPS position hold at 2.5 m, baseline gains. How the wind is modelled: "Wind" in `sim/README.md`.

| | 7" typical | 7" windy | 5" typical | 5" windy |
| --- | --- | --- | --- | --- |
| Lean, average / peak | 11 / 21 deg | 17 / 33 deg | 9 / 22 deg | 16 / 31 deg |
| Pushed off its spot, peak | 1.3 m | 1.6 m | 0.9 m | 1.8 m |
| Height while holding 2.5 m | 2.45 to 2.63 m | 2.38 to 2.70 m | 2.34 to 2.56 m | 2.34 to 2.64 m |
| Highest motor output | 0.42 | 0.42 | 0.38 | 0.41 |

Both hold on with lots of thrust spare. The weak spot is position: gusts push the drone 1 to 2 m, against P3's
0.5 m obstacle margin. The position gains are still PX4 defaults, so that's something to tune (and to retest with
flow instead of GPS).

## Parameter files (`params/`)

- `sim_fiveinch_baseline.params`, `sim_seveninch_baseline.params`: the gains and hover thrust above, plus P3's speed
  limits and the failsafes. The speed limits and failsafes weren't loaded in the step-test flights, so they still
  need their own sim test.
- `hw_seveninch_first_flight.params`: the real 7". Port map, 4S battery setup, failsafes, no-GPS height source,
  and the board's own starting gains (not the sim ones).

Failsafes in both:

| What goes wrong | What the drone does |
| --- | --- |
| RC link lost | Return (`NAV_RCL_ACT 2`). No GPS, so PX4 lands where it is |
| Battery critical (15%) | Return (`COM_LOW_BAT_ACT 3`), so also lands. Warning at 25%, emergency land at 7% |
| Return height | 2.5 m, descend to 2.0 m (the defaults are 60 m and 30 m) |
| ROS 2 stops sending for 0.5 s | Land (`COM_OBL_RC_ACT 4`). In offboard, losing the position estimate counts as this too |
| Flow sensor stops | Position counted as lost after 1 s (`EKF2_NOAID_TOUT`) instead of 5 s |
| Wi-Fi to QGC drops | Nothing (`NAV_DLL_ACT 0`). The RC link is the safety link |

## Fixes found along the way

- The 7" sim parameter file had the 6S hover thrust (0.32) after we moved to 4S. Now 0.39.
- `step_tests.py` hover thrust bug (above).
- 5" inertia recomputed (it was hand estimates), and its rate gain went 0.2 -> 0.4.
- ROS 2 link lost: changed from hover (Position mode) to Land after reading PX4's offboard check in the source.
- Real-drone file now sets the barometer as the height source (`EKF2_HGT_REF 0`) and turns GPS off. PX4 defaults to
  GPS for height, which this build doesn't have.
- TELEM1 clash: the board puts MAVLink on TELEM1 by default, where ROS 2 goes. `MAV_0_CONFIG 0` turns it off.

## Not done yet

- The sensor versions (`gz_fiveinch_lidar_flow`, `gz_seveninch_lidar_flow`) need a GPU for the lidar and flow
  camera, so they aren't in the headless tests. The 5" one passed on Oct 29 (72 lidar readings, flow quality 255).
  The 7" one boots and spawns with the right parameters (4S motors, 0.39 hover, no GPS, 1.5 m/s limit), but
  nobody has flown it with the Gazebo window yet.
- Failsafes in sim: drain the battery with `SIM_BAT_DRAIN` and cut the offboard stream.
- The sim 7" uses 295 mm motor spacing. The TBS frame is 320 mm (rotors at +-0.113 m).
- Angle gains (`MC_ROLL_P`, `MC_PITCH_P`) are still PX4 defaults. That's Day 5.
- Position hold in gusts: tune `MPC_XY_P` / `MPC_XY_VEL_*` with `sim/wind.py`, then retest on flow instead of GPS.
