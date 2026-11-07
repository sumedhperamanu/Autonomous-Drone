# P1 parameter files (Day 4 baseline)

Load a file in QGroundControl: **Vehicle Setup > Parameters > Tools > Load from file**, then reboot the vehicle.
Lines starting with `#` are notes that explain each value; QGC skips them.

| File | For | What's in it |
| --- | --- | --- |
| `sim_fiveinch_baseline.params` | Gazebo, airframe 22050 | Rate gains from the Day 4 sweep, hover thrust, P3's speed limits, failsafes |
| `sim_seveninch_baseline.params` | Gazebo, airframe 22054 (4S) | Same, for the 7" on 4S (hover thrust 0.39) |
| `hw_seveninch_first_flight.params` | The real 7" (MicoAir743 V2 + MicoAir 55A) | Port map, battery setup, failsafes, the board's own starting gains |

## Why sim and real-drone files are separate

The sim files use gains tuned on a model. The real drone has motor delay, frame flex and vibration the model
doesn't, so its first flight starts at the board's default gains (`MC_ROLLRATE_K 0.35`, `MC_PITCHRATE_K 0.4`)
and gets tuned on the real thing.

Both kinds of file set `NAV_DLL_ACT 0`: when the QGroundControl link drops, the drone carries on. That's on
purpose for the real drone too. Wi-Fi to QGC drops often indoors, and the RC link is the safety link, not Wi-Fi.

## Port map (real drone, MicoAir743 V2)

| Port | Plugged in | PX4 settings |
| --- | --- | --- |
| RC | ELRS receiver (RadioMaster RP1 V2) | `RC_INPUT_PROTO -1` (auto-detect, board default) |
| TELEM1 | Pi 5, ROS 2 over uXRCE-DDS | `UXRCE_DDS_CFG 101`, `SER_TEL1_BAUD 921600` (default 57600), `MAV_0_CONFIG 0` (the board puts MAVLink here by default, and two programs on one port garble each other) |
| TELEM2 | MTF-01P flow + range sensor (MAVLink) | `MAV_1_CONFIG 102`, `SER_TEL2_BAUD 115200` |
| USB | Pi 5, MAVLink that the Pi forwards to QGroundControl | default |
| URT6, TELEM3, GPS1, GPS2 | Spare: SiK radio, GPS in phase 2 | Board default MAVLink on TELEM4 off (`MAV_2_CONFIG 0`, TELEM4 is wired to the onboard Bluetooth) and OSD on GPS2 off (`MSP_OSD_CONFIG 0`) |

For P2: on the real drone the ROS 2 link is this serial port (TELEM1, 921600 baud), so the XRCE-DDS agent on the
Pi runs as `MicroXRCEAgent serial --dev <port> -b 921600`. In sim it runs over UDP.

## Failsafes in both files

- RC lost and critical battery both trigger **Return**. With no GPS, PX4 can't run Return and lands instead
  (`src/modules/commander/failsafe/framework.cpp`, the RTL case falls through to Land).
- `RTL_RETURN_ALT 2.5` and `RTL_DESCEND_ALT 2.0`: the defaults (60 m and 30 m) would fly into an indoor ceiling.
- Offboard lost for 0.5 s: Land (`COM_OBL_RC_ACT 4`, `COM_OF_LOSS_T 0.5`). This fires when the ROS 2 node stops
  sending setpoints. In offboard flight a lost position estimate also counts as offboard lost, so this covers flow loss
  too (P3's P-11b), **but only while the node sends position or velocity setpoints** (`offboardCheck.cpp` only checks
  the estimate the setpoint type needs). With no position, PX4 falls through to Descend. The pilot can take over with
  the mode switch.
- P3's P-11a (depth older than 0.5 s: hover) is **not** in these files. PX4 never sees the depth data, so a node that
  keeps sending setpoints while depth is stale looks healthy to PX4. P-11a has to be checked in P2's offboard node.
  P1 and P3 still need to agree on hover vs land for it.
- No GPS in phase 1: `EKF2_HGT_REF 0` (barometer) and `EKF2_GPS_CTRL 0` (real-drone file only).
- `EKF2_NOAID_TOUT 1000000` (1 s): with flow only, declare the position lost 1 s after flow stops, not 5 s.

## What was and wasn't tested in sim

The step-test flights used the rate gains and hover thrust in these files. The motion limits and failsafes were **not** loaded in
those flights. Before trusting them, load the sim file, fly once, and trigger a low battery (`SIM_BAT_DRAIN`) and an offboard loss.

QGC flags `MPC_XY_CRUISE 1.0` and `MPC_VEL_MANUAL 1.5` because PX4 lists 3 m/s as their minimum. That's expected: P3's speed limit is 1.5 m/s.

## Before the real drone's first flight

1. Measure the pack with a multimeter and set `BAT1_V_DIV` in QGC's Power setup to match.
2. After the first pack, compare the mAh the charger put back with what PX4 logged, and correct `BAT1_A_PER_V`.
3. Check the flight-controller-to-ESC cable pinout with a multimeter, and power up through a smoke stopper.
4. Set the ELRS receiver's failsafe to "No Pulses".
5. Keep the microSD card in. This board stores its parameters there.
6. Calibrate the sensors in QGC (accelerometer, gyro, compass, level horizon) after the board is mounted in the frame.
7. Set up the actuators in QGC and test each motor with the props off: right motor, right direction, matching the airframe picture.
8. Put arm and kill switches on the transmitter, and test the kill switch with the props off before every first flight.
9. Optional: AM32 supports bidirectional DShot, and all four motor outputs on this board share one timer, so try `DSHOT_BIDIR_EN 1`.
   Then the gyro filter can follow motor RPM.

Regenerate these files with `sim/calc/make_params.py` (usage at the top of that file).
