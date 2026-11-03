"""Write the Day 4 baseline parameter files (QGroundControl .params format: Parameters > Tools > Load from file).

Two kinds of file, kept apart on purpose (a design-review finding):
  sim_<drone>_baseline.params  the gains and hover thrust the Gazebo drone flew with in the Day 4 step tests.
      The motion limits and failsafes were NOT loaded in those flights: test them in sim before trusting them.
  hw_<drone>_first_flight.params  the real MicoAir743 V2 drone: ports, battery, failsafes, safe starting gains
Usage: make_params.py <outdir> <k5> <hover5> <k7> <hover7> <cells7> <capacity7_mAh>
The Nov 3 files came from:  python3 sim/calc/make_params.py params 0.4 0.35 0.5 0.39 4 5000
"""
import sys, pathlib
out = pathlib.Path(sys.argv[1]); out.mkdir(parents=True, exist_ok=True)
k5, h5, k7, h7 = map(float, sys.argv[2:6]); cells7, cap7 = int(sys.argv[6]), int(sys.argv[7])
F, I = 9, 6   # MAV_PARAM_TYPE_REAL32, MAV_PARAM_TYPE_INT32
DATE = "2026-11-03"

motion = [
  ("# Motion limits from P3's perception requirements (docs/PERCEPTION_REQUIREMENTS.md)",),
  ("MPC_XY_VEL_MAX", 1.5, F, "v_max 1.5 m/s"),
  ("MPC_XY_CRUISE", 1.0, F, "normal autonomous speed 1.0 m/s. Below PX4's listed minimum of 3, so QGC flags it: expected"),
  ("MPC_VEL_MANUAL", 1.5, F, "same cap when a person flies in Position mode. Also below the listed minimum of 3: expected"),
  ("MPC_ACC_HOR", 2.0, F, "braking 2 m/s^2, the number P3 used for stopping distance"),
]
failsafes = [
  ("# Failsafes. Team spec: return-home and low-battery failsafes.",),
  ("# Return needs a global position. With no GPS PX4 cannot run it and drops to Land by itself",),
  ("# (commander/failsafe/framework.cpp: case RTL falls through to case Land).",),
  ("NAV_RCL_ACT", 2, I, "RC lost -> Return (Land when there is no GPS)"),
  ("COM_LOW_BAT_ACT", 3, I, "Return at critical battery, land at emergency"),
  ("RTL_RETURN_ALT", 2.5, F, "default is 60 m. Indoors that flies into the ceiling. P3's max height is 3 m"),
  ("RTL_DESCEND_ALT", 2.0, F, "default is 30 m, same reason"),
  ("BAT_LOW_THR", 0.25, F, "warning at 25%"),
  ("BAT_CRIT_THR", 0.15, F, "critical (Return) at 15%"),
  ("BAT_EMERGEN_THR", 0.07, F, "emergency (Land) at 7%"),
  ("COM_OF_LOSS_T", 0.5, F, "offboard setpoints older than 0.5 s count as lost (P3's P-11a, 0.5 s)"),
  ("COM_OBL_RC_ACT", 4, I, "offboard lost -> Land. In offboard, a lost position estimate also counts as offboard lost\n#   (HealthAndArmingChecks/checks/offboardCheck.cpp), so this also lands on flow loss (P3's P-11b);\n#   with no position PX4 falls through to Descend (failsafe/framework.cpp). The pilot can take over with the mode switch"),
  ("EKF2_NOAID_TOUT", 1000000, I, "flow drone: declare position lost after 1 s without flow, not 5 s (in microseconds)"),
]

def write(name, rows):
    lines = []
    for r in rows:
        if len(r) == 1: lines.append(r[0]); continue
        n, v, t, why = r
        if why: lines.append(f"# {n}: {why}")
        lines.append(f"1\t1\t{n}\t{v}\t{t}")
    (out / name).write_text("\n".join(lines) + "\n")

def sim(label, k, hover):
    return [
      (f"# SIM baseline: {label}. PX4 v1.17, Gazebo Harmonic, {DATE}.",),
      ("# Load on top of the airframe defaults. Lines starting with # are notes and QGC skips them.",),
      ("# Vehicle-Id Component-Id Name Value Type",),
      ("# Rate gains from the Day 4 gain sweep (step_tests.py + analyze_steps.py)",),
      ("MC_ROLLRATE_K", k, F, "about half the gain where hover started to shake in sim"),
      ("MC_PITCHRATE_K", k, F, "same as roll; checked with the pitch step"),
      ("MPC_THR_HOVER", hover, F, "the hover thrust PX4 learned in sim (hover_thrust_estimate)"),
      ("NAV_DLL_ACT", 0, I, "SIM ONLY: no ground station needed"),
    ] + motion + failsafes

def hw(label, cells, cap):
    return [
      (f"# REAL DRONE first-flight file: {label}. MicoAir743 V2 + MicoAir 55A AM32. PX4 v1.17, {DATE}.",),
      ("# This board keeps its parameters on the microSD card. Never fly without the card.",),
      ("# Vehicle-Id Component-Id Name Value Type",),
      ("# Ports (see the port map in the P1 doc)",),
      ("MAV_0_CONFIG", 0, I, "board default puts MAVLink on TELEM1, which would clash with ROS 2 on TELEM1"),
      ("UXRCE_DDS_CFG", 101, I, "ROS 2 link to the Pi on TELEM1"),
      ("SER_TEL1_BAUD", 921600, I, "default is 57600, too slow for ROS 2 topics"),
      ("MAV_1_CONFIG", 102, I, "MTF-01P flow + range sensor on TELEM2 (MAVLink)"),
      ("SER_TEL2_BAUD", 115200, I, "the MTF-01P's default baud"),
      ("MAV_2_CONFIG", 0, I, "board default MAVLink on TELEM4 off (TELEM4 is wired to the onboard Bluetooth)"),
      ("MSP_OSD_CONFIG", 0, I, "board default OSD on GPS2 off"),
      ("# Height and position sources: no GPS in phase 1. PX4's default height reference is GPS (EKF2_HGT_REF 1).",),
      ("EKF2_HGT_REF", 0, I, "barometer is the height reference; the rangefinder helps near the ground (EKF2_RNG_CTRL 1, the default)"),
      ("EKF2_GPS_CTRL", 0, I, "no GPS in phase 1. Set it back to 7 when the phase 2 GPS goes on"),
      ("# Flow is on by default (EKF2_OF_CTRL 1). Once the MTF-01 is mounted, set EKF2_OF_POS_X/Y/Z and EKF2_RNG_POS_X/Y/Z to its position.",),
      ("# Battery (board defaults assume 4S, 40 A/V and 3.2 V empty)",),
      ("BAT1_N_CELLS", cells, I, f"{cells}S pack"),
      ("BAT1_V_EMPTY", 3.5, F, "board default 3.2 V makes the % read too high under load"),
      ("BAT1_V_CHARGED", 4.2, F, "standard LiPo, no LiHV packs"),
      ("BAT1_CAPACITY", cap, F, "mAh"),
      ("BAT1_A_PER_V", 78.4, F, "START VALUE: 1000 / 12.75 mV per A. Calibrate: compare mAh put back by the charger with mAh PX4 logged"),
      ("# BAT1_V_DIV: set it in QGC Power setup by matching a multimeter reading. Not set here on purpose.",),
      ("# Gains: start at the board's own defaults, then tune on the real drone (autotune or by hand).",),
      ("# The sim gains are NOT copied here: the real motors, props and frame flex differ.",),
      ("MC_ROLLRATE_K", 0.35, F, "board default"),
      ("MC_PITCHRATE_K", 0.4, F, "board default"),
    ] + motion + failsafes + [
      ("NAV_DLL_ACT", 0, I, "Wi-Fi to QGC drops often; the RC link is the safety link"),
      ("# COM_POS_FS_EPH (default 5 m) is left alone until the Day 10 flow-loss test: 1 m may trip on normal flow drift.",),
    ]

write("sim_fiveinch_baseline.params", sim('5" (2207 1750KV, 5.1x3x3, 6S 1800)', k5, h5))
write("sim_seveninch_baseline.params", sim(f'7" (2807, 7x3.5 two-blade, {cells7}S {cap7})', k7, h7))
write("hw_seveninch_first_flight.params", hw(f'7" ({cells7}S {cap7} mAh)', cells7, cap7))
print("written to", out)
