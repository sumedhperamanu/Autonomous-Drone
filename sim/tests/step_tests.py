#!/usr/bin/env python3
"""Day 4 step tests over MAVLink (offboard): hover check, 1 m altitude step, 10 deg roll step.

Run while the sim is up:   python3 sim/tests/step_tests.py [--url udpin:0.0.0.0:14540]
Then analyse the newest log: python3 sim/tests/analyze_steps.py <log.ulg>

Why offboard: it gives the exact same, repeatable step every run, so the 5" and 7"
(or two tuning runs) compare fairly. Hand-flying a step in QGC is never the same twice.
"""
import argparse, math, time
from pymavlink import mavutil

p = argparse.ArgumentParser()
p.add_argument('--url', default='udpin:0.0.0.0:14540')
p.add_argument('--hover-thr', type=float, default=None, help='collective thrust for the roll step (measured in hover if omitted)')
p.add_argument('--pitch', action='store_true', help='also run a 10 deg pitch step after the roll step')
p.add_argument('--param', action='append', default=[], help='NAME=VALUE to set before flying, e.g. MC_ROLLRATE_K=0.3')
a = p.parse_args()

m = mavutil.mavlink_connection(a.url)
m.wait_heartbeat(); print('heartbeat from system', m.target_system)
M = mavutil.mavlink
t_boot = time.time()

def ms(): return int((time.time() - t_boot) * 1000) & 0xFFFFFFFF

def pos(z, yaw=0.0):
    m.mav.set_position_target_local_ned_send(ms(), m.target_system, m.target_component,
        M.MAV_FRAME_LOCAL_NED, 0b0000101111111000, 0, 0, z, 0, 0, 0, 0, 0, 0, yaw, 0)

def att(roll_deg, thrust, pitch_deg=0.0):
    r, pt = math.radians(roll_deg) / 2, math.radians(pitch_deg) / 2
    # roll-then-pitch quaternion (yaw 0); one of the two angles is always 0 here
    q = [math.cos(r)*math.cos(pt), math.sin(r)*math.cos(pt), math.cos(r)*math.sin(pt), -math.sin(r)*math.sin(pt)]
    m.mav.set_attitude_target_send(ms(), m.target_system, m.target_component,
        0b00000111, q, 0, 0, 0, thrust)

def hold(fn, seconds, *args):
    end = time.time() + seconds
    while time.time() < end:
        fn(*args); time.sleep(0.05)

def cmd(c, *params):
    params = list(params) + [0] * (7 - len(params))
    m.mav.command_long_send(m.target_system, m.target_component, c, 0, *params)

def get_param(name):
    m.mav.param_request_read_send(m.target_system, m.target_component, name.encode(), -1)
    msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=5)
    return msg.param_value if msg else None

def set_param(name, value):
    m.mav.param_set_send(m.target_system, m.target_component, name.encode(), float(value), M.MAV_PARAM_TYPE_REAL32)
    msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=5)
    print(f'set {name} = {value}' + ('' if msg else '  (no reply!)'))

for kv in a.param:
    n, v = kv.split('='); set_param(n, v)

# ask for the thrust setpoint stream, so the roll step can use the real hover thrust
m.mav.command_long_send(m.target_system, m.target_component, M.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                        M.MAVLINK_MSG_ID_ATTITUDE_TARGET, 20000, 0, 0, 0, 0, 0)

print('streaming setpoints, switching to offboard, arming')
hold(pos, 1.5, -2.0)
cmd(M.MAV_CMD_DO_SET_MODE, 1, 6)          # PX4 custom main mode 6 = OFFBOARD
hold(pos, 0.5, -2.0)
cmd(M.MAV_CMD_COMPONENT_ARM_DISARM, 1)
print('climb to 2 m and settle (10 s)'); hold(pos, 10, -2.0)
print('STEP: 2 m -> 3 m (8 s)');          hold(pos, 8, -3.0)
print('STEP: 3 m -> 2 m (8 s)');          hold(pos, 8, -2.0)
# Measure the thrust the position controller uses to hover, for 2 s.
# Settle first, then drop every queued message: the ATTITUDE_TARGET stream has been
# queueing since take-off, and that backlog is from the climb and descent, not hover.
# (Fixed 2026-11-03: the old code read the backlog and came out about 20% low.)
hold(pos, 2, -2.0)
while m.recv_msg() is not None:
    pass
samples, end = [], time.time() + 2
while time.time() < end:
    pos(-2.0)
    t = m.recv_match(type='ATTITUDE_TARGET', blocking=True, timeout=0.05)
    if t: samples.append(t.thrust)
thr = a.hover_thr or (sorted(samples)[len(samples) // 2] if samples else 0.5)   # median
print(f'hover thrust measured: {thr:.3f} ({len(samples)} samples)')
# Attitude mode does not hold height by itself, so this script adds a small height
# loop on top: thrust = hover thrust + gain x height error. Without it the drone
# climbs or sinks during the roll test, which also changes the roll response.
m.mav.command_long_send(m.target_system, m.target_component, M.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                        M.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 20000, 0, 0, 0, 0, 0)
state = {'z': -2.0, 'vz': 0.0}
def att_hold(roll_deg, seconds, z_ref=-2.0, pitch_deg=0.0):
    end = time.time() + seconds
    while time.time() < end:
        lp = m.recv_match(type='LOCAL_POSITION_NED', blocking=False)
        while lp:
            state['z'], state['vz'] = lp.z, lp.vz
            lp = m.recv_match(type='LOCAL_POSITION_NED', blocking=False)
        t = thr + 0.15 * (state['z'] - z_ref) + 0.10 * state['vz']
        t = max(0.05, min(0.9, t)) / (math.cos(math.radians(roll_deg)) * math.cos(math.radians(pitch_deg)))
        att(roll_deg, t, pitch_deg); time.sleep(0.02)
print('roll test: level 0.5 s, +10 deg 1.0 s, level 1.0 s')
att_hold(0.0, 0.5)
att_hold(10.0, 1.0)
att_hold(0.0, 1.0)
if a.pitch:
    # the 7" body is 35% harder to pitch than to roll (Iyy/Ixx), so pitch needs its own check
    hold(pos, 3, -2.0)
    print('pitch test: level 0.5 s, +10 deg nose-up 1.0 s, level 1.0 s')
    att_hold(0.0, 0.5); att_hold(0.0, 1.0, pitch_deg=10.0); att_hold(0.0, 1.0)
print('back to position hold (5 s), then land'); hold(pos, 5, -2.0)
cmd(M.MAV_CMD_NAV_LAND)
end = time.time() + 20
while time.time() < end:
    hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
    if hb and not (hb.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED):
        print('landed and disarmed'); break
