#!/usr/bin/env python3
"""Measure step responses from a PX4 log (.ulg) made with step_tests.py.

Prints, for the altitude steps and the roll step:
  rise time (10% -> 90% of the step), overshoot (% of step), settling time (stays within 5%),
  and the hover motor outputs. These are the numbers you compare between airframes and tunings.
Usage: python3 analyze_steps.py path/to/log.ulg [--plot out.png]
"""
import sys, argparse
import numpy as np
from pyulog import ULog

ap = argparse.ArgumentParser(); ap.add_argument('log'); ap.add_argument('--plot'); a = ap.parse_args()
u = ULog(a.log)
def topic(name, i=0):
    for d in u.data_list:
        if d.name == name and d.multi_id == i: return d.data
    return None

def step_metrics(t, y, t0, y0, y1, window):
    sel = (t >= t0) & (t <= t0 + window); t, y = t[sel] - t0, y[sel]
    s = (y - y0) / (y1 - y0)                       # 0 -> 1 normalised response
    try:
        t10 = t[np.argmax(s >= 0.1)]; t90 = t[np.argmax(s >= 0.9)]
        rise = t90 - t10 if (s >= 0.9).any() else float('nan')
    except Exception: rise = float('nan')
    over = max(0.0, (s.max() - 1) * 100)
    outside = np.where(np.abs(s - 1) > 0.05)[0]
    settle = t[outside[-1]] if len(outside) and outside[-1] < len(t) - 1 else (float('nan') if len(outside) else 0.0)
    final_err = (1 - s[-int(len(s) * 0.2):].mean()) * 100
    return rise, over, settle, final_err

lp, ts = topic('vehicle_local_position'), topic('trajectory_setpoint')
att, atts = topic('vehicle_attitude'), topic('vehicle_attitude_setpoint')
mot = topic('actuator_motors')
T = lambda d: d['timestamp'] / 1e6

print(f'log: {a.log}')
# ---- altitude steps: find setpoint jumps of ~1 m
tz, zsp = T(ts), ts['position[2]']
# only the airborne 2 m <-> 3 m steps (skip takeoff and landing)
jumps = [i for i in range(1, len(zsp)) if np.isfinite(zsp[i]) and np.isfinite(zsp[i-1])
         and abs(zsp[i] - zsp[i-1]) > 0.5 and min(-zsp[i], -zsp[i-1]) > 1.5]
for i in jumps:
    t0, z0, z1 = tz[i], zsp[i-1], zsp[i]
    r, o, s, e = step_metrics(T(lp), lp['z'], t0, z0, z1, 7.5)
    print(f'ALT step {-z0:.1f} m -> {-z1:.1f} m: rise {r:.2f} s, overshoot {o:.0f}%, settle(5%) {s:.2f} s, final error {e:.1f}%')
# ---- hover motor outputs (the 3 s before the first altitude step)
if jumps and mot is not None:
    t0 = tz[jumps[0]]; sel = (T(mot) > t0 - 3) & (T(mot) < t0)
    outs = [mot[f'control[{k}]'][sel].mean() for k in range(4)]
    print('hover motor outputs: ' + ', '.join(f'{x:.3f}' for x in outs) + f'  (mean {np.mean(outs):.3f})')
    av = topic('vehicle_angular_velocity')
    if av is not None:
        sel = (T(av) > t0 - 3) & (T(av) < t0)
        print(f'hover roll-rate wobble: {np.degrees(np.std(av["xyz[0]"][sel])):.2f} deg/s RMS (shaking shows up here first)')
# ---- roll step: attitude setpoint roll jumps of ~10 deg
def roll_of(q0, q1, q2, q3): return np.degrees(np.arctan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2)))
rs = roll_of(*(atts[f'q_d[{k}]'] for k in range(4))); tr = T(atts)
ra = roll_of(*(att[f'q[{k}]'] for k in range(4)))
up = [i for i in range(1, len(rs)) if rs[i] > 5 and rs[i-1] <= 5]
for i in up[:1]:
    t0 = tr[i - 1] if rs[i] > 9 else tr[i]
    r, o, s, e = step_metrics(T(att), ra, t0, 0.0, 10.0, 0.95)
    ta = T(att); sel = (ta > t0) & (ta < t0 + 0.95)
    lag = ta[sel][np.argmax(ra[sel] > 1.0)] - t0
    print(f'ROLL step 0 -> 10 deg: delay {lag*1000:.0f} ms, rise {r:.3f} s, overshoot {o:.0f}%, '
          f'settle(5%) {s:.3f} s, error at 0.8-0.95 s {e:.0f}%')
# ---- inner (rate) loop: how well actual roll rate follows the rate setpoint during the roll step
rsp, av = topic('vehicle_rates_setpoint'), topic('vehicle_angular_velocity')
if up and rsp is not None and av is not None:
    t0 = tr[up[0]]; grid = np.arange(t0 - 0.2, t0 + 1.0, 0.002)
    s = np.interp(grid, T(rsp), rsp['roll']); y = np.interp(grid, T(av), av['xyz[0]'])
    lags = np.arange(0, 0.1, 0.002)
    err = [np.mean((s[:len(s)-int(l/0.002) or None] - y[int(l/0.002):])**2) for l in lags]
    best = lags[int(np.argmin(err))]
    print(f'rate tracking (inner loop): lag {best*1000:.0f} ms, peak actual/setpoint {y.max()/s.max():.2f}')
# ---- pitch step (only when step_tests.py ran with --pitch)
def pitch_of(q0, q1, q2, q3): return np.degrees(np.arcsin(np.clip(2*(q0*q2 - q3*q1), -1, 1)))
ps = pitch_of(*(atts[f'q_d[{k}]'] for k in range(4))); pa = pitch_of(*(att[f'q[{k}]'] for k in range(4)))
pup = [i for i in range(1, len(ps)) if ps[i] > 8.5 and ps[i-1] < 2]   # a jump to ~10 deg, not the tilt of position hold
for i in pup[:1]:
    t0 = tr[i - 1] if ps[i] > 9 else tr[i]
    r, o, s_, e = step_metrics(T(att), pa, t0, 0.0, 10.0, 0.95)
    ta = T(att); sel = (ta > t0) & (ta < t0 + 0.95)
    lag = ta[sel][np.argmax(pa[sel] > 1.0)] - t0
    print(f'PITCH step 0 -> 10 deg: delay {lag*1000:.0f} ms, rise {r:.3f} s, overshoot {o:.0f}%, '
          f'settle(5%) {s_:.3f} s, error at 0.8-0.95 s {e:.0f}%')
    if rsp is not None and av is not None:
        t0 = tr[i]; grid = np.arange(t0 - 0.2, t0 + 1.0, 0.002)
        sp = np.interp(grid, T(rsp), rsp['pitch']); y = np.interp(grid, T(av), av['xyz[1]'])
        lags = np.arange(0, 0.1, 0.002)
        err = [np.mean((sp[:len(sp)-int(l/0.002) or None] - y[int(l/0.002):])**2) for l in lags]
        print(f'pitch rate tracking: lag {lags[int(np.argmin(err))]*1000:.0f} ms, peak actual/setpoint {y.max()/sp.max():.2f}')
if a.plot:
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    n = 3 if pup else 2
    fig, ax = plt.subplots(n, 1, figsize=(9, 2.6 * n + 0.8))
    ax[0].plot(T(lp), -lp['z'], label='altitude'); ax[0].plot(tz, -zsp, '--', label='setpoint'); ax[0].set_ylabel('altitude (m)'); ax[0].legend()
    ax[1].plot(T(att), ra, label='roll'); ax[1].plot(tr, rs, '--', label='setpoint'); ax[1].set_ylabel('roll (deg)'); ax[1].legend()
    if up: ax[1].set_xlim(tr[up[0]] - 0.5, tr[up[0]] + 2.0); ax[1].set_ylim(-3, 14)
    if pup:
        ax[2].plot(T(att), pa, label='pitch'); ax[2].plot(tr, ps, '--', label='setpoint'); ax[2].set_ylabel('pitch (deg)'); ax[2].legend()
        ax[2].set_xlim(tr[pup[0]] - 0.5, tr[pup[0]] + 2.0); ax[2].set_ylim(-3, 14)
    ax[-1].set_xlabel('time (s)')
    fig.tight_layout(); fig.savefig(a.plot); print('plot:', a.plot)
