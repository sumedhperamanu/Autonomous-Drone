#!/usr/bin/env python3
"""Measure step responses from a PX4 log (.ulg) made with step_tests.py.

Prints, for the altitude steps and the roll step:
  rise time (10% -> 90% of the step), overshoot (% of step), settling time (stays within 5%),
  and the hover motor outputs. These are the numbers you compare between airframes and tunings.
Usage: python3 analyze_steps.py path/to/log.ulg [--plot out.png]
       add --calm other.ulg to draw a no-wind run in grey under this one (for wind runs),
       --label 'typical wind' to name this run in the legend, --title '...' for the plot title.
"""
import sys, argparse
import numpy as np
from pyulog import ULog

ap = argparse.ArgumentParser(); ap.add_argument('log'); ap.add_argument('--plot')
ap.add_argument('--calm', help='a no-wind log to draw underneath, in grey')
ap.add_argument('--label', default='this run'); ap.add_argument('--title')
a = ap.parse_args()
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
def step_starts(sp, t):
    # The script's step jumps from level (under 2 deg) to exactly 10 deg and stays there for 1 s. In wind,
    # position hold leans the drone too, and switching back to it after a step can jump the setpoint past
    # 10 deg, so check that it really sits at 10 for the next half second
    return [i for i in range(1, len(sp)) if sp[i-1] < 2 and 9.5 <= sp[i] <= 10.5
            and np.all(np.abs(sp[(t >= t[i]) & (t <= t[i] + 0.5)] - 10) <= 0.5)]
up = step_starts(rs, tr)
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
pup = step_starts(ps, tr)   # a jump to 10 deg, not the tilt of position hold
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

    def series(path):
        """What the plot needs from one log. Times are measured from each step, so two runs line up."""
        uu = ULog(path)
        def tp(name):
            for d in uu.data_list:
                if d.name == name and d.multi_id == 0: return d.data
            return None
        l, sp, q, qs = tp('vehicle_local_position'), tp('trajectory_setpoint'), tp('vehicle_attitude'), tp('vehicle_attitude_setpoint')
        z = sp['position[2]']
        j = [i for i in range(1, len(z)) if np.isfinite(z[i]) and np.isfinite(z[i-1])
             and abs(z[i] - z[i-1]) > 0.5 and min(-z[i], -z[i-1]) > 1.5]
        r_sp = roll_of(*(qs[f'q_d[{k}]'] for k in range(4))); p_sp = pitch_of(*(qs[f'q_d[{k}]'] for k in range(4)))
        ru = step_starts(r_sp, T(qs))
        pu = step_starts(p_sp, T(qs))
        return dict(t_lp=T(l), alt=-l['z'], drift=np.hypot(l['x'], l['y']), t_sp=T(sp), alt_sp=-z,
                    t_att=T(q), roll=roll_of(*(q[f'q[{k}]'] for k in range(4))),
                    pitch=pitch_of(*(q[f'q[{k}]'] for k in range(4))),
                    t_atts=T(qs), roll_sp=r_sp, pitch_sp=p_sp,
                    t_alt=T(sp)[j[0]] if j else T(l)[0],
                    t_roll=T(qs)[ru[0]] if ru else None, t_pitch=T(qs)[pu[0]] if pu else None)

    main = series(a.log)
    runs = ([(series(a.calm), 'no wind', '0.6', 1.2)] if a.calm else []) + [(main, a.label, 'C0', 1.6)]
    panels = ['alt', 'roll'] + (['pitch'] if main['t_pitch'] is not None else []) + ['drift']
    fig, ax = plt.subplots(len(panels), 1, figsize=(9, 2.5 * len(panels) + 1))
    A = dict(zip(panels, ax))
    for s, lab, c, w in runs:
        A['alt'].plot(s['t_lp'] - s['t_alt'], s['alt'], color=c, lw=w, label=lab)
        A['drift'].plot(s['t_lp'] - s['t_alt'], s['drift'], color=c, lw=w, label=lab)
        if s['t_roll'] is not None:
            A['roll'].plot(s['t_att'] - s['t_roll'], s['roll'], color=c, lw=w, label=lab)
        if 'pitch' in A and s['t_pitch'] is not None:
            A['pitch'].plot(s['t_att'] - s['t_pitch'], s['pitch'], color=c, lw=w, label=lab)
    def angle_limits(key, t0key):
        # At least -6..15 deg, wider if a run leans further (in strong wind it starts tilted into the wind)
        lo, hi = -6, 15
        for s, *_ in runs:
            if s[t0key] is None: continue
            m = (s['t_att'] - s[t0key] >= -0.5) & (s['t_att'] - s[t0key] <= 2.0)
            if m.any(): lo, hi = min(lo, s[key][m].min() - 1), max(hi, s[key][m].max() + 1)
        return lo, hi
    A['alt'].plot(main['t_sp'] - main['t_alt'], main['alt_sp'], 'k--', lw=1, label='setpoint')
    A['alt'].set_ylim(bottom=-0.3); A['alt'].set_ylabel('altitude (m)'); A['alt'].set_xlabel('time from the first height step (s)')
    if main['t_roll'] is not None:
        A['roll'].plot(main['t_atts'] - main['t_roll'], main['roll_sp'], 'k--', lw=1, label='setpoint')
        A['roll'].set_xlim(-0.5, 2.0); A['roll'].set_ylim(*angle_limits('roll', 't_roll'))
    A['roll'].set_ylabel('roll (deg)'); A['roll'].set_xlabel('time from the roll step (s)')
    if 'pitch' in A:
        A['pitch'].plot(main['t_atts'] - main['t_pitch'], main['pitch_sp'], 'k--', lw=1, label='setpoint')
        A['pitch'].set_xlim(-0.5, 2.0); A['pitch'].set_ylim(*angle_limits('pitch', 't_pitch'))
        A['pitch'].set_ylabel('pitch (deg)'); A['pitch'].set_xlabel('time from the pitch step (s)')
    A['drift'].set_ylabel('off its spot (m)'); A['drift'].set_xlabel('time from the first height step (s)')
    A['drift'].set_title('Distance from the hold point. The roll and pitch steps move it sideways on purpose; '
                         'watch the hover parts', fontsize=9, loc='left')
    for x in ax: x.legend(loc='upper left', fontsize=8); x.grid(alpha=0.3)
    if a.title: fig.suptitle(a.title)
    fig.tight_layout(); fig.savefig(a.plot, dpi=110); print('plot:', a.plot)
