#!/usr/bin/env python3
"""One line per log: airframe, rate gain K, and the step-test numbers. Usage: summarize_logs.py log1.ulg log2.ulg ..."""
import sys, subprocess, re
from pyulog import ULog
here = __file__.rsplit('/', 1)[0]
for path in sys.argv[1:]:
    u = ULog(path)
    p = dict(u.initial_parameters)
    for t, n, v in u.changed_parameters: p[n] = v
    af, k = int(p.get('SYS_AUTOSTART', 0)), p.get('MC_ROLLRATE_K')
    out = subprocess.run([sys.executable, f'{here}/analyze_steps.py', path], capture_output=True, text=True).stdout
    g = lambda pat: (re.search(pat, out) or [None, '-'])[1]
    ks = f"{k:.2f}" if k is not None else "-"   # older logs may not have MC_ROLLRATE_K
    print(f"{af}\tK={ks}\thover={g(r'mean ([0-9.]+)')}\twobble={g(r'wobble: ([0-9.]+)')}\t"
          f"alt_rise={g(r'ALT step 2.0 m -> 3.0 m: rise ([0-9.]+)')}\troll_delay={g(r'delay ([0-9]+) ms')}\t"
          f"roll_rise={g(r'ROLL.*rise ([0-9.na]+) s')}\tovershoot={g(r'ROLL.*overshoot ([0-9]+)%')}\t"
          f"end_err={g(r'error at (?:end|0.8-0.95 s) (-?[0-9]+)%')}\trate_lag={g(r'lag ([0-9]+) ms')}\tpeak_ratio={g(r'setpoint ([0-9.]+)')}\tpitch_rise={g(r'PITCH.*rise ([0-9.na]+) s')}\tpitch_lag={g(r'pitch rate tracking: lag ([0-9]+) ms')}\t{path.split('/')[-1]}")
