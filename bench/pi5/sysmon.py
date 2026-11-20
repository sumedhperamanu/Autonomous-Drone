#!/usr/bin/env python3
"""Watch the Raspberry Pi 5 while the drone's software runs: CPU per process group, temperature, throttling.

  python3 sysmon.py --duration 120 --report sysmon.json

Groups processes by name so the result reads as a budget: SLAM, OAK-D driver, lidar driver, XRCE agent,
perception, everything else. Reads /proc and /sys only, so it needs no extra packages.
Pass: at least 0.75 of a core spare on average, CPU never above 85 C, and no throttling or under-voltage.
"""
import argparse
import json
import os
import subprocess
import time

GROUPS = [('slam', ('rtabmap', 'rgbd_odometry', 'orb_slam')), ('oakd_driver', ('depthai', 'oak_container', 'camera_node')),
          ('lidar_driver', ('ldlidar', 'stl27l', 'stl_27l', 'lidar_node')), ('xrce_agent', ('microxrceagent',)),
          ('perception', ('perception_load', 'p3_perception', 'obstacle')), ('ros_other', ('ros2', 'python3', '_ros2_daemon')),
          ('gazebo_px4', ('gz sim', 'gz-sim', 'ruby', 'px4'))]
HZ = os.sysconf('SC_CLK_TCK')


def cpu_total():
    with open('/proc/stat') as f:
        v = [int(x) for x in f.readline().split()[1:]]
    return sum(v), v[3] + v[4]          # all, idle + iowait


def procs():
    out = {}
    for pid in os.listdir('/proc'):
        if not pid.isdigit():
            continue
        try:
            with open(f'/proc/{pid}/stat') as f:
                s = f.read()
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                cmd = f.read().replace(b'\0', b' ').decode(errors='ignore').lower()
            fields = s[s.rindex(')') + 2:].split()
            out[pid] = (int(fields[11]) + int(fields[12]), cmd or s[s.index('(') + 1:s.rindex(')')].lower())
        except (OSError, ValueError):
            continue
    return out


def group(cmd):
    for name, keys in GROUPS:
        if any(k in cmd for k in keys):
            return name
    return 'other'


def temperature():
    try:
        with open('/sys/class/thermal/thermal_zone0/temp') as f:
            return int(f.read()) / 1000
    except OSError:
        return None


def throttled():
    try:
        return int(subprocess.run(['vcgencmd', 'get_throttled'], capture_output=True, text=True, timeout=2)
                   .stdout.strip().split('=')[1], 16)
    except (OSError, IndexError, ValueError, subprocess.SubprocessError):
        return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--duration', type=float, default=120)
    ap.add_argument('--report', default='')
    a = ap.parse_args()
    ncpu = os.cpu_count()
    t_all, t_idle = cpu_total()
    p0 = procs()
    used, temps, flags, per_group = [], [], 0, {}
    end = time.time() + a.duration
    w = time.time()
    while time.time() < end:
        time.sleep(1.0)
        all1, idle1 = cpu_total()
        p1 = procs()
        dt = time.time() - w
        w = time.time()
        busy = 1 - (idle1 - t_idle) / max(1, all1 - t_all)
        used.append(busy * ncpu)
        t_all, t_idle = all1, idle1
        sample = {}
        for pid, (ticks, cmd) in p1.items():
            if pid in p0:
                g = group(cmd)
                sample[g] = sample.get(g, 0) + (ticks - p0[pid][0]) / HZ / dt
        for g, v in sample.items():
            per_group.setdefault(g, []).append(v)
        p0 = p1
        t = temperature()
        if t is not None:
            temps.append(t)
        f = throttled()
        if f:
            flags |= f
    n = max(1, len(used))
    res = {'cores': ncpu, 'seconds': len(used), 'cores_used_mean': round(sum(used) / n, 2),
           'cores_used_max': round(max(used or [0]), 2),
           'groups_cores_mean': {g: round(sum(v) / n, 2) for g, v in sorted(per_group.items(), key=lambda x: -sum(x[1]))},
           'temp_c_max': max(temps) if temps else None, 'throttled_flags': hex(flags)}
    spare = ncpu - res['cores_used_mean']
    res['pass'] = bool(spare >= 0.75 and (res['temp_c_max'] or 0) < 85 and flags == 0)
    res['notes'] = ('throttled_flags: 0x1 under-voltage now, 0x4 throttled now, 0x10000 under-voltage has happened, '
                    '0x40000 throttling has happened. Anything but 0x0 fails.')
    print(json.dumps(res, indent=1))
    if a.report:
        with open(a.report, 'w') as f:
            json.dump(res, f, indent=1)


if __name__ == '__main__':
    main()
