#!/usr/bin/env python3
"""Perception load benchmark for the 5" drone's Raspberry Pi 5.

Times the per-frame work the drone's perception has to do:
  depth  - OAK-D depth -> 3D points -> 10 cm voxels -> obstacle clusters -> tracked velocity ->
           time to collision -> pick a dodge direction
  scan   - STL-27L scan -> 2D clusters -> tracked velocity -> time to collision -> dodge direction
  vo     - a stand-in for the SLAM front end on the mono image: GFTT corners + ORB descriptors (RTAB-Map's
           default features), matching to the last frame and a PnP RANSAC pose using depth

It doesn't run SLAM itself; RTAB-Map's cost on a Pi 5 comes from published measurements (see README.md).

Two ways to run it:
  --replay sim_clip.npz   loops a recorded clip, no ROS or sensors needed: run this on a bare Pi 5
  (no --replay)           subscribes to the live topics, from the sim or from the real drivers

On a PC, --scale projects the times to a Pi 5. On the Pi itself, leave it at 1.
"""
import argparse
import json
import math
import time

import cv2
import numpy as np
from scipy import ndimage

VOXEL = 0.10                                             # m
BOX = np.array([[0.2, 8.0], [-4.0, 4.0], [-2.0, 2.0]])   # forward, left, up from the camera, m
DRONE_RADIUS = 0.17                                      # m
HFOV = 1.274                                             # rad, OAK-D Lite mono and depth
FULL_RATE_HZ = {'depth': 30.0, 'scan': 10.0, 'vo': 30.0}
# Pass marks on the Pi 5. Depth and the SLAM front end must finish inside one 30 Hz frame on one core, and the
# lidar decision inside P-6's 50 ms.
LIMIT_P95_MS = {'depth': 33.3, 'scan': 50.0, 'vo': 33.3}


class Tracker:
    """Matches each cluster to the nearest one from the last frame and estimates its velocity."""

    def __init__(self, gate):
        self.gate, self.prev, self.t = gate, None, None

    def update(self, t, cents):
        vel = np.zeros_like(cents)
        if self.prev is not None and len(cents) and len(self.prev) and t > self.t:
            d = np.linalg.norm(cents[:, None, :] - self.prev[None, :, :], axis=2)
            j = d.argmin(1)
            ok = d[np.arange(len(cents)), j] < self.gate
            vel[ok] = (cents[ok] - self.prev[j[ok]]) / (t - self.t)
        self.prev, self.t = cents, t
        return vel


def decide(cents, vel, sizes, dims):
    """Time to collision of the most urgent object, and the 1 m escape move that keeps it furthest away."""
    if not len(cents):
        return math.inf, None
    r = np.linalg.norm(cents, axis=1)
    closing = -(cents * vel).sum(1) / np.maximum(r, 1e-6)
    ttc = np.where(closing > 0.05, (r - DRONE_RADIUS - sizes / 2) / np.maximum(closing, 1e-6), np.inf)
    k = int(np.argmin(ttc))
    if not np.isfinite(ttc[k]):
        return math.inf, None
    moves = np.concatenate([np.eye(dims), -np.eye(dims)])
    future = cents + vel * 0.5
    gaps = [np.min(np.linalg.norm(future - m, axis=1) - sizes / 2) for m in moves]
    return float(ttc[k]), int(np.argmax(gaps))


class Pipelines:
    def __init__(self, decimate, vo_half, fx=None):
        self.decimate, self.vo_half, self.fx = decimate, vo_half, fx
        self.rays = None
        self.last_depth = None
        self.depth_tracker, self.scan_tracker = Tracker(0.6), Tracker(0.6)
        self.dims = np.ceil((BOX[:, 1] - BOX[:, 0]) / VOXEL).astype(int)
        self.gftt = cv2.GFTTDetector_create(maxCorners=1000, qualityLevel=0.01, minDistance=7)
        self.orb = cv2.ORB_create(nfeatures=1000)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self.prev_vo = None
        self.times = {'depth': [], 'scan': [], 'vo': []}
        self.stamps = {'depth': [], 'scan': [], 'vo': []}

    def record(self, name, t, t0):
        self.times[name].append((time.perf_counter() - t0) * 1000)
        self.stamps[name].append(t)

    def depth(self, raw, t):
        """raw: depth image, uint16 mm (real OAK-D) or float32 m (sim)."""
        t0 = time.perf_counter()
        s = self.decimate
        d = raw[::s, ::s]
        d = d.astype(np.float32) * (0.001 if raw.dtype == np.uint16 else 1.0)
        h, w = d.shape
        if self.rays is None or self.rays[0].shape != d.shape:
            fx = (self.fx or raw.shape[1] / (2 * math.tan(HFOV / 2))) / s
            u, v = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
            self.rays = ((u - w / 2) / fx, (v - h / 2) / fx)
        ok = (d > BOX[0, 0]) & (d < BOX[0, 1])
        z = d[ok]
        pts = np.stack([z, -self.rays[0][ok] * z, -self.rays[1][ok] * z], axis=1)
        self.last_depth = d
        idx = ((pts - BOX[:, 0]) / VOXEL).astype(np.int32)
        idx = idx[np.all((idx >= 0) & (idx < self.dims), axis=1)]
        grid = np.zeros(self.dims, dtype=bool)
        grid[idx[:, 0], idx[:, 1], idx[:, 2]] = True
        lab, n = ndimage.label(grid, structure=np.ones((3, 3, 3)))
        if n:
            cells = lab[idx[:, 0], idx[:, 1], idx[:, 2]]          # each point's cluster
            occ = np.flatnonzero(grid)
            cl = lab.ravel()[occ] - 1
            pos = (np.stack(np.unravel_index(occ, self.dims), axis=1) + 0.5) * VOXEL + BOX[:, 0]
            count = np.bincount(cl, minlength=n).astype(np.float32)
            cents = np.stack([np.bincount(cl, pos[:, i], n) for i in range(3)], axis=1) / count[:, None]
            sizes = np.cbrt(count) * VOXEL
            big = count >= 2                                      # drop single noisy voxels
            cents, sizes = cents[big], sizes[big]
            del cells
        else:
            cents, sizes = np.zeros((0, 3)), np.zeros(0)
        vel = self.depth_tracker.update(t, cents)
        decide(cents, vel, sizes, 3)
        self.record('depth', t, t0)

    def scan(self, r, t, angle_min, angle_inc, range_min, range_max):
        t0 = time.perf_counter()
        ang = angle_min + np.arange(len(r), dtype=np.float32) * angle_inc
        ok = np.isfinite(r) & (r > range_min) & (r < min(range_max, 12.0))
        pts = np.stack([r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok])], axis=1)
        groups = []
        if len(pts):
            breaks = np.flatnonzero(np.linalg.norm(np.diff(pts, axis=0), axis=1) > 0.2) + 1
            groups = [g for g in np.split(pts, breaks) if len(g) >= 3]
        cents = np.array([g.mean(0) for g in groups]) if groups else np.zeros((0, 2))
        sizes = np.array([np.linalg.norm(g[-1] - g[0]) for g in groups]) if groups else np.zeros(0)
        vel = self.scan_tracker.update(t, cents)
        decide(cents, vel, sizes, 2)
        self.record('scan', t, t0)

    def vo(self, img, t):
        t0 = time.perf_counter()
        if self.vo_half:
            img = cv2.resize(img, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
        kps = self.gftt.detect(img)
        kps, des = self.orb.compute(img, kps)
        h, w = img.shape
        f = w / (2 * math.tan(HFOV / 2))
        if self.prev_vo is not None and des is not None and self.prev_vo[1] is not None and self.last_depth is not None:
            pk, pd = self.prev_vo
            matches = self.matcher.match(pd, des)
            d = self.last_depth
            sc = d.shape[1] / w
            if matches:
                uv = np.array([pk[m.queryIdx].pt for m in matches], dtype=np.float32)
                cur = np.array([kps[m.trainIdx].pt for m in matches], dtype=np.float32)
                du = np.clip((uv[:, 0] * sc).astype(int), 0, d.shape[1] - 1)
                dv = np.clip((uv[:, 1] * sc).astype(int), 0, d.shape[0] - 1)
                z = d[dv, du]
                ok = (z > 0.2) & (z < 8)
                if ok.sum() >= 20:
                    obj = np.stack([(uv[ok, 0] - w / 2) * z[ok] / f, (uv[ok, 1] - h / 2) * z[ok] / f, z[ok]], 1)
                    k = np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1]])
                    cv2.solvePnPRansac(obj.astype(np.float32), cur[ok], k, None, iterationsCount=100,
                                       reprojectionError=3.0)
        self.prev_vo = (kps, des)
        self.record('vo', t, t0)


def summary(p, wall, cpu, scale):
    out = {'wall_s': round(wall, 1), 'scale_to_pi5': scale, 'process_cpu_cores_here': round(cpu / wall, 3),
           'pipelines': {}}
    load = 0.0
    for name, t in p.times.items():
        if len(t) < 5:
            continue
        t = np.array(t[3:])                                   # skip warm-up frames
        st = p.stamps[name]
        pi = t * scale
        cores = pi.mean() * FULL_RATE_HZ[name] / 1000
        load += cores
        out['pipelines'][name] = {
            'frames': int(len(t)),
            'here_ms': {'mean': round(float(t.mean()), 2), 'p95': round(float(np.percentile(t, 95)), 2),
                        'max': round(float(t.max()), 2)},
            'pi5_ms': {'mean': round(float(pi.mean()), 2), 'p95': round(float(np.percentile(pi, 95)), 2),
                       'max': round(float(pi.max()), 2)},
            'pi5_cores_at_full_rate': round(cores, 3), 'full_rate_hz': FULL_RATE_HZ[name],
            'limit_p95_ms': LIMIT_P95_MS[name], 'pass': bool(np.percentile(pi, 95) <= LIMIT_P95_MS[name]),
        }
        if len(st) > 1 and st[-1] > st[0]:
            out['pipelines'][name]['input_rate_hz'] = round((len(st) - 1) / (st[-1] - st[0]), 1)
    out['pi5_cores_total'] = round(load, 3)
    out['pass'] = all(v['pass'] for v in out['pipelines'].values())
    return out


def replay(a, p):
    c = np.load(a.replay)
    p.fx = float(c['fx']) if 'fx' in c else None
    depth, mono, scans = c['depth'], c['mono'], c['scan']
    sp = [float(c[k]) for k in ('angle_min', 'angle_increment', 'range_min', 'range_max')]
    start = time.time()
    end = start + a.duration
    i, t, late = 0, 0.0, 0
    while time.time() < end:
        p.depth(depth[i % len(depth)], t)
        if not a.no_vo:
            p.vo(mono[i % len(mono)], t)
        if i % 3 == 0:
            p.scan(scans[(i // 3) % len(scans)], t, *sp)
        i += 1
        t += 1 / 30
        wait = start + i / 30 - time.time()       # paced like a 30 Hz camera, unless --flat-out
        if wait > 0 and not a.flat_out:
            time.sleep(wait)
        elif wait < 0:
            late += 1
    p.late_frames = (late, i)


def live(a, p):
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image, LaserScan

    def stamp(m):
        return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    def depth_cb(m):
        dt = np.uint16 if m.encoding in ('16UC1', 'mono16') else np.float32
        p.depth(np.frombuffer(m.data, dtype=dt).reshape(m.height, m.width), stamp(m))

    def mono_cb(m):
        ch = 1 if m.encoding in ('mono8', '8UC1') else 3
        img = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, ch)
        p.vo(img[:, :, 0] if ch == 1 else cv2.cvtColor(img, cv2.COLOR_RGB2GRAY), stamp(m))

    def info_cb(m):
        p.fx = m.k[0] or None

    rclpy.init()
    n = rclpy.create_node('p3_perception_load')
    q = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
    n.create_subscription(CameraInfo, a.info, info_cb, q)
    n.create_subscription(Image, a.depth, depth_cb, q)
    n.create_subscription(LaserScan, a.scan, lambda m: p.scan(np.asarray(m.ranges, np.float32), stamp(m), m.angle_min,
                                                              m.angle_increment, m.range_min, m.range_max), q)
    if not a.no_vo:
        n.create_subscription(Image, a.mono, mono_cb, q)
    end = time.time() + a.duration
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
    n.destroy_node()
    rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--replay', default='', help='loop a recorded clip (.npz) instead of live topics')
    ap.add_argument('--duration', type=float, default=60, help='seconds of wall time to measure')
    ap.add_argument('--flat-out', action='store_true', help='replay as fast as possible instead of at 30 Hz')
    ap.add_argument('--scale', type=float, default=1.0, help='multiply times by this to project to a Pi 5 (1 on the Pi)')
    ap.add_argument('--decimate', type=int, default=2, help='use every nth depth pixel (2 = 320 x 240)')
    ap.add_argument('--vo-half', action='store_true', help='run the SLAM front-end stand-in at 320 x 240')
    ap.add_argument('--no-vo', action='store_true', help='skip the SLAM front-end stand-in')
    ap.add_argument('--depth', default='/depth_camera/image')
    ap.add_argument('--info', default='/depth_camera/camera_info')
    ap.add_argument('--mono', default='/oakd/left/image')
    ap.add_argument('--scan', default='/scan')
    ap.add_argument('--report', default='')
    a = ap.parse_args()
    p = Pipelines(a.decimate, a.vo_half)
    w0, c0 = time.time(), time.process_time()
    (replay if a.replay else live)(a, p)
    res = summary(p, time.time() - w0, time.process_time() - c0, a.scale)
    if hasattr(p, 'late_frames'):
        late, total = p.late_frames
        res['replay_kept_up'] = f'{total - late} of {total} frames on time at 30 Hz'
        if a.scale == 1 and not a.flat_out and late > 0.02 * total:
            res['pass'] = False
    res['settings'] = {'mode': 'replay' if a.replay else 'live',
                       'depth_px': f'{640 // a.decimate} x {480 // a.decimate}',
                       'vo_px': 'off' if a.no_vo else ('320 x 240' if a.vo_half else '640 x 480')}
    print(json.dumps(res, indent=1))
    if a.report:
        with open(a.report, 'w') as f:
            json.dump(res, f, indent=1)


if __name__ == '__main__':
    main()
