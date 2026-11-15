"""Record a short clip of depth, mono and scan messages into one .npz file for perception_load.py --replay.

  python3 record_clip.py my_clip.npz 40     (topic names below; change them for the real drivers)
"""
import sys
import time

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, LaserScan

N = int(sys.argv[2]) if len(sys.argv) > 2 else 40
out = sys.argv[1]
q = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
data = {'depth': [], 'depth_t': [], 'mono': [], 'mono_t': [], 'scan': [], 'scan_t': []}
meta = {}


def st(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def depth(m):
    if len(data['depth']) < N:
        d = np.frombuffer(m.data, dtype=np.float32).reshape(m.height, m.width)
        data['depth'].append(np.where(np.isfinite(d), np.clip(d * 1000, 0, 65535), 0).astype(np.uint16))
        data['depth_t'].append(st(m))


def mono(m):
    if len(data['mono']) < N:
        data['mono'].append(np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width).copy())
        data['mono_t'].append(st(m))


def scan(m):
    if len(data['scan']) < max(4, N // 3):
        data['scan'].append(np.asarray(m.ranges, dtype=np.float32))
        data['scan_t'].append(st(m))
        meta.update(angle_min=m.angle_min, angle_increment=m.angle_increment, range_min=m.range_min,
                    range_max=m.range_max)


def info(m):
    meta['fx'] = m.k[0]


rclpy.init()
n = rclpy.create_node('p3_capture')
n.create_subscription(Image, '/depth_camera/image', depth, q)
n.create_subscription(Image, '/oakd/left/image', mono, q)
n.create_subscription(LaserScan, '/scan', scan, q)
n.create_subscription(CameraInfo, '/depth_camera/camera_info', info, q)
end = time.time() + 120
while time.time() < end and (len(data['depth']) < N or len(data['mono']) < N or len(data['scan']) < max(4, N // 3)):
    rclpy.spin_once(n, timeout_sec=0.05)
np.savez_compressed(out, depth=np.array(data['depth']), depth_t=np.array(data['depth_t']), mono=np.array(data['mono']),
                    mono_t=np.array(data['mono_t']), scan=np.array(data['scan']), scan_t=np.array(data['scan_t']),
                    **{k: np.array(v) for k, v in meta.items()})
print('saved', out, {k: len(v) for k, v in data.items()}, meta)
rclpy.shutdown()
