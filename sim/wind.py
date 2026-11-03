#!/usr/bin/env python3
"""Gusty wind in a running sim (5" or 7", any world). Numbers are for Waterloo.

Run it in a second terminal while the sim is up:
    python3 ~/wind.py typical        5 m/s, gusts to about 8 (an average day in Waterloo)
    python3 ~/wind.py windy          9 m/s, gusts to about 15 (a windy fall or winter day)
    python3 ~/wind.py storm          13 m/s, gusts to about 20 (only to find the limit)
    python3 ~/wind.py calm           2 m/s, gusts to about 4
    python3 ~/wind.py 7 12           your own: 7 m/s, gusts to about 12
    python3 ~/wind.py off            stop the wind (if the script got killed and left it on)
Options:
    --from 250    where the wind comes from, in compass degrees (0 north, 90 east, 270 west).
                  Default 250 (WSW), Waterloo's most common direction.
    --steady      steady wind, no gusts
Ctrl+C stops the wind.

How it works: it sets Gazebo's world wind and turns on Gazebo's wind system, then changes the wind a couple of
times a second to make gusts. The wind pushes the drone through
the motor model's rotor drag (rotorDragCoefficient in models/seveninch and models/fiveinch), which uses the air
speed relative to the drone. So the push drops when the drone drifts with the wind, and grows when the motors spin
faster. The coefficient covers two things:
  - the props getting pushed sideways by air moving across them (the x500's value, scaled down for the 5")
  - drag on the frame, battery and electronics: 0.04 m^2 of drag area on the 7", 0.025 m^2 on the 5"
Gazebo's drag is linear in speed, and real body drag grows with speed squared. The coefficient is matched at
10 m/s, so the sim is a bit too harsh below that and a bit too gentle in the biggest gusts.
The gusts are random turbulence: each direction drifts with a time constant of about 25 m / wind speed, set so the
biggest gust in a minute is about the gust number above. Wind speeds are weather-station numbers (10 m up). At our
3 m it's usually a bit less, so this errs on the harsh side.
"""
import argparse
import math
import random
import re
import subprocess
import sys
import time

PRESETS = {  # mean wind and typical one-minute gust peak, m/s
    "calm": (2.0, 4.0),
    "typical": (5.0, 8.0),
    "windy": (9.0, 15.0),
    "storm": (13.0, 20.0),
}
MASS = {"seveninch": 1.32, "fiveinch": 0.84}
BODY_K = 0.0      # body drag is lumped into rotorDragCoefficient instead (see the docstring)
LENGTH = 25.0     # turbulence length scale near the ground, m
DT = 0.5          # target time between wind updates, s


def gz(*args, timeout=10):
    try:
        return subprocess.run(["gz", *args], capture_output=True, text=True, timeout=timeout).stdout
    except subprocess.TimeoutExpired:
        return ""


def find_world():
    m = re.search(r"^/world/([^/\s]+)/clock$", gz("topic", "-l"), re.M)
    if not m:
        sys.exit("No running sim found. Start the sim first, then run this in a second terminal.")
    return m.group(1)


def find_drone():
    models = re.findall(r"^\s*-\s*(\S+)", gz("model", "--list", timeout=20), re.M)
    return next((n for n in models if any(k in n for k in MASS)), None)


def wind_system_loaded(world):
    return "gz.msgs.Wind" in gz("topic", "-i", "-t", "/world/%s/wind" % world)


def load_wind_system(world):
    xml = ("<horizontal><magnitude><time_for_rise>0.5</time_for_rise></magnitude>"
           "<direction><time_for_rise>0.5</time_for_rise></direction></horizontal>"
           "<force_approximation_scaling_factor>%g</force_approximation_scaling_factor>" % BODY_K)
    req = ('entity: {id: 1}, plugins: {name: "gz::sim::systems::WindEffects", '
           'filename: "gz-sim-wind-effects-system", innerxml: "%s"}' % xml)
    for _ in range(3):
        gz("service", "-s", "/world/%s/entity/system/add" % world, "--reqtype", "gz.msgs.EntityPlugin_V",
           "--reptype", "gz.msgs.Boolean", "--timeout", "5000", "--req", req)
        time.sleep(1.0)
        if wind_system_loaded(world):
            return True
    return False


def set_wind(world, x, y, z):
    """x east, y north, z up (Gazebo's frame), m/s. A dropped message only means the old wind stays a bit longer."""
    gz("topic", "-t", "/world/%s/wind" % world, "-m", "gz.msgs.Wind",
       "-p", "linear_velocity: {x: %.3f, y: %.3f, z: %.3f}, enable_wind: true" % (x, y, z))


def main():
    p = argparse.ArgumentParser(description="Gusty wind for the 5\" and 7\" sims.")
    p.add_argument("speed", nargs="?", default="typical",
                   help="calm, typical, windy, storm, off, or a mean speed in m/s")
    p.add_argument("gust", nargs="?", type=float, help="gust peak in m/s (only with a number for speed)")
    p.add_argument("--from", dest="frm", type=float, default=250.0, help="direction it comes from, deg")
    p.add_argument("--steady", action="store_true", help="no gusts")
    a = p.parse_args()

    world = find_world()
    if a.speed == "off":
        for _ in range(3):
            set_wind(world, 0, 0, 0)
        print("Wind off.")
        return
    if a.speed in PRESETS:
        mean, peak = PRESETS[a.speed]
    else:
        try:
            mean = float(a.speed)
        except ValueError:
            sys.exit("speed has to be calm, typical, windy, storm, off or a number")
        peak = a.gust if a.gust is not None else mean * 1.6
    sigma = 0.0 if a.steady else max(peak - mean, 0.0) / 2.0
    tau = max(LENGTH / max(mean, 1.0), 1.0)

    if not wind_system_loaded(world) and not load_wind_system(world):
        sys.exit("Couldn't turn on Gazebo's wind system. Is this Gazebo Harmonic?")
    drone = find_drone() or "the drone"

    toward = math.radians(a.frm + 180.0)          # direction it blows toward
    ex, ey = math.sin(toward), math.cos(toward)   # along the wind (Gazebo x east, y north)
    lx, ly = -ey, ex                              # across it
    print("Wind on %s in world '%s': %.1f m/s (%.0f km/h) from %.0f deg, gusts to about %.1f m/s (%.0f km/h)."
          % (drone, world, mean, mean * 3.6, a.frm % 360, peak if sigma else mean, (peak if sigma else mean) * 3.6))
    print("Watch the lean in the Gazebo window, or at pxh> run: listener vehicle_local_position. Ctrl+C to stop.")

    u = v = w = 0.0      # turbulence along, across and vertical, m/s
    last = time.time()
    shown = 0.0
    try:
        while True:
            now = time.time()
            dt, last = now - last, now
            if sigma:
                c = math.exp(-dt / tau)
                s = math.sqrt(1.0 - c * c)
                u = c * u + s * sigma * random.gauss(0, 1)
                v = c * v + s * sigma * random.gauss(0, 1)
                w = c * w + s * 0.3 * sigma * random.gauss(0, 1)
            along = max(mean + u, 0.0)
            wx, wy = along * ex + v * lx, along * ey + v * ly
            set_wind(world, wx, wy, w)
            if now - shown > 2.0:
                speed = math.hypot(wx, wy)
                print("\r  wind now %4.1f m/s (%3.0f km/h)   " % (speed, speed * 3.6), end="", flush=True)
                shown = now
            time.sleep(max(DT - (time.time() - now), 0.0))
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(3):
            set_wind(world, 0, 0, 0)
        print("\nWind off.")


if __name__ == "__main__":
    main()
