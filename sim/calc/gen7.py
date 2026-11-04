"""Generate the 7-inch Gazebo models from the 5-inch ones.

Usage:  python3 sim/calc/gen7.py sim/fiveinch <outdir>
Check:  python3 sim/calc/gen7.py sim/fiveinch /tmp/gen7 && diff -r /tmp/gen7/models sim/seveninch/models
        (no output from diff means the committed 7" models match this script)

Geometry scales by k = 0.104/0.078 (rotor offset 104 mm = 295 mm motor to motor).
Mass, inertia, rotor and motor numbers are set from the 7" estimate (calc/seven.py, inertia.py).
Every value is set by tag, not by matching the 5" value, so a change to the 5" model can't
leave a 5" number in the 7". The asserts stop the script if a value didn't land.
"""
import re, sys, pathlib

src = pathlib.Path(sys.argv[1]); dst = pathlib.Path(sys.argv[2])
k = 0.104 / 0.078

BODY = {"mass": "1.292", "ixx": "0.0040", "iyy": "0.0054", "izz": "0.0074"}  # 1.320 kg AUW minus four 7 g props
PROP = {"mass": "0.007", "ixx": "9.8e-06", "iyy": "9.8e-06", "izz": "1.9e-05"}  # 7 g 7x3.5, thin rod 178 mm: Izz = m L^2 / 12
MOTOR = {
    "maxRotVelocity": "2213.0",            # 2807 1300 KV on 6S. The 4S airframes cap it at 1964 with SIM_GZ_EC_MAX
    "motorConstant": "3.661e-06",
    "momentConstant": "0.012",
    "timeConstantUp": "0.03",
    "timeConstantDown": "0.04",
    "rotorDragCoefficient": "1.47e-04",    # props + 0.04 m^2 of body drag, matched at 10 m/s (see sim/wind.py)
}
DESC = {
    "seveninch_base": "7-inch quad body, scaled from the team 5-inch model (1.32 kg, 295 mm motor to motor).",
    "seveninch": "7-inch quad. Motor numbers from the motor model for a 2807 1300KV on 6S (max 2213 rad/s); "
                 "the 4S airframes 22053 and 22054 cap the motors at 1964 rad/s.",
    "seveninch_lidar_flow": "7-inch quad, phase 1 build: LD19-style 360 deg 2D lidar on a mast plus downward optical "
                            "flow and range sensor.",
}


def set_tags(block, values):
    """Set <tag>value</tag> for each tag in one block. Each tag must appear exactly once."""
    for tag, val in values.items():
        block, n = re.subn(rf"<{tag}>[^<]*</{tag}>", f"<{tag}>{val}</{tag}>", block)
        assert n == 1, f"{tag}: found {n} times, expected 1"
    return block


def set_links(sdf, name_re, values, expect):
    """Apply set_tags inside every <link name='...'> whose name matches name_re."""
    count = 0
    def fix(m):
        nonlocal count
        count += 1
        return set_tags(m.group(0), values)
    sdf = re.sub(rf"<link name=['\"]({name_re})['\"]>.*?</link>", fix, sdf, flags=re.S)
    assert count == expect, f"links matching {name_re}: {count}, expected {expect}"
    return sdf


def scale_nums(txt, f, n=None):
    parts = txt.split()
    return " ".join(f"{float(p) * f:.6g}" if (n is None or i < n) else p for i, p in enumerate(parts))


def config(name, src_config):
    txt = src_config.replace(src_config[src_config.index("<name>") + 6:src_config.index("</name>")], name)
    txt, n = re.subn(r"<description>[^<]*</description>", f"<description>{DESC[name]}</description>", txt)
    assert n == 1
    return txt


def write(rel, text):
    p = dst / "models" / rel
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(text)


# Body: scale geometry, swap meshes, set mass and inertia
s = (src / "models/fiveinch_base/model.sdf").read_text()
s = re.sub(r"<pose>([^<]*)</pose>", lambda m: f"<pose>{scale_nums(m.group(1), k, 3)}</pose>", s)
s = re.sub(r"<size>([^<]*)</size>", lambda m: f"<size>{scale_nums(m.group(1), k)}</size>", s)
s, n_body = re.subn(r"<scale>0\.448276 0\.448276 0\.448276</scale>", "<scale>0.597701 0.597701 0.597701</scale>", s)
s, n_prop = re.subn(r"<scale>0\.37931 0\.37931 0\.37931</scale>", "<scale>0.520446 0.520446 0.520446</scale>", s)  # 7.0" / 13.45" prop mesh
assert n_body > 0 and n_prop == 4, (n_body, n_prop)
s = s.replace("fiveinch_base", "seveninch_base")
s = set_links(s, "base_link", BODY, 1)
s = set_links(s, r"rotor_\d", PROP, 4)
write("seveninch_base/model.sdf", s)
write("seveninch_base/model.config", config("seveninch_base", (src / "models/fiveinch_base/model.config").read_text()))

# Motor plugins
m = (src / "models/fiveinch/model.sdf").read_text()
m = m.replace("model name='fiveinch'", "model name='seveninch'").replace("model://fiveinch_base", "model://seveninch_base")
for tag, val in MOTOR.items():
    m, n = re.subn(rf"<{tag}>[^<]*</{tag}>", f"<{tag}>{val}</{tag}>", m)
    assert n == 4, f"{tag}: found {n} times, expected 4 (one per motor)"
write("seveninch/model.sdf", m)
write("seveninch/model.config", config("seveninch", (src / "models/fiveinch/model.config").read_text()))

# Sensor version: same sensors, included on top of the 7" model
l = (src / "models/fiveinch_lidar_flow/model.sdf").read_text()
l = l.replace("fiveinch_lidar_flow", "seveninch_lidar_flow")
l, n = re.subn(r"<uri>(model://)?fiveinch</uri>", r"<uri>\1seveninch</uri>", l)
assert n == 1, f"sensor model include: found {n}, expected 1"
write("seveninch_lidar_flow/model.sdf", l)
write("seveninch_lidar_flow/model.config", config("seveninch_lidar_flow", (src / "models/fiveinch_lidar_flow/model.config").read_text()))

# Final check: no 5" leftovers in the 7" files
for rel in ["seveninch_base/model.sdf", "seveninch/model.sdf", "seveninch_lidar_flow/model.sdf"]:
    txt = (dst / "models" / rel).read_text()
    assert "fiveinch" not in txt, f"{rel} still mentions fiveinch"
print("ok:", dst / "models")
