"""Generate the 7-inch Gazebo model from the 5-inch one.
Geometry scales by k = 0.104/0.078 (rotor offset 104 mm = 295 mm motor-to-motor).
Mass, inertia, rotor and motor numbers are set from the 7" estimate (calc/seven.py, inertia.py)."""
import re, sys, pathlib
src = pathlib.Path(sys.argv[1]); dst = pathlib.Path(sys.argv[2])
k = 0.104/0.078
s = (src/'models/fiveinch_base/model.sdf').read_text()
def scale_nums(txt, f, n=None):
    parts = txt.split(); out=[]
    for i,p in enumerate(parts):
        out.append(f"{float(p)*f:.6g}" if (n is None or i < n) else p)
    return ' '.join(out)
s = re.sub(r'<pose>([^<]*)</pose>', lambda m: f"<pose>{scale_nums(m.group(1), k, 3)}</pose>", s)
s = re.sub(r'<size>([^<]*)</size>', lambda m: f"<size>{scale_nums(m.group(1), k)}</size>", s)
s = s.replace('<scale>0.448276 0.448276 0.448276</scale>', '<scale>0.597701 0.597701 0.597701</scale>')
s = s.replace('<scale>0.37931 0.37931 0.37931</scale>', '<scale>0.520446 0.520446 0.520446</scale>')  # 7.0" / 13.45" prop mesh
s = s.replace('fiveinch_base', 'seveninch_base')
# body mass and inertia (1.320 kg AUW minus four 7 g props)
s = s.replace('<mass>0.819</mass>', '<mass>1.292</mass>')
s = s.replace('<ixx>0.0016</ixx>', '<ixx>0.0040</ixx>').replace('<iyy>0.0023</iyy>', '<iyy>0.0054</iyy>').replace('<izz>0.0035</izz>', '<izz>0.0074</izz>')
# props: 7 g two-blade 7x3.5 (thin rod, 178 mm): Izz = m L^2 / 12
s = s.replace('<mass>0.0045</mass>', '<mass>0.007</mass>')
s = s.replace('<ixx>3.1e-06</ixx>', '<ixx>9.8e-06</ixx>').replace('<iyy>3.1e-06</iyy>', '<iyy>9.8e-06</iyy>').replace('<izz>6e-06</izz>', '<izz>1.9e-05</izz>')
assert s.count('<mass>1.292</mass>')==1 and s.count('<mass>0.007</mass>')==4
(dst/'models/seveninch_base').mkdir(parents=True, exist_ok=True)
(dst/'models/seveninch_base/model.sdf').write_text(s)
(dst/'models/seveninch_base/model.config').write_text((src/'models/fiveinch_base/model.config').read_text()
    .replace('fiveinch_base','seveninch_base')
    .replace('5-inch quad body, scaled from the PX4 x500 (0.84 kg, 220 mm motor to motor).',
             '7-inch quad body, scaled from the team 5-inch model (1.32 kg, 295 mm motor to motor).'))
# motor plugins
m = (src/'models/fiveinch/model.sdf').read_text()
m = m.replace("model name='fiveinch'", "model name='seveninch'").replace('model://fiveinch_base','model://seveninch_base')
m = m.replace('<maxRotVelocity>3121.0</maxRotVelocity>', '<maxRotVelocity>2213.0</maxRotVelocity>')
m = m.replace('<motorConstant>1.050e-06</motorConstant>', '<motorConstant>3.661e-06</motorConstant>')
m = m.replace('<momentConstant>0.016</momentConstant>', '<momentConstant>0.012</momentConstant>')
m = m.replace('<timeConstantUp>0.02</timeConstantUp>', '<timeConstantUp>0.03</timeConstantUp>')
m = m.replace('<timeConstantDown>0.025</timeConstantDown>', '<timeConstantDown>0.04</timeConstantDown>')
assert m.count('2213.0')==4 and m.count('3.661e-06')==4
(dst/'models/seveninch').mkdir(parents=True, exist_ok=True)
(dst/'models/seveninch/model.sdf').write_text(m)
(dst/'models/seveninch/model.config').write_text((src/'models/fiveinch/model.config').read_text().replace('fiveinch','seveninch').replace('5-inch','7-inch').replace('5"','7"'))
# sensor variant
l = (src/'models/fiveinch_lidar_flow/model.sdf').read_text()
l = l.replace('fiveinch_lidar_flow','seveninch_lidar_flow').replace('model://fiveinch','model://seveninch')
(dst/'models/seveninch_lidar_flow').mkdir(parents=True, exist_ok=True)
(dst/'models/seveninch_lidar_flow/model.sdf').write_text(l)
(dst/'models/seveninch_lidar_flow/model.config').write_text((src/'models/fiveinch_lidar_flow/model.config').read_text().replace('fiveinch','seveninch').replace('5-inch','7-inch').replace('5"','7"'))
print("ok")
