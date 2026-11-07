"""7-inch on 4S: same 2807 motor size, wound for 4S (KV x a, R / a^2, I0 x a). Same air, payload and frame as seven.py."""
import math
from scipy.optimize import brentq
exec(open('prop7b.py').read().split('r0 = rho(0,25)')[0])
Kv6,R6,Ct3,Cp3 = map(float, open('fit.txt').read().split())
Ct2, Cp2 = Ct3*0.75, Cp3*0.68
r = rho(330, 10); g0 = 9.81
def run(name, cells, kv_nom, batt_g, cap_mAh, Rpack, payload=190, motor_g=52, avionics_W=18):
    a = kv_nom/1300; Kv = Kv6*a; R = R6/a**2; I0 = 1.5*a
    V0 = 3.7*cells
    frame=160; esc=14; props=4*7
    dry = payload+frame+4*motor_g+props+esc
    auw = (dry+batt_g)*1.10; W = auw/1000*g0
    Vb=V0
    for _ in range(20):
        rpm,Ib,T,Im = motor(1.0,Vb,Kv,R,I0,Ct2,Cp2,r,D7); Vb = V0-4*Ib*Rpack
    Vh=V0
    for _ in range(20):
        d = brentq(lambda d: 4*motor(d,Vh,Kv,R,I0,Ct2,Cp2,r,D7)[2]-W, 0.05, 1.0)
        rpmh,Ih,Th,Imh = motor(d,Vh,Kv,R,I0,Ct2,Cp2,r,D7); Vh=V0-4*Ih*Rpack
    Iav = avionics_W/V0   # FC, Pi 5, lidar, flow, RX (eCalc 'additional current'; 5" run used 0.8 A at 6S = 18 W)
    t_h = 0.85*cap_mAh/1000/(4*Ih+Iav)*60
    wh = V0*cap_mAh/1000
    print(f"{name}: AUW {auw:.0f} g | T/W {4*T/W:.2f} | hover {d*100:.0f}% | hover {4*Ih+Iav:.1f} A -> {t_h:.1f} min (mixed ~{0.55*t_h:.1f}) | max {Ib:.1f} A/motor ({4*Ib:.0f} A, {4*Ib/(cap_mAh/1000):.0f}C) | pack {wh:.0f} Wh")
    w_max = rpm*2*math.pi/60; k = T/w_max**2
    print(f"   sim: mass {auw/1000:.3f} kg, maxRotVelocity {w_max:.0f} rad/s, motorConstant {k:.3e}, hover omega {math.sqrt(W/4/k):.0f} rad/s ({math.sqrt(W/4/k)/w_max:.2f} of max)")
    return auw
print("Phase 1 payload (190 g)")
run("6S4000 1300KV (current pick)", 6, 1300, 600, 4000, 0.021)
for kv in (1700, 1950):
    run(f"4S4000 {kv}KV", 4, kv, 430, 4000, 0.016)
    run(f"4S5000 {kv}KV", 4, kv, 520, 5000, 0.014)
    run(f"4S6000 {kv}KV", 4, kv, 640, 6000, 0.013)
print("Phase 2 payload (+OAK-D Lite 61 g, GPS 10 g = 261 g)")
run("4S5000 1700KV", 4, 1700, 520, 5000, 0.014, payload=261, avionics_W=18)
run("6S4000 1300KV phase 2", 6, 1300, 600, 4000, 0.021, payload=261, avionics_W=18)
