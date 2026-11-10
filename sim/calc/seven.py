import math
from scipy.optimize import brentq
exec(open('prop7b.py').read().split('r0 = rho(0,25)')[0])
Kv,R,Ct3,Cp3 = map(float, open('fit.txt').read().split())
Ct2, Cp2 = Ct3*0.75, Cp3*0.68      # 7x3.5 two-blade vs three-blade (assumed ratios)
r = rho(330, 10)
g0=9.81
def run(name, batt_g, cap_mAh, Rpack, extra=0):
    payload=190; frame=160; motors=4*52; props=4*7; esc=14
    dry = payload+frame+motors+props+esc+extra
    auw = (dry+batt_g)*1.10
    W = auw/1000*g0
    # max: iterate battery sag
    Vb=22.2
    for _ in range(20):
        rpm,Ib,T,Im = motor(1.0,Vb,Kv,R,1.5,Ct2,Cp2,r,D7); Vb = 22.2-4*Ib*Rpack
    rpm_max,I_max,T_max = rpm,Ib,T
    # hover
    Vh=22.2
    for _ in range(20):
        d = brentq(lambda d: 4*motor(d,Vh,Kv,R,1.5,Ct2,Cp2,r,D7)[2]-W, 0.05, 1.0)
        rpmh,Ih,Th,Imh = motor(d,Vh,Kv,R,1.5,Ct2,Cp2,r,D7); Vh=22.2-4*Ih*Rpack
    t_h = 0.85*cap_mAh/1000/(4*Ih)*60
    print(f"{name}: dry {dry:.0f} g, AUW {auw:.0f} g | T/W {4*T_max/W:.2f} | hover throttle {d*100:.0f}% ({rpmh:.0f} rpm) | "
          f"hover {4*Ih:.1f} A, {t_h:.1f} min (mixed ~{0.55*t_h:.1f}) | max {I_max:.1f} A/motor, {4*I_max:.0f} A total, {rpm_max:.0f} rpm, {T_max/g0*1000:.0f} g/motor")
    w_max = rpm_max*2*math.pi/60
    k = T_max/w_max**2
    print(f"   sim: maxRotVelocity {w_max:.0f} rad/s, motorConstant {k:.3e}, hover omega {math.sqrt(W/4/k):.0f} rad/s")
    return auw
run("7in 6S4000 (P2 spec)", 600, 4000, 0.021)
run("7in 6S4000 +15% AUW", 600, 4000, 0.021, extra=(600+600)*0.15/1.0)
run("7in 6S2200", 345, 2200, 0.030)
run("7in 6S3000", 460, 3000, 0.025)
