import math, itertools
from scipy.optimize import brentq, least_squares
def rho(elev_m, temp_c):
    p = 101325*(1-2.25577e-5*elev_m)**5.25588
    return p/(287.05*(temp_c+273.15))
D7 = 7*0.0254
def motor(d, Vb, Kv, R, I0, Ct, Cp, r, D):
    """duty d, battery volts Vb -> rpm, battery current, thrust N, motor current"""
    Kt = 60/(2*math.pi*Kv)
    def f(n):
        Im = Cp*r*n**2*D**5/(2*math.pi)/Kt + I0
        return Kv*(d*Vb - Im*R)/60 - n
    n = brentq(f, 0.1, 2000)
    Im = Cp*r*n**2*D**5/(2*math.pi)/Kt + I0
    return n*60, Im*d, Ct*r*n**2*D**4, Im
r0 = rho(0,25)
tab=[(0.4,5.3,596,10925),(0.5,9.0,913,13198),(0.6,13.8,1258,15318),(0.7,20.3,1635,17419),(0.8,28.6,2053,19433),(0.9,38.7,2482,21096),(1.0,48.5,2824,22329)]
def res(p):
    Kv,R,Ct,Cp=p; out=[]
    for d,I,g,rpm in tab:
        rp,Ib,T,_=motor(d,24,Kv,R,1.5,Ct,Cp,r0,D7)
        out += [(rp-rpm)/rpm,(Ib-I)/I,(T*1000/9.81-g)/g]
    return out
fit=least_squares(res,[1250,0.06,0.16,0.074],bounds=([900,0.02,0.1,0.03],[1400,0.3,0.25,0.12]))
Kv,R,Ct3,Cp3=fit.x
print(f"fit Kv_eff={Kv:.0f} R_eff={R*1000:.0f} mOhm Ct3={Ct3:.3f} Cp3={Cp3:.4f}")
for d,I,g,rpm in tab:
    rp,Ib,T,_=motor(d,24,Kv,R,1.5,Ct3,Cp3,r0,D7)
    print(f" {d:.1f}: rpm {rp:6.0f}/{rpm}  I {Ib:5.1f}/{I}  g {T*1000/9.81:5.0f}/{g}")
open('fit.txt','w').write(f"{Kv} {R} {Ct3} {Cp3}\n")
