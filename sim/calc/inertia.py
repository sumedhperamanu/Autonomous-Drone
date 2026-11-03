import numpy as np
def box(m, sx, sy, sz, p):
    I = m/12*np.diag([sy**2+sz**2, sx**2+sz**2, sx**2+sy**2]); return m, np.array(p), I
def point(m, p): return m, np.array(p), np.zeros((3,3))
def rod_diag(m, L, ang, z):  # rod from centre outward along angle
    c, s = np.cos(ang), np.sin(ang); u = np.array([c, s, 0])
    Ic = m*L**2/12*(np.eye(3)-np.outer(u,u)); return m, u*L/2+np.array([0,0,z]), Ic
def total(parts, scale_to=None):
    M = sum(p[0] for p in parts); cg = sum(p[0]*p[1] for p in parts)/M
    I = np.zeros((3,3))
    for m,p,Ic in parts:
        d = p-cg; I += Ic + m*(np.dot(d,d)*np.eye(3)-np.outer(d,d))
    if scale_to: I *= scale_to/M; M = scale_to
    return M, cg, I
def build(arm, motor_g, frame_g, batt_g, batt_dims, arm_len):
    P=[]
    for sx in (1,-1):
        for sy in (1,-1):
            P.append(point(motor_g/1000, [sx*arm, sy*arm, 0.01]))
    for k in range(4): P.append(rod_diag(frame_g*0.6/4/1000, arm_len, np.pi/4+k*np.pi/2, 0.0))
    P.append(box(frame_g*0.4/1000, 0.12, 0.07, 0.03, [0,0,0.01]))
    P.append(box(batt_g/1000, *batt_dims, [0,0,0.045]))
    P.append(box(0.056, 0.085, 0.056, 0.02, [0,0,-0.02]))       # Pi 5 + cooler under top plate
    P.append(point(0.047, [0,0,0.10]))                          # LD19 on mast
    P.append(box(0.0315+0.06+0.014, 0.10, 0.06, 0.04, [0,0,0.0]))  # FC, ESC, BEC, RX, flow, mounts, wiring
    return P
# 5": rotors +-0.078, 33 g motors, 120 g frame, 6S1800 290 g
p5 = build(0.078, 33, 120, 290, (0.075,0.035,0.040), 0.11)
M5,cg5,I5 = total(p5, scale_to=0.819)
# 7": rotors +-0.104, 52 g motors, 160 g frame, 6S4000 600 g
p7 = build(0.104, 52, 160, 600, (0.155,0.046,0.043), 0.147)
M7,cg7,I7 = total(p7, scale_to=1.320-4*0.007)
print("5in base_link", M5, np.round(np.diag(I5),5), "Day2 used 0.0016 0.0023 0.0035")
print("7in base_link", round(M7,3), np.round(np.diag(I7),5))
r = np.diag(I7)/np.diag(I5); print("ratio 7/5", np.round(r,2))
print("7in scaled from Day2 values:", np.round(np.array([0.0016,0.0023,0.0035])*r,4))
