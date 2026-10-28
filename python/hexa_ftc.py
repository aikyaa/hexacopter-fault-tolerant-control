# Hexacopter fault tolerant control - control re-allocation (thesis ch.3 / ch.4 sim)
# fault is assumed to be known immediately (no FDI)
import os
import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import lsq_linear, linprog
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# hexacopter parameters (table 3.1)
Jxx = 6.89e-3
Jyy = 6.89e-3
Jzz = 3.44e-2
l = 0.33
m = 0.95
kf = 3.13e-4
km = 7.5e-6
g = 9.81

w_max = 110.0      # max motor speed (rad/s)
tau_mot = 0.10     # motor time constant (s), chosen to match thesis fig 4.2 roll peak
c_yaw = 0.02       # rotor drag yaw damping (Nm per rad/s)
Ts = 0.01          # control loop time (s)

# same setup as thesis ch.4: hover at 10 m, fault at 2 s, 10 s run
z_ref = 10.0
t_fault = 2.0
t_end = 10.0

# PID gains (table 3.2, PX4 style cascade)
kp_ang = np.array([4.0, 4.0, 2.8])     # angle -> rate setpoint (6.5 too oscillatory with motor lag)
kp_rate = np.array([8.0, 8.0, 3.2])    # rate loop P
ki_rate = np.array([1.0, 1.0, 0.4])    # rate loop I
kp_zpos = 1.0                          # altitude -> climb rate setpoint
kp_z = 4.0                             # climb rate loop
ki_z = 2.0
kd_z = 0.1

# weight matrix for WCA (thesis sec 3.4.1)
Ws = np.diag([0.3, 0.3, 0.15, 0.25])

s3 = np.sqrt(3) / 2
# G matrix eq 1.7  ->  M = G * w^2   with M = [tau_phi, tau_theta, tau_psi, T]
G = np.array([
    [-l*kf, l*kf, 0.5*l*kf, -0.5*l*kf, -0.5*l*kf, 0.5*l*kf],
    [0, 0, -s3*l*kf, s3*l*kf, -s3*l*kf, s3*l*kf],
    [-km, km, -km, km, km, -km],
    [kf, kf, kf, kf, kf, kf]])

w_hover = np.sqrt(m * g / (6 * kf))

# approx. values read by eye from thesis fig 4.2-4.5 (orange = immediate CA, motor 1)
thesis = {"phi": 28, "wx": 200, "psi": 36, "wz": 100, "z": 9.55}


def get_G(fault):
    # fault = 0 -> healthy, fault = i -> motor i dead (column i = 0)
    Gi = G.copy()
    if fault > 0:
        Gi[:, fault-1] = 0
    return Gi


def allocate(M, fault=0, ftc=True):
    # returns motor speed commands (rad/s)
    if fault > 0 and ftc:
        Gi = get_G(fault)      # eq 3.31
        ok = [i for i in range(6) if i != fault-1]
    else:
        Gi = G                 # normal mixer
        ok = list(range(6))
    A = Ws @ Gi                # G_Wi = Ws * Gi
    b = Ws @ M
    u = np.linalg.pinv(A) @ b  # eq 3.32  w^2 = G_Wi^+ M
    if fault > 0 and ftc:
        u[fault-1] = 0
    # if motors saturate, do weighted least squares with limits
    if u.min() < 0 or u.max() > w_max**2:
        res = lsq_linear(A[:, ok], b, bounds=(0, w_max**2))
        u = np.zeros(6)
        u[ok] = res.x
    return np.sqrt(u)


def wrap(a):
    return (a + np.pi) % (2*np.pi) - np.pi


class Controller:
    def __init__(self):
        self.I_rate = np.zeros(3)
        self.I_z = 0.0
        self.ez_old = 0.0

    def update(self, x, z_ref, psi_ref=0.0):
        phi, th, psi, p, q, r, z, zd = x[:8]
        # altitude (outer P, inner PID on climb rate)
        vz_sp = np.clip(kp_zpos * (z_ref - z), -1.5, 1.5)
        ez = vz_sp - zd
        self.I_z = np.clip(self.I_z + ez*Ts, -3, 3)
        acc = kp_z*ez + ki_z*self.I_z + kd_z*(ez - self.ez_old)/Ts
        self.ez_old = ez
        c = max(np.cos(phi)*np.cos(th), 0.5)
        T = m * (g + acc) / c
        # attitude (angle P, rate PI)
        e_ang = np.array([0 - phi, 0 - th, wrap(psi_ref - psi)])
        rate_sp = np.clip(kp_ang * e_ang, -3.8, 3.8)
        e_rate = rate_sp - np.array([p, q, r])
        self.I_rate = np.clip(self.I_rate + e_rate*Ts, -1, 1)
        alpha = kp_rate*e_rate + ki_rate*self.I_rate
        tau = np.array([Jxx, Jyy, Jzz]) * alpha
        return np.array([tau[0], tau[1], tau[2], T])


def dist(t, on):
    # external disturbance [tau_x, tau_y, tau_z, Fz]
    d = np.zeros(4)
    if not on:
        return d
    if 3 <= t < 4:          # wind gust
        d[:3] = [0.06, -0.04, 0.01]
    if 6 <= t < 7:          # downward push / extra load
        d[3] = -2.0
    return d


def dyn(t, y, w_cmd, fault, d):
    phi, th, psi, p, q, r, z, zd = y[:8]
    w = y[8:14]
    M = G @ (w**2) + np.r_[d[:3], 0]
    M[2] = M[2] - c_yaw*r
    # eq 1.1
    pd = (Jyy - Jzz)/Jxx*q*r + M[0]/Jxx
    qd = (Jzz - Jxx)/Jyy*p*r + M[1]/Jyy
    rd = (Jxx - Jyy)/Jzz*p*q + M[2]/Jzz
    # eq 1.2
    phid = p + np.sin(phi)*np.tan(th)*q + np.cos(phi)*np.tan(th)*r
    thd = np.cos(phi)*q - np.sin(phi)*r
    psid = (np.sin(phi)*q + np.cos(phi)*r) / np.cos(th)
    # eq 1.3
    zdd = np.cos(phi)*np.cos(th)*M[3]/m - g + d[3]/m
    # motors (first order)
    wd = (w_cmd - w) / tau_mot
    if fault > 0:
        wd[fault-1] = 0
    return np.r_[phid, thd, psid, pd, qd, rd, zd, zdd, wd]


def plant_step(y, t, w_cmd, fault, d):
    y = y.copy()
    if fault > 0:
        y[8+fault-1] = 0     # failed motor stops at once (T_i = 0)
    sol = solve_ivp(dyn, (t, t+Ts), y, method="DOP853",
                    args=(w_cmd, fault, d), rtol=1e-7, atol=1e-9)
    return sol.y[:, -1]


def start_state():
    y = np.zeros(14)
    y[6] = z_ref               # start in hover
    y[8:14] = w_hover
    return y


def run_case(fault=0, dist_on=False, ftc=True):
    y = start_state()
    ctrl = Controller()
    data = []
    crashed = False
    n = int(round(t_end / Ts))
    for k in range(n+1):
        t = k*Ts
        f = fault if (fault > 0 and t >= t_fault) else 0   # fault info known at once
        if f > 0:
            y[8+f-1] = 0
        M_des = ctrl.update(y, z_ref)
        w_cmd = allocate(M_des, f, ftc)
        M_act = G @ (y[8:14]**2)
        data.append(np.r_[t, y, M_des, M_act])
        if abs(y[0]) > np.pi/2 or abs(y[1]) > np.pi/2 or y[6] < 0:
            crashed = True
            break
        if k < n:
            y = plant_step(y, t, w_cmd, f, dist(t, dist_on))
    return np.array(data), crashed


def controllability():
    # thesis sec 3.1.3: rank of C_i = [B AB A^2B A^3B], B = Jc * G_i, at hover (A = 0)
    Jc = np.diag([1/Jxx, 1/Jyy, 1/Jzz, 1.0])
    A = np.zeros((4, 4))
    ranks = []
    for i in range(0, 7):
        B = Jc @ get_G(i)
        C = np.hstack([B, A @ B, A @ A @ B, A @ A @ A @ B])
        ranks.append(np.linalg.matrix_rank(C))
    return ranks


def authority(fault):
    # thesis sec 3.3: largest roll / pitch / yaw torque the remaining motors can make
    # while the thrust is mg and the other two torques are zero (linear program)
    Gi = get_G(fault)
    ok = [i for i in range(6) if i != fault-1]
    A = Gi[:, ok]
    out = []
    for ax in range(3):
        others = [a for a in range(3) if a != ax]
        Aeq = np.vstack([A[others], A[3]])
        beq = [0, 0, m*g]
        lim = []
        for sign in [-1, 1]:
            r = linprog(-sign*A[ax], A_eq=Aeq, b_eq=beq, bounds=(0, w_max**2))
            lim.append(sign * (-r.fun) if r.status == 0 else 0.0)
        out.append(lim)
    return np.array(out)      # rows roll, pitch, yaw ; columns min, max


def plot_authority(folder):
    names = ["healthy"] + ["M%d fail" % i for i in range(1, 7)]
    L = np.array([authority(i) for i in range(7)])    # (7, 3, 2)
    fig, ax = plt.subplots(1, 3, figsize=(14, 4))
    for k, nm in enumerate(["roll", "pitch", "yaw"]):
        ax[k].bar(np.arange(7), L[:, k, 1], label="max")
        ax[k].bar(np.arange(7), L[:, k, 0], label="min")
        ax[k].set_xticks(np.arange(7))
        ax[k].set_xticklabels(names, rotation=45)
        ax[k].set_title("%s torque authority at hover thrust" % nm)
        ax[k].set_ylabel("torque (Nm)")
        ax[k].grid(True)
        ax[k].legend()
    fig.tight_layout()
    fig.savefig(os.path.join(folder, "control_authority.png"), dpi=110)
    plt.close(fig)
    with open(os.path.join(folder, "control_authority.csv"), "w") as f:
        f.write("case,roll_min,roll_max,pitch_min,pitch_max,yaw_min,yaw_max\n")
        for i in range(7):
            f.write(names[i] + "," + ",".join("%.4f" % (round(v, 4) + 0.0) for v in L[i].flatten()) + "\n")
    return L


def metrics(D):
    t = D[:, 0]
    a = t >= t_fault
    ang = np.rad2deg(D[:, 1:7])
    res = {}
    res["max_roll_deg"] = np.abs(ang[a, 0]).max()
    res["max_pitch_deg"] = np.abs(ang[a, 1]).max()
    res["final_yaw_deg"] = ang[-1, 2]
    res["max_wx_degps"] = np.abs(ang[a, 3]).max()
    res["max_wz_degps"] = np.abs(ang[a, 5]).max()
    res["min_z_m"] = D[a, 7].min()
    # settling: time after t_fault until roll,pitch < 2 deg and alt err < 5 cm
    bad = (np.abs(ang[:, 0]) > 2) | (np.abs(ang[:, 1]) > 2) | (np.abs(D[:, 7] - z_ref) > 0.05)
    bad_t = t[bad & a]
    res["settle_time_s"] = (bad_t[-1] - t_fault) if len(bad_t) > 0 else 0.0
    res["final_motor_speeds"] = " ".join(str(int(round(v))) for v in D[-1, 9:15])
    return res


def plot_case(D, name, title, show_fault=True, D2=None, folder="results", ref=None):
    # same layout as thesis: (phi, wx), (theta, wy), (psi, wz), (z, zdot), motors
    os.makedirs(folder, exist_ok=True)
    t = D[:, 0]
    deg = np.rad2deg
    fig, ax = plt.subplots(5, 2, figsize=(12, 16))
    ax = ax.flatten()
    ax[0].plot(t, deg(D[:, 1]), label="$\\phi$")
    ax[0].set_ylabel("$\\phi$ (deg)")
    ax[1].plot(t, deg(D[:, 4]), label="$\\omega_x$")
    ax[1].set_ylabel("$\\omega_x$ (deg/s)")
    ax[2].plot(t, deg(D[:, 2]), label="$\\theta$")
    ax[2].set_ylabel("$\\theta$ (deg)")
    ax[3].plot(t, deg(D[:, 5]), label="$\\omega_y$")
    ax[3].set_ylabel("$\\omega_y$ (deg/s)")
    ax[4].plot(t, deg(D[:, 3]), label="$\\psi$")
    ax[4].set_ylabel("$\\psi$ (deg)")
    ax[5].plot(t, deg(D[:, 6]), label="$\\omega_z$")
    ax[5].set_ylabel("$\\omega_z$ (deg/s)")
    ax[6].plot(t, D[:, 7], label="z")
    ax[6].axhline(z_ref, color="k", ls=":", lw=1, label="z ref")
    ax[6].set_ylabel("z (m)")
    ax[7].plot(t, D[:, 8], label="$\\dot z$")
    ax[7].set_ylabel("$\\dot z$ (m/s)")
    for i in range(6):
        ax[8].plot(t, D[:, 9+i], label="M%d" % (i+1))
    ax[8].set_ylabel("motor speed (rad/s)")
    ax[9].plot(t, D[:, 15], label="$\\tau_\\phi$ des")
    ax[9].plot(t, D[:, 19], "--", label="$\\tau_\\phi$ act")
    ax[9].plot(t, D[:, 17]*10, label="$\\tau_\\psi$ des x10")
    ax[9].plot(t, D[:, 21]*10, "--", label="$\\tau_\\psi$ act x10")
    ax[9].set_ylabel("torque (Nm)")
    if D2 is not None:   # case without re-allocation, for comparison
        ax[0].plot(D2[:, 0], deg(D2[:, 1]), "k:", lw=1.5, label="no realloc. (crash)")
        ax[6].plot(D2[:, 0], D2[:, 7], "k:", lw=1.5, label="no realloc. (crash)")
        lim = max(40, 1.3*np.abs(deg(D[:, 1])).max())
        ax[0].set_ylim(-lim, lim)
    if ref is not None:  # thesis values, approx.
        ax[0].axhline(ref["phi"], color="gray", ls="-.", label="thesis peak (approx)")
        ax[1].axhline(ref["wx"], color="gray", ls="-.", label="thesis peak (approx)")
        ax[4].axhline(ref["psi"], color="gray", ls="-.", label="thesis final (approx)")
        ax[5].axhline(ref["wz"], color="gray", ls="-.", label="thesis peak (approx)")
        ax[6].axhline(ref["z"], color="gray", ls="-.", label="thesis min (approx)")
    for a in ax:
        lo, hi = a.get_ylim()
        if hi - lo < 0.01:           # flat line: don't zoom into numerical noise
            a.set_ylim((lo+hi)/2 - 0.5, (lo+hi)/2 + 0.5)
        if show_fault:
            a.axvline(t_fault, color="r", lw=1, label="fault")
        a.grid(True)
        a.legend(fontsize=7, loc="best")
        a.set_xlabel("time (s)")
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(os.path.join(folder, name + ".png"), dpi=110)
    plt.close(fig)


def save_csv(D, path):
    head = "t,phi,theta,psi,p,q,r,z,zdot,w1,w2,w3,w4,w5,w6," \
           "tau_phi_d,tau_theta_d,tau_psi_d,T_d,tau_phi,tau_theta,tau_psi,T"
    np.savetxt(path, D, delimiter=",", header=head, comments="", fmt="%.6f")


if __name__ == "__main__":
    folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")
    os.makedirs(folder, exist_ok=True)
    ranks = controllability()
    print("rank of controllability matrix (healthy, motor 1..6 failed):", ranks)
    L = plot_authority(folder)
    print("torque authority [min max] roll/pitch/yaw, healthy:", np.round(L[0], 3).tolist())
    print("torque authority [min max] roll/pitch/yaw, motor 1 failed:", np.round(L[1], 3).tolist())
    rows = []
    cases = [("no_fault", 0, False), ("no_fault_disturbance", 0, True)]
    for i in range(1, 7):
        cases.append(("motor%d_fault" % i, i, False))

    fig2, ax2 = plt.subplots(2, 3, figsize=(13, 6))
    ax2 = ax2.flatten()
    for name, fault, dis in cases:
        D, crash = run_case(fault, dis, ftc=True)
        save_csv(D, os.path.join(folder, name + ".csv"))
        if fault > 0:
            D2, crash2 = run_case(fault, dis, ftc=False)
            title = "Motor %d failure at t=%.0fs with control re-allocation" % (fault, t_fault)
            ref = thesis if fault == 1 else None
            plot_case(D, name, title, True, D2, folder, ref)
            a = ax2[fault-1]
            a.plot(D[:, 0], np.rad2deg(D[:, 1]), label="with re-allocation")
            a.plot(D2[:, 0], np.rad2deg(D2[:, 1]), "k:", label="without")
            a.set_ylim(-90, 90)
            a.set_title("Motor %d failure (roll)" % fault)
            a.set_xlabel("time (s)")
            a.set_ylabel("roll (deg)")
            a.grid(True)
            a.legend(fontsize=7)
        else:
            crash2 = ""
            title = "No failure" + (" + disturbance (gust 3-4s, push 6-7s)" if dis else "")
            plot_case(D, name, title, False, None, folder)
        r = metrics(D)
        r["case"] = name
        r["crashed"] = crash
        r["crashed_without_realloc"] = crash2
        rows.append(r)
        print(name, "crashed:", crash, "| no realloc crashed:", crash2,
              "| roll %.1f pitch %.1f yaw_end %.1f wx %.0f wz %.0f zmin %.2f settle %.2f"
              % (r["max_roll_deg"], r["max_pitch_deg"], r["final_yaw_deg"], r["max_wx_degps"],
                 r["max_wz_degps"], r["min_z_m"], r["settle_time_s"]))
    fig2.tight_layout()
    fig2.savefig(os.path.join(folder, "compare_with_without_realloc.png"), dpi=110)

    keys = ["case", "crashed", "crashed_without_realloc", "max_roll_deg", "max_pitch_deg",
            "final_yaw_deg", "max_wx_degps", "max_wz_degps", "min_z_m", "settle_time_s",
            "final_motor_speeds"]
    with open(os.path.join(folder, "summary.csv"), "w") as f:
        f.write(",".join(keys) + "\n")
        for r in rows:
            f.write(",".join(str(round(float(r[k]), 3)) if isinstance(r[k], (float, np.floating)) else str(r[k]) for k in keys) + "\n")
    print("hover speed %.1f rad/s" % w_hover)
