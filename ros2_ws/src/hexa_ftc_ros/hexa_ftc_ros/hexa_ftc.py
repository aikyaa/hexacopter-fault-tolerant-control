# Hexacopter fault tolerant control with control re-allocation (thesis ch.3 and ch.4)
# the failed motor is assumed to be known right away
import os
import numpy as np
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
tau_mot = 0.10     # motor time constant (s)
c_yaw = 0.02       # rotor drag yaw damping
Ts = 0.01          # control loop time (s)

z_ref = 10.0
t_fault = 2.0
t_end = 10.0

# PID gains (table 3.2)
kp_ang = np.array([4.0, 4.0, 2.8])
kp_rate = np.array([8.0, 8.0, 3.2])
ki_rate = np.array([1.0, 1.0, 0.4])
kp_zpos = 1.0
kp_z = 4.0
ki_z = 2.0
kd_z = 0.1

# weights for the allocation (sec 3.4.1)
Ws = np.diag([0.3, 0.3, 0.15, 0.25])

# M = G * w^2, M = [tau_phi, tau_theta, tau_psi, T]  (eq 1.7)
s3 = np.sqrt(3) / 2
G = np.array([
    [-l*kf, l*kf, 0.5*l*kf, -0.5*l*kf, -0.5*l*kf, 0.5*l*kf],
    [0, 0, -s3*l*kf, s3*l*kf, -s3*l*kf, s3*l*kf],
    [-km, km, -km, km, km, -km],
    [kf, kf, kf, kf, kf, kf]])

w_hover = np.sqrt(m * g / (6 * kf))


def motor_eff(fault, eta=1.0):
    # thrust efficiency of each motor, motor "fault" loses eta of its thrust
    eff = np.ones(6)
    if fault > 0:
        eff[fault-1] = 1 - eta
    return eff


def allocate(M, eff):
    # eff = what the controller knows about the motors (all ones = normal mixer)
    A = Ws @ (G * eff)
    b = Ws @ M
    u = np.linalg.pinv(A) @ b          # w^2 (eq 3.32)
    u[eff == 0] = 0
    if u.min() < 0 or u.max() > w_max**2:
        ok = eff > 0
        u = np.zeros(6)
        u[ok] = lsq_linear(A[:, ok], b, bounds=(0, w_max**2)).x
    return np.sqrt(u)


def wrap(a):
    return (a + np.pi) % (2*np.pi) - np.pi


class PID:
    def __init__(self):
        self.I_rate = np.zeros(3)
        self.I_z = 0.0
        self.ez_old = 0.0

    def update(self, x, z_ref):
        phi, th, psi, p, q, r, z, zd = x[:8]
        # altitude
        vz_sp = np.clip(kp_zpos * (z_ref - z), -1.5, 1.5)
        ez = vz_sp - zd
        self.I_z = np.clip(self.I_z + ez*Ts, -3, 3)
        acc = kp_z*ez + ki_z*self.I_z + kd_z*(ez - self.ez_old)/Ts
        self.ez_old = ez
        T = m * (g + acc) / max(np.cos(phi)*np.cos(th), 0.5)
        # attitude
        e_ang = np.array([-phi, -th, wrap(-psi)])
        rate_sp = np.clip(kp_ang * e_ang, -3.8, 3.8)
        e_rate = rate_sp - np.array([p, q, r])
        self.I_rate = np.clip(self.I_rate + e_rate*Ts, -1, 1)
        tau = np.array([Jxx, Jyy, Jzz]) * (kp_rate*e_rate + ki_rate*self.I_rate)
        return np.array([tau[0], tau[1], tau[2], T])


def dist(t, on):
    # wind gust at 3-4 s and a downward push at 6-7 s: [tau_x, tau_y, tau_z, Fz]
    d = np.zeros(4)
    if on and 3 <= t < 4:
        d[:3] = [0.06, -0.04, 0.01]
    if on and 6 <= t < 7:
        d[3] = -2.0
    return d


def no_payload():
    # (payload mass * position, inertia, total mass)
    return np.zeros(2), np.array([Jxx, Jyy, Jzz]), m


def dyn(y, w_cmd, eff, payload, d):
    phi, th, psi, p, q, r, z, zd = y[:8]
    w = y[8:14]
    mp_pos, J, mass = payload
    c = np.cos(phi) * np.cos(th)
    M = G @ (eff * w**2)
    M[:2] -= mp_pos * g * c            # payload weight off the centre
    M[:3] += d[:3]
    M[2] -= c_yaw * r
    # eq 1.1 - 1.3
    pd = (J[1] - J[2]) / J[0] * q * r + M[0] / J[0]
    qd = (J[2] - J[0]) / J[1] * p * r + M[1] / J[1]
    rd = (J[0] - J[1]) / J[2] * p * q + M[2] / J[2]
    phid = p + np.sin(phi) * np.tan(th) * q + np.cos(phi) * np.tan(th) * r
    thd = np.cos(phi) * q - np.sin(phi) * r
    psid = (np.sin(phi) * q + np.cos(phi) * r) / np.cos(th)
    zdd = c * M[3] / mass - g + d[3] / mass
    wd = (w_cmd - w) / tau_mot
    wd[eff == 0] = 0
    return np.r_[phid, thd, psid, pd, qd, rd, zd, zdd, wd]


def plant_step(y, w_cmd, eff, payload, d=np.zeros(4)):
    # RK4, 4 sub steps per control step
    dt = Ts / 4
    for i in range(4):
        k1 = dyn(y, w_cmd, eff, payload, d)
        k2 = dyn(y + dt/2 * k1, w_cmd, eff, payload, d)
        k3 = dyn(y + dt/2 * k2, w_cmd, eff, payload, d)
        k4 = dyn(y + dt * k3, w_cmd, eff, payload, d)
        y = y + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
    return y


def start_state():
    y = np.zeros(14)
    y[6] = z_ref
    y[8:14] = w_hover
    return y


def crashed(y):
    return abs(y[0]) > np.pi/2 or abs(y[1]) > np.pi/2 or y[6] < 0


def run_case(fault=0, dist_on=False, ftc=True):
    y = start_state()
    ctrl = PID()
    data = []
    crash = False
    n = int(round(t_end / Ts))
    for k in range(n+1):
        t = k*Ts
        f = fault if t >= t_fault else 0
        eff = motor_eff(f)
        y[8:14][eff == 0] = 0          # failed motor stops at once
        M_des = ctrl.update(y, z_ref)
        w_cmd = allocate(M_des, eff if ftc else np.ones(6))
        data.append(np.r_[t, y, M_des, G @ (eff * y[8:14]**2)])
        if crashed(y):
            crash = True
            break
        if k < n:
            y = plant_step(y, w_cmd, eff, no_payload(), dist(t, dist_on))
    return np.array(data), crash


def controllability():
    # rank of [B AB A^2B A^3B] at hover, A = 0 so this is the rank of B (sec 3.1.3)
    Jc = np.diag([1/Jxx, 1/Jyy, 1/Jzz, 1.0])
    return [np.linalg.matrix_rank(Jc @ (G * motor_eff(i))) for i in range(7)]


def authority(fault):
    # largest roll / pitch / yaw torque at hover thrust with the other torques zero (sec 3.3)
    ok = motor_eff(fault) > 0
    A = G[:, ok]
    lims = np.zeros((3, 2))
    for ax in range(3):
        others = [a for a in range(3) if a != ax]
        Aeq = np.vstack([A[others], A[3]])
        for j, sign in enumerate([-1, 1]):
            res = linprog(-sign*A[ax], A_eq=Aeq, b_eq=[0, 0, m*g], bounds=(0, w_max**2))
            lims[ax, j] = -sign * res.fun if res.status == 0 else 0.0
    return lims


def plot_authority(folder):
    names = ["healthy"] + ["M%d fail" % i for i in range(1, 7)]
    L = np.array([authority(i) for i in range(7)])
    fig, ax = plt.subplots(1, 3, figsize=(14, 4))
    for k, axis in enumerate(["roll", "pitch", "yaw"]):
        ax[k].bar(range(7), L[:, k, 1], label="max")
        ax[k].bar(range(7), L[:, k, 0], label="min")
        ax[k].set_xticks(range(7))
        ax[k].set_xticklabels(names, rotation=45)
        ax[k].set_title("%s torque authority at hover thrust" % axis)
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
    after = t >= t_fault
    ang = np.rad2deg(D[:, 1:7])
    res = {}
    res["max_roll_deg"] = np.abs(ang[after, 0]).max()
    res["max_pitch_deg"] = np.abs(ang[after, 1]).max()
    res["final_yaw_deg"] = ang[-1, 2]
    res["max_wx_degps"] = np.abs(ang[after, 3]).max()
    res["max_wz_degps"] = np.abs(ang[after, 5]).max()
    res["min_z_m"] = D[after, 7].min()
    # settle time: roll and pitch < 2 deg and altitude error < 5 cm
    bad = (np.abs(ang[:, 0]) > 2) | (np.abs(ang[:, 1]) > 2) | (np.abs(D[:, 7] - z_ref) > 0.05)
    bad_t = t[bad & after]
    res["settle_time_s"] = bad_t[-1] - t_fault if len(bad_t) > 0 else 0.0
    res["final_motor_speeds"] = " ".join(str(int(round(v))) for v in D[-1, 9:15])
    return res


def plot_case(D, name, title, t_mark=None, D2=None, folder="results"):
    # same layout as the thesis plots
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
    if D2 is not None:
        ax[0].plot(D2[:, 0], deg(D2[:, 1]), "k:", lw=1.5, label="no realloc. (crash)")
        ax[6].plot(D2[:, 0], D2[:, 7], "k:", lw=1.5, label="no realloc. (crash)")
        lim = max(40, 1.3*np.abs(deg(D[:, 1])).max())
        ax[0].set_ylim(-lim, lim)
    for a in ax:
        lo, hi = a.get_ylim()
        if hi - lo < 0.01:
            a.set_ylim((lo+hi)/2 - 0.5, (lo+hi)/2 + 0.5)
        if t_mark is not None:
            a.axvline(t_mark, color="r", lw=1, label="fault")
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
    print("rank of controllability matrix (healthy, motor 1..6 failed):", controllability())
    L = plot_authority(folder)
    print("torque authority healthy:", np.round(L[0], 3).tolist())
    print("torque authority motor 1 failed:", np.round(L[1], 3).tolist())

    cases = [("no_fault", 0, False), ("no_fault_disturbance", 0, True)]
    for i in range(1, 7):
        cases.append(("motor%d_fault" % i, i, False))
    rows = []
    fig2, ax2 = plt.subplots(2, 3, figsize=(13, 6))
    ax2 = ax2.flatten()
    for name, fault, dis in cases:
        D, crash = run_case(fault, dis, ftc=True)
        save_csv(D, os.path.join(folder, name + ".csv"))
        crash2 = ""
        if fault > 0:
            D2, crash2 = run_case(fault, dis, ftc=False)
            title = "Motor %d failure at t=%.0fs with control re-allocation" % (fault, t_fault)
            plot_case(D, name, title, t_fault, D2, folder)
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
            title = "No failure" + (" + disturbance (gust 3-4s, push 6-7s)" if dis else "")
            plot_case(D, name, title, None, None, folder)
        r = metrics(D)
        r["case"] = name
        r["crashed"] = crash
        r["crashed_without_realloc"] = crash2
        rows.append(r)
        print(name, "crashed:", crash, "| without realloc:", crash2,
              "| roll %.1f pitch %.1f yaw %.1f zmin %.2f settle %.2f"
              % (r["max_roll_deg"], r["max_pitch_deg"], r["final_yaw_deg"], r["min_z_m"], r["settle_time_s"]))
    fig2.tight_layout()
    fig2.savefig(os.path.join(folder, "compare_with_without_realloc.png"), dpi=110)

    keys = ["case", "crashed", "crashed_without_realloc", "max_roll_deg", "max_pitch_deg",
            "final_yaw_deg", "max_wx_degps", "max_wz_degps", "min_z_m", "settle_time_s",
            "final_motor_speeds"]
    with open(os.path.join(folder, "summary.csv"), "w") as f:
        f.write(",".join(keys) + "\n")
        for r in rows:
            vals = []
            for k in keys:
                v = r[k]
                vals.append(str(round(float(v), 3)) if isinstance(v, (float, np.floating)) else str(v))
            f.write(",".join(vals) + "\n")
    print("hover speed %.1f rad/s" % w_hover)
