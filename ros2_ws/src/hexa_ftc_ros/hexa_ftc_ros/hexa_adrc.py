# Hexacopter passive fault tolerant control with ADRC (TD + LESO), following
# "Active Disturbance Rejection based approach for FTC of a Quadcopter" (ETAAV 2025)
# the motor fault and an off-centre payload are both unknown to the controller:
# the LESO lumps them into one total disturbance and cancels it
# compared against the PID cascade of hexa_ftc.py with and without fault knowledge
import os
import numpy as np
from scipy.optimize import lsq_linear
from multiprocessing import Pool
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
try:
    from hexa_ftc_ros import hexa_ftc as h     # inside the ROS 2 package
except ImportError:
    import hexa_ftc as h
G, Ws, Jxx, Jyy, Jzz, m, kf, g = h.G, h.Ws, h.Jxx, h.Jyy, h.Jzz, h.m, h.kf, h.g
w_max, tau_mot, c_yaw, Ts, z_ref = h.w_max, h.tau_mot, h.c_yaw, h.Ts, h.z_ref

t_fault = 5.0      # later than hexa_ftc so the payload transient settles first
t_end = 15.0
mp = 0.2           # payload mass (kg), ~20% of the vehicle
t_ramp = 2.0       # payload pick-up time (s)
fmot = 1           # failed motor in the payload study

# thrust f on motor i makes roll/pitch torque lever[:, i] * f  (from G)
lever = G[:2] / kf
arm_ang = np.rad2deg(np.arctan2(lever[1], lever[0])) % 360

# ADRC tuning: closed loop bandwidth wc, observer bandwidth wo (rad/s), TD accel limit r0
# r0 values from the paper; roll/pitch wc/wo set high enough to catch a sudden motor
# loss without being told about it (picked by grid search)
adrc_par = {"roll": (14.0, 50.0, 201.875), "pitch": (14.0, 50.0, 201.875),
            "yaw": (2.0, 10.0, 23.3527), "alt": (2.0, 10.0, 30.4)}


def fhan(x1, x2, r, h0):
    # Han's time-optimal tracking function (paper eq 14-15, ref [16])
    d = r * h0
    d0 = h0 * d
    y = x1 + h0 * x2
    a0 = np.sqrt(d * d + 8 * r * abs(y))
    a = x2 + (a0 - d) / 2 * np.sign(y) if abs(y) > d0 else x2 + y / h0
    return -r * np.sign(a) if abs(a) > d else -r * a / d


class ADRCAxis:
    # one SISO loop: TD on the reference, 3rd order LESO, PD on the estimated states
    def __init__(self, b0, wc, wo, r0, y0, ff=0.0):
        self.b0, self.r0, self.ff = b0, r0, ff
        self.kp, self.kd = wc**2, 2 * wc
        self.b1, self.b2, self.b3 = 3 * wo, 3 * wo**2, wo**3    # bandwidth parameterised LESO
        self.z = np.array([y0, 0.0, 0.0])
        self.v = np.array([y0, 0.0])
        self.u = 0.0                                             # last applied input

    def update(self, ref, y):
        # LESO (eq 21) with the input that was actually allocated last step
        e = self.z[0] - y
        z1, z2, z3 = self.z
        self.z = np.array([z1 + Ts * (z2 - self.b1 * e),
                           z2 + Ts * (z3 + self.ff + self.b0 * self.u - self.b2 * e),
                           z3 - Ts * self.b3 * e])
        # TD / profile generator (eq 16)
        v1, v2 = self.v
        self.v = np.array([v1 + Ts * v2, v2 + Ts * fhan(v1 - ref, v2, self.r0, Ts)])
        # control law (eq 23), disturbance z3 cancelled
        u0 = self.kp * (self.v[0] - self.z[0]) + self.kd * (self.v[1] - self.z[1])
        return (u0 - self.z[2] - self.ff) / self.b0


class ADRC:
    def __init__(self, y):
        P = adrc_par
        self.ax = [ADRCAxis(1/Jxx, *P["roll"], y[0]), ADRCAxis(1/Jyy, *P["pitch"], y[1]),
                   ADRCAxis(1/Jzz, *P["yaw"], y[2]), ADRCAxis(1/m, *P["alt"], y[6], ff=-g)]

    def update(self, y, zr):
        refs = [0.0, 0.0, 0.0, zr]
        meas = [y[0], y[1], y[2], y[6]]
        M = np.array([a.update(r, v) for a, r, v in zip(self.ax, refs, meas)])
        M[3] = np.clip(M[3], 0, 6 * kf * w_max**2)
        return M

    def applied(self, M):
        for a, u in zip(self.ax, M):
            a.u = u


class PID(h.Controller):
    # baseline cascade from hexa_ftc.py, nothing changed
    def __init__(self, y):
        super().__init__()

    def applied(self, M):
        pass


def allocate(M, eff, mode):
    # realloc: knows motor effectiveness (fault known at once, as in hexa_ftc)
    # normal: healthy mixer, no fault information
    Gi = G * eff if mode == "realloc" else G
    ok = eff > 0 if mode == "realloc" else np.ones(6, bool)
    A = Ws @ Gi
    b = Ws @ M
    u = np.linalg.pinv(A) @ b
    if u.min() < 0 or u.max() > w_max**2:
        res = lsq_linear(A[:, ok], b, bounds=(0, w_max**2))
        u = np.zeros(6)
        u[ok] = res.x
    return np.sqrt(u), Gi          # motor commands and the G matrix the controller believes


def payload(r, ang, mpl):
    # payload of mass mpl at radius r (m) in direction ang (deg, same frame as arm_ang)
    p = r * np.array([np.cos(np.deg2rad(ang)), np.sin(np.deg2rad(ang))])
    J = np.array([Jxx + mpl * p[0]**2, Jyy + mpl * p[1]**2, Jzz + mpl * r**2])
    return p * mpl, J, m + mpl


def dyn(y, w_cmd, eff, P, d):
    p_pl, J, mt = P
    phi, th, psi, p, q, r, z, zd = y[:8]
    w = y[8:14]
    cc = np.cos(phi) * np.cos(th)
    M = G @ (eff * w**2)
    M[:2] -= p_pl * g * cc               # payload weight acting off-centre
    M[:3] += d[:3]                       # external disturbance (hexa_ftc.dist)
    M[2] -= c_yaw * r
    pd = (J[1] - J[2]) / J[0] * q * r + M[0] / J[0]
    qd = (J[2] - J[0]) / J[1] * p * r + M[1] / J[1]
    rd = (J[0] - J[1]) / J[2] * p * q + M[2] / J[2]
    phid = p + np.sin(phi) * np.tan(th) * q + np.cos(phi) * np.tan(th) * r
    thd = np.cos(phi) * q - np.sin(phi) * r
    psid = (np.sin(phi) * q + np.cos(phi) * r) / np.cos(th)
    zdd = cc * M[3] / mt - g + d[3] / mt
    wd = (w_cmd - w) / tau_mot
    wd[eff == 0] = 0
    return np.r_[phid, thd, psid, pd, qd, rd, zd, zdd, wd]


def plant_step(y, w_cmd, eff, P, d=np.zeros(4), n=4):
    # RK4 with n sub steps per control step
    dt = Ts / n
    for _ in range(n):
        k1 = dyn(y, w_cmd, eff, P, d)
        k2 = dyn(y + dt/2 * k1, w_cmd, eff, P, d)
        k3 = dyn(y + dt/2 * k2, w_cmd, eff, P, d)
        k4 = dyn(y + dt * k3, w_cmd, eff, P, d)
        y = y + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
    return y


class Loop:
    # one control step: controller -> allocation -> motor model that feeds the LESO
    # mode "normal" = no fault information, "realloc" = fault known at once
    # (shared by run_case and the ROS 2 ftc_node)
    def __init__(self, ctrl, y, mode):
        self.ctrl, self.mode = ctrl, mode
        self.C = ADRC(y) if ctrl == "adrc" else PID(y)
        self.w_m = y[8:14].copy()        # controller side motor model (known lag)
        self.a_m = 1 - np.exp(-Ts / tau_mot)

    def step(self, y_meas, eff, zr=z_ref):
        M_des = self.C.update(y_meas, zr)
        w_cmd, Gi = allocate(M_des, eff, self.mode)
        if self.mode == "realloc":
            self.w_m[eff == 0] = 0       # known dead motor
        w_next = self.w_m + self.a_m * (w_cmd - self.w_m)
        M_exp = Gi @ ((self.w_m**2 + w_next**2) / 2)    # moment expected over this step
        self.w_m = w_next
        self.C.applied(M_exp)
        return w_cmd, M_des, M_exp

    def z3(self):
        return self.C.ax[0].z[2] if self.ctrl == "adrc" else 0.0


def run_case(ctrl="adrc", motor=fmot, eta=1.0, r=0.0, ang=0.0, mode="normal",
             tf=t_fault, te=t_end, mpl=mp, noise=0.0):
    # start in plain hover, payload is picked up (ramped in) over 0..t_ramp,
    # the controllers are never told about it
    y = h.start_state()
    L = Loop(ctrl, y, mode)
    rng = np.random.default_rng(0)
    data = []
    crashed = False
    n = int(round(te / Ts))
    for k in range(n + 1):
        t = k * Ts
        eff = np.ones(6)
        if motor > 0 and t >= tf:
            eff[motor-1] = 1 - eta
            if eta >= 1:
                y[8+motor-1] = 0
        y_meas = y.copy()
        if noise > 0:      # sensor noise on attitude (deg) and rates (deg/s, x5), altitude (cm)
            y_meas[0:3] += np.deg2rad(noise) * rng.standard_normal(3)
            y_meas[3:6] += np.deg2rad(5 * noise) * rng.standard_normal(3)
            y_meas[6] += 0.01 * noise * rng.standard_normal()
        w_cmd, M_des, M_exp = L.step(y_meas, eff)
        data.append(np.r_[t, y, M_des, M_exp, L.z3()])
        if abs(y[0]) > np.pi/2 or abs(y[1]) > np.pi/2 or y[6] < 0:
            crashed = True
            break
        if k < n:
            y = plant_step(y, w_cmd, eff, payload(r, ang, mpl * min(t / t_ramp, 1)))
    return np.array(data), crashed


def metrics(D, crashed, tf=t_fault):
    t = D[:, 0]
    a = t >= tf
    deg = np.rad2deg(D[:, 1:4])
    tilt = np.maximum(abs(deg[:, 0]), abs(deg[:, 1]))
    res = {"crashed": crashed,
           "pre_tilt_deg": tilt[~a].max(),
           "max_tilt_deg": tilt[a].max() if a.any() else np.nan,
           "final_tilt_deg": tilt[-1],
           "final_yaw_deg": deg[-1, 2],
           "min_z_m": D[a, 7].min() if a.any() else np.nan,
           "max_motor": D[:, 9:15].max()}
    bad = (tilt > 2) | (abs(D[:, 7] - z_ref) > 0.05)
    bad_t = t[bad & a]
    res["settle_s"] = (bad_t[-1] - tf) if len(bad_t) else 0.0
    # success = no crash and back near level hover at the end
    res["ok"] = (not crashed) and tilt[-1] < 5 and abs(D[-1, 7] - z_ref) < 0.2
    return res


def run_metrics(kw):
    D, c = run_case(**kw)
    return metrics(D, c, kw.get("tf", t_fault))


def max_offset(args):
    # bisection on payload radius for the largest offset that still recovers
    setup, ang = args
    kw = dict(SETUPS[setup], ang=ang)
    lo, hi = 0.0, 0.33
    if not run_metrics(dict(kw, r=lo))["ok"]:
        return 0.0
    for _ in range(7):
        mid = (lo + hi) / 2
        if run_metrics(dict(kw, r=mid))["ok"]:
            lo = mid
        else:
            hi = mid
    return lo


def dir_name(ang):
    d = (arm_ang - ang + 180) % 360 - 180
    i = np.argsort(abs(d))
    if abs(d[i[0]]) < 1:
        return "toward M%d" % (i[0] + 1)
    return "between M%d/M%d" % tuple(sorted((i[0] + 1, i[1] + 1)))


def write_csv(path, rows):
    keys = list(rows[0].keys())
    with open(path, "w") as f:
        f.write(",".join(keys) + "\n")
        for r in rows:
            f.write(",".join(("%.3f" % v) if isinstance(v, float) else str(v) for v in r.values()) + "\n")


# compared setups: what the controller knows about the fault
SETUPS = {"pid_blind": dict(ctrl="pid", mode="normal"),
          "pid_realloc": dict(ctrl="pid", mode="realloc"),
          "adrc_blind": dict(ctrl="adrc", mode="normal"),
          "adrc_realloc": dict(ctrl="adrc", mode="realloc")}
NAMES = {"pid_blind": "PID (fault unknown)",
         "pid_realloc": "PID + re-allocation (fault known - thesis)",
         "adrc_blind": "ADRC (fault unknown - proposed)",
         "adrc_realloc": "ADRC + re-allocation (fault known - reference)"}
STYLE = {"pid_blind": "C0:", "pid_realloc": "C0-", "adrc_blind": "C3-", "adrc_realloc": "C3:"}


def run_args(kw):
    return run_case(**kw)


if __name__ == "__main__":
    folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results_adrc")
    os.makedirs(folder, exist_ok=True)
    pool = Pool()
    print("arm directions (deg):", {i+1: round(a) for i, a in enumerate(arm_ang)})

    # 1) fault severity sweep, motor 1, no payload
    etas = [0.0, 0.25, 0.5, 0.75, 1.0]
    keys = [(s_, e) for s_ in SETUPS for e in etas]
    runs = dict(zip(keys, pool.map(run_args, [dict(SETUPS[s_], eta=e, mpl=0.0) for s_, e in keys])))
    rows = []
    fig, ax = plt.subplots(3, len(etas), figsize=(20, 9), sharex=True)
    for j, e in enumerate(etas):
        for s_ in SETUPS:
            D, cr = runs[(s_, e)]
            ax[0, j].plot(D[:, 0], np.rad2deg(D[:, 1]), STYLE[s_], label=NAMES[s_] + (" (crash)" if cr else ""))
            ax[1, j].plot(D[:, 0], D[:, 7], STYLE[s_])
            ax[2, j].plot(D[:, 0], np.rad2deg(D[:, 3]), STYLE[s_])
            rows.append({"setup": NAMES[s_], "eta": e, **metrics(D, cr)})
        ax[0, j].set_title("Motor 1 thrust loss %d%%" % (e * 100))
        ax[0, j].set_ylim(-60, 60)
        ax[0, j].legend(fontsize=6)
        ax[1, j].set_ylim(9.0, 10.5)
        ax[2, j].set_ylim(-60, 120)
        ax[2, j].set_xlabel("time (s)")
    ax[0, 0].set_ylabel("roll (deg)")
    ax[1, 0].set_ylabel("z (m)")
    ax[2, 0].set_ylabel("yaw (deg)")
    for a in ax.flatten():
        a.axvline(t_fault, color="r", lw=1)
        a.grid(True)
    fig.tight_layout()
    fig.savefig(os.path.join(folder, "fault_sweep.png"), dpi=110)
    plt.close(fig)
    write_csv(os.path.join(folder, "fault_sweep.csv"), rows)
    print("fault sweep done")

    # 2) payload placement, motor 1 fails completely, payload at 0.1 m in each direction
    angs = list(range(0, 360, 30))
    r_fix = 0.1
    cases = [("centre", 0.0, 0.0)] + [(dir_name(a), r_fix, float(a)) for a in angs]
    keys = [(c, s_) for c in cases for s_ in SETUPS]
    res = pool.map(run_metrics, [dict(SETUPS[s_], r=c[1], ang=c[2]) for c, s_ in keys])
    rows = [{"payload": c[0], "angle_deg": c[2], "r_m": c[1], "setup": NAMES[s_], **m_}
            for (c, s_), m_ in zip(keys, res)]
    write_csv(os.path.join(folder, "payload_locations.csv"), rows)
    print("payload locations done")

    # 3) largest payload offset that still recovers, per direction
    env = pool.map(max_offset, [(s_, float(a)) for s_ in SETUPS for a in angs])
    env = np.array(env).reshape(len(SETUPS), len(angs))
    with open(os.path.join(folder, "payload_envelope.csv"), "w") as f:
        f.write("angle_deg,direction," + ",".join(s_ + "_max_r_m" for s_ in SETUPS) + "\n")
        for j, a in enumerate(angs):
            f.write("%d,%s," % (a, dir_name(a)) + ",".join("%.3f" % v for v in env[:, j]) + "\n")
    th = np.deg2rad(np.r_[angs, angs[0]])
    fig = plt.figure(figsize=(7, 7))
    ax = fig.add_subplot(projection="polar")
    for k, s_ in enumerate(SETUPS):
        ax.plot(th, np.r_[env[k], env[k, 0]], STYLE[s_], marker="o", label=NAMES[s_])
    for i, a in enumerate(arm_ang):
        ax.annotate("M%d%s" % (i+1, " (failed)" if i+1 == fmot else ""), (np.deg2rad(a), 0.36),
                    ha="center", color="r" if i+1 == fmot else "k")
    ax.set_ylim(0, 0.35)
    ax.set_title("Largest payload offset (m) that still recovers\nmotor %d total failure, %.1f kg payload"
                 % (fmot, mp))
    ax.legend(loc="lower left", bbox_to_anchor=(-0.1, -0.15), fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(folder, "payload_envelope.png"), dpi=110)
    plt.close(fig)
    print("envelope done")

    # 4) time response for a few placements + LESO disturbance estimate
    show = [("centre", 0.0, 0.0), (dir_name(arm_ang[fmot-1]), r_fix, arm_ang[fmot-1]),
            (dir_name(arm_ang[1]), r_fix, arm_ang[1]), (dir_name(90.0), r_fix, 90.0)]
    keys = [(j, s_) for j in range(len(show)) for s_ in SETUPS]
    runs = dict(zip(keys, pool.map(run_args, [dict(SETUPS[s_], r=show[j][1], ang=show[j][2]) for j, s_ in keys])))
    fig, ax = plt.subplots(3, len(show), figsize=(20, 10), sharex=True)
    for j, (nm, r, a) in enumerate(show):
        for s_ in SETUPS:
            D, cr = runs[(j, s_)]
            tilt = np.rad2deg(np.maximum(abs(D[:, 1]), abs(D[:, 2])))
            ax[0, j].plot(D[:, 0], tilt, STYLE[s_], label=NAMES[s_] + (" (crash)" if cr else ""))
            ax[1, j].plot(D[:, 0], D[:, 7], STYLE[s_])
            if s_ == "adrc_blind":
                # true total roll disturbance = actual roll accel - b0 * expected torque
                pdot = np.gradient(D[:, 4], Ts)
                ax[2, j].plot(D[:, 0], pdot - D[:, 19] / Jxx, label="actual")
                ax[2, j].plot(D[:, 0], D[:, 23], "--", label="LESO estimate z3")
        ax[0, j].set_title("payload %s (r = %.2f m)" % (nm, r))
        ax[0, j].set_ylim(0, 60)
        ax[1, j].set_ylim(9.0, 10.5)
        ax[2, j].set_ylim(-100, 250)
        ax[0, j].legend(fontsize=6)
        ax[2, j].legend(fontsize=7)
        ax[2, j].set_xlabel("time (s)")
        for a_ in ax[:, j]:
            a_.axvline(t_fault, color="r", lw=1)
            a_.grid(True)
    ax[0, 0].set_ylabel("tilt (deg)")
    ax[1, 0].set_ylabel("z (m)")
    ax[2, 0].set_ylabel("roll disturbance (rad/s^2)\nADRC (fault unknown)")
    fig.tight_layout()
    fig.savefig(os.path.join(folder, "payload_response.png"), dpi=110)
    plt.close(fig)

    # 5) sensor noise: high observer bandwidth vs noisy measurements, motor 1 total failure
    rows = []
    for nz in [0.0, 0.3, 1.0]:
        for s_ in ("pid_realloc", "adrc_blind"):
            D, cr = run_case(**dict(SETUPS[s_], eta=1.0, mpl=0.0, noise=nz))
            hover = D[(D[:, 0] > 1) & (D[:, 0] < t_fault)]
            jitter = float(np.std(np.diff(hover[:, 9:15], axis=0)))
            rows.append({"setup": NAMES[s_], "noise_deg": nz, "motor_jitter_radps": jitter, **metrics(D, cr)})
    write_csv(os.path.join(folder, "noise.csv"), rows)
    print("done, results in", folder)
