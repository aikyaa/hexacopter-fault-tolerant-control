# Hexacopter fault tolerant control with ADRC (TD + LESO), following
# "Active Disturbance Rejection based approach for FTC of a Quadcopter" (ETAAV 2025)
# the controller is not told about the motor fault or the CG offset,
# the LESO estimates both as one total disturbance
import os
import numpy as np
from multiprocessing import Pool
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
try:
    from hexa_ftc_ros import hexa_ftc as h
except ImportError:
    import hexa_ftc as h

t_fault = 5.0      # after the CG offset is ramped in
t_end = 15.0
t_ramp = 2.0       # the CG offset is ramped in over this time (s), so hover starts trimmed
fault_motor = 1

# direction of each arm in the roll/pitch lever plane (deg)
lever = h.G[:2] / h.kf
arm_ang = np.rad2deg(np.arctan2(lever[1], lever[0])) % 360

# (wc, wo, r0) per axis: controller bandwidth, observer bandwidth, TD limit
adrc_gains = {"roll": (14.0, 50.0, 201.875), "pitch": (14.0, 50.0, 201.875),
              "yaw": (2.0, 10.0, 23.3527), "alt": (2.0, 10.0, 30.4)}


def fhan(x1, x2, r, h0):
    # Han's tracking function (eq 14-15)
    d = r * h0
    d0 = h0 * d
    y = x1 + h0 * x2
    a0 = np.sqrt(d * d + 8 * r * abs(y))
    if abs(y) > d0:
        a = x2 + (a0 - d) / 2 * np.sign(y)
    else:
        a = x2 + y / h0
    if abs(a) > d:
        return -r * np.sign(a)
    return -r * a / d


class ADRCAxis:
    def __init__(self, b0, wc, wo, r0, y0, ff=0.0):
        self.b0 = b0
        self.r0 = r0
        self.ff = ff
        self.kp = wc**2
        self.kd = 2 * wc
        self.b1 = 3 * wo
        self.b2 = 3 * wo**2
        self.b3 = wo**3
        self.z = np.array([y0, 0.0, 0.0])     # LESO states
        self.v = np.array([y0, 0.0])          # TD states
        self.u = 0.0

    def update(self, ref, y):
        # LESO (eq 21)
        e = self.z[0] - y
        z1, z2, z3 = self.z
        self.z = np.array([z1 + h.Ts * (z2 - self.b1 * e),
                           z2 + h.Ts * (z3 + self.ff + self.b0 * self.u - self.b2 * e),
                           z3 - h.Ts * self.b3 * e])
        # TD (eq 16)
        v1, v2 = self.v
        self.v = np.array([v1 + h.Ts * v2, v2 + h.Ts * fhan(v1 - ref, v2, self.r0, h.Ts)])
        # control law (eq 23)
        u0 = self.kp * (self.v[0] - self.z[0]) + self.kd * (self.v[1] - self.z[1])
        return (u0 - self.z[2] - self.ff) / self.b0


class ADRC:
    def __init__(self, y):
        k = adrc_gains
        self.axes = [ADRCAxis(1/h.Jxx, *k["roll"], y[0]),
                     ADRCAxis(1/h.Jyy, *k["pitch"], y[1]),
                     ADRCAxis(1/h.Jzz, *k["yaw"], y[2]),
                     ADRCAxis(1/h.m, *k["alt"], y[6], ff=-h.g)]

    def update(self, y, z_ref):
        refs = [0.0, 0.0, 0.0, z_ref]
        meas = [y[0], y[1], y[2], y[6]]
        M = np.array([self.axes[i].update(refs[i], meas[i]) for i in range(4)])
        M[3] = np.clip(M[3], 0, 6 * h.kf * h.w_max**2)
        return M

    def applied(self, M):
        for i in range(4):
            self.axes[i].u = M[i]


class Loop:
    # one control step: controller -> allocation -> motor model (for the LESO input)
    def __init__(self, ctrl, y):
        self.ctrl = ctrl
        self.C = ADRC(y) if ctrl == "adrc" else h.PID()
        self.w_model = y[8:14].copy()
        self.a = 1 - np.exp(-h.Ts / h.tau_mot)

    def step(self, y, eff_known, z_ref=h.z_ref):
        # eff_known: all ones = fault unknown, otherwise re-allocation
        M_des = self.C.update(y, z_ref)
        w_cmd = h.allocate(M_des, eff_known)
        self.w_model[eff_known == 0] = 0
        w_next = self.w_model + self.a * (w_cmd - self.w_model)
        M_exp = (h.G * eff_known) @ ((self.w_model**2 + w_next**2) / 2)
        self.w_model = w_next
        self.M_exp = M_exp
        if self.ctrl == "adrc":
            self.C.applied(M_exp)
        return w_cmd, M_des

    def roll_dist(self):
        # LESO estimate of the roll disturbance
        if self.ctrl == "adrc":
            return self.C.axes[0].z[2]
        return 0.0


def cg_offset(r, ang):
    # CG offset r (m) in direction ang (deg, same frame as arm_ang)
    return r * np.array([np.cos(np.deg2rad(ang)), np.sin(np.deg2rad(ang))])


def run_case(ctrl="adrc", ftc=False, eta=1.0, r=0.0, ang=0.0, noise=0.0):
    y = h.start_state()
    loop = Loop(ctrl, y)
    rng = np.random.default_rng(0)
    data = []
    crash = False
    n = int(round(t_end / h.Ts))
    for k in range(n + 1):
        t = k * h.Ts
        eff = h.motor_eff(fault_motor if t >= t_fault else 0, eta)
        y[8:14][eff == 0] = 0
        y_meas = y.copy()
        if noise > 0:
            y_meas[0:3] += np.deg2rad(noise) * rng.standard_normal(3)
            y_meas[3:6] += np.deg2rad(5 * noise) * rng.standard_normal(3)
            y_meas[6] += 0.01 * noise * rng.standard_normal()
        w_cmd, M_des = loop.step(y_meas, eff if ftc else np.ones(6))
        data.append(np.r_[t, y, loop.M_exp[0], loop.roll_dist()])
        if h.crashed(y):
            crash = True
            break
        if k < n:
            y = h.plant_step(y, w_cmd, eff, cg_offset(r, ang) * min(t / t_ramp, 1))
    return np.array(data), crash


def metrics(D, crash):
    t = D[:, 0]
    after = t >= t_fault
    deg = np.rad2deg(D[:, 1:4])
    tilt = np.maximum(abs(deg[:, 0]), abs(deg[:, 1]))
    res = {"crashed": crash,
           "pre_tilt_deg": tilt[~after].max(),
           "max_tilt_deg": tilt[after].max() if after.any() else np.nan,
           "final_tilt_deg": tilt[-1],
           "final_yaw_deg": deg[-1, 2],
           "min_z_m": D[after, 7].min() if after.any() else np.nan,
           "max_motor": D[:, 9:15].max()}
    bad = (tilt > 2) | (abs(D[:, 7] - h.z_ref) > 0.05)
    bad_t = t[bad & after]
    res["settle_s"] = bad_t[-1] - t_fault if len(bad_t) else 0.0
    # ok = no crash and back to level hover at the end
    res["ok"] = (not crash) and tilt[-1] < 5 and abs(D[-1, 7] - h.z_ref) < 0.2
    return res


def run_metrics(args):
    D, crash = run_case(**args)
    return metrics(D, crash)


def run_args(args):
    return run_case(**args)


def max_offset(args):
    # bisection for the largest CG offset that still recovers
    setup, ang = args
    lo, hi = 0.0, 0.2
    if not run_metrics(dict(SETUPS[setup], ang=ang, r=lo))["ok"]:
        return 0.0
    for i in range(9):
        mid = (lo + hi) / 2
        if run_metrics(dict(SETUPS[setup], ang=ang, r=mid))["ok"]:
            lo = mid
        else:
            hi = mid
    return lo


def dir_name(ang):
    d = (arm_ang - ang + 180) % 360 - 180
    i = np.argsort(abs(d))
    if abs(d[i[0]]) < 1:
        return "toward M%d" % (i[0] + 1)
    a, b = sorted([i[0] + 1, i[1] + 1])
    return "between M%d/M%d" % (a, b)


def write_csv(path, rows):
    with open(path, "w") as f:
        f.write(",".join(rows[0].keys()) + "\n")
        for r in rows:
            f.write(",".join(("%.3f" % v) if isinstance(v, float) else str(v) for v in r.values()) + "\n")


SETUPS = {"pid_blind": dict(ctrl="pid", ftc=False),
          "pid_realloc": dict(ctrl="pid", ftc=True),
          "adrc_blind": dict(ctrl="adrc", ftc=False),
          "adrc_realloc": dict(ctrl="adrc", ftc=True)}
NAMES = {"pid_blind": "PID (fault unknown)",
         "pid_realloc": "PID + re-allocation (fault known - thesis)",
         "adrc_blind": "ADRC (fault unknown - proposed)",
         "adrc_realloc": "ADRC + re-allocation (fault known - reference)"}
STYLE = {"pid_blind": "C0:", "pid_realloc": "C0-", "adrc_blind": "C3-", "adrc_realloc": "C3:"}


if __name__ == "__main__":
    folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results_adrc")
    os.makedirs(folder, exist_ok=True)
    pool = Pool()

    # 1) fault severity, motor 1, no CG offset
    etas = [0.0, 0.25, 0.5, 0.75, 1.0]
    jobs = []
    for e in etas:
        for s in SETUPS:
            jobs.append(dict(SETUPS[s], eta=e))
    runs = pool.map(run_args, jobs)
    rows = []
    fig, ax = plt.subplots(3, len(etas), figsize=(20, 9), sharex=True)
    i = 0
    for j, e in enumerate(etas):
        for s in SETUPS:
            D, crash = runs[i]
            i += 1
            ax[0, j].plot(D[:, 0], np.rad2deg(D[:, 1]), STYLE[s], label=NAMES[s] + (" (crash)" if crash else ""))
            ax[1, j].plot(D[:, 0], D[:, 7], STYLE[s])
            ax[2, j].plot(D[:, 0], np.rad2deg(D[:, 3]), STYLE[s])
            rows.append({"setup": NAMES[s], "eta": e, **metrics(D, crash)})
    for j, e in enumerate(etas):
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

    # 2) CG 1.5 cm off the centre in 12 directions, motor 1 fails completely
    angs = list(range(0, 360, 30))
    r_cg = 0.04
    cases = [("centre", 0.0, 0.0)] + [(dir_name(a), r_cg, float(a)) for a in angs]
    jobs = []
    labels = []
    for name, r, a in cases:
        for s in SETUPS:
            jobs.append(dict(SETUPS[s], r=r, ang=a))
            labels.append({"cg": name, "angle_deg": a, "r_cm": 100 * r, "setup": NAMES[s]})
    res = pool.map(run_metrics, jobs)
    rows = [{**labels[i], **res[i]} for i in range(len(jobs))]
    write_csv(os.path.join(folder, "cg_locations.csv"), rows)
    print("cg locations done")

    # 3) largest CG offset that still recovers, per direction
    env = pool.map(max_offset, [(s, float(a)) for s in SETUPS for a in angs])
    env = np.array(env).reshape(len(SETUPS), len(angs))
    env = 100 * env      # cm
    with open(os.path.join(folder, "cg_envelope.csv"), "w") as f:
        f.write("angle_deg,direction," + ",".join(s + "_max_cg_cm" for s in SETUPS) + "\n")
        for j, a in enumerate(angs):
            f.write("%d,%s," % (a, dir_name(a)) + ",".join("%.2f" % v for v in env[:, j]) + "\n")
    th = np.deg2rad(angs + [angs[0]])
    fig = plt.figure(figsize=(7, 7))
    ax = fig.add_subplot(projection="polar")
    for k, s in enumerate(SETUPS):
        ax.plot(th, np.r_[env[k], env[k, 0]], STYLE[s], marker="o", label=NAMES[s])
    # motor names on the angle axis
    ax.set_xticks(np.deg2rad(arm_ang))
    ax.set_xticklabels(["M%d%s" % (i+1, " (failed)" if i + 1 == fault_motor else "") for i in range(6)])
    ax.set_ylim(0, 20)
    ax.set_title("Largest CG offset (cm) that still recovers\nmotor %d total failure" % fault_motor)
    ax.legend(loc="lower left", bbox_to_anchor=(-0.1, -0.15), fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(folder, "cg_envelope.png"), dpi=110)
    plt.close(fig)
    print("envelope done")

    # 4) time response for a few CG places + LESO disturbance estimate
    show = [("centre", 0.0, 0.0), ("toward M1 (failed)", r_cg, 180.0),
            ("toward M2", r_cg, 0.0), ("between M4/M6", r_cg, 90.0)]
    jobs = []
    for name, r, a in show:
        for s in SETUPS:
            jobs.append(dict(SETUPS[s], r=r, ang=a))
    runs = pool.map(run_args, jobs)
    fig, ax = plt.subplots(3, len(show), figsize=(20, 10), sharex=True)
    i = 0
    for j, (name, r, a) in enumerate(show):
        for s in SETUPS:
            D, crash = runs[i]
            i += 1
            tilt = np.rad2deg(np.maximum(abs(D[:, 1]), abs(D[:, 2])))
            ax[0, j].plot(D[:, 0], tilt, STYLE[s], label=NAMES[s] + (" (crash)" if crash else ""))
            ax[1, j].plot(D[:, 0], D[:, 7], STYLE[s])
            if s == "adrc_blind":
                # actual roll disturbance = roll acceleration - expected torque / Jxx
                pdot = np.gradient(D[:, 4], h.Ts)
                ax[2, j].plot(D[:, 0], pdot - D[:, 15] / h.Jxx, label="actual")
                ax[2, j].plot(D[:, 0], D[:, 16], "--", label="LESO estimate z3")
        ax[0, j].set_title("CG %s (%.1f cm)" % (name, 100 * r))
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
    fig.savefig(os.path.join(folder, "cg_response.png"), dpi=110)
    plt.close(fig)

    # 5) sensor noise, motor 1 total failure
    rows = []
    for noise in [0.0, 0.3, 1.0]:
        for s in ["pid_realloc", "adrc_blind"]:
            D, crash = run_case(**SETUPS[s], noise=noise)
            hover = D[(D[:, 0] > 1) & (D[:, 0] < t_fault)]
            jitter = float(np.std(np.diff(hover[:, 9:15], axis=0)))
            rows.append({"setup": NAMES[s], "noise_deg": noise, "motor_jitter_radps": jitter, **metrics(D, crash)})
    write_csv(os.path.join(folder, "noise.csv"), rows)
    print("done, results in", folder)
