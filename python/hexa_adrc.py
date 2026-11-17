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
        M_act = h.G @ (eff * y[8:14]**2)
        data.append(np.r_[t, y, loop.M_exp[0], loop.roll_dist(), eff * h.kf * y[8:14]**2,
                          M_des[0], M_act[0], M_des, M_act])
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


def plot_full(D, crash, title, path):
    # every metric of one run: attitude, altitude, motors, thrust, torques, observer
    t = D[:, 0]
    deg = np.rad2deg
    tilt = deg(np.maximum(abs(D[:, 1]), abs(D[:, 2])))
    f_max = h.kf * h.w_max**2
    fig, ax = plt.subplots(4, 3, figsize=(18, 15), sharex=True)
    ax = ax.flatten()
    ax[0].plot(t, deg(D[:, 1]), label="roll")
    ax[0].plot(t, deg(D[:, 2]), label="pitch")
    ax[0].set_ylabel("angle (deg)")
    ax[1].plot(t, deg(D[:, 3]), label="yaw")
    ax[1].set_ylabel("yaw (deg)")
    ax[2].plot(t, tilt, label="tilt = max(|roll|, |pitch|)")
    ax[2].set_ylabel("tilt (deg)")
    ax[3].plot(t, deg(D[:, 4]), label="roll rate p")
    ax[3].plot(t, deg(D[:, 5]), label="pitch rate q")
    ax[3].plot(t, deg(D[:, 6]), label="yaw rate r")
    ax[3].set_ylabel("rate (deg/s)")
    ax[4].plot(t, D[:, 7], label="z")
    ax[4].axhline(h.z_ref, color="k", ls=":", lw=1, label="z ref")
    ax[4].set_ylabel("altitude (m)")
    ax[5].plot(t, D[:, 8], label="climb rate")
    ax[5].set_ylabel("climb rate (m/s)")
    for i in range(6):
        ax[6].plot(t, D[:, 9+i], label="M%d" % (i+1))
        ax[7].plot(t, D[:, 17+i], label="M%d" % (i+1))
    ax[6].axhline(h.w_max, color="k", ls="--", lw=1, label="max")
    ax[6].set_ylabel("motor speed (rad/s)")
    ax[7].axhline(f_max, color="k", ls="--", lw=1, label="max")
    ax[7].axhline(h.m * h.g / 6, color="k", ls=":", lw=1, label="W/6")
    ax[7].set_ylabel("motor thrust (N)")
    ax[8].plot(t, D[:, 32], label="total thrust")
    ax[8].plot(t, D[:, 28], "--", label="desired thrust")
    ax[8].axhline(h.m * h.g, color="k", ls=":", lw=1, label="weight")
    ax[8].axhline(6 * f_max, color="k", ls="--", lw=1, label="6-motor max")
    ax[8].set_ylabel("thrust (N)")
    ax[9].plot(t, D[:, 25], label="desired roll torque")
    ax[9].plot(t, D[:, 29], "--", label="produced by motors")
    ax[9].set_ylabel("roll torque (N m)")
    ax[10].plot(t, D[:, 26], label="desired pitch torque")
    ax[10].plot(t, D[:, 30], "--", label="produced by motors")
    ax[10].set_ylabel("pitch torque (N m)")
    if np.any(D[:, 16] != 0):
        pdot = np.gradient(D[:, 4], h.Ts)
        ax[11].plot(t, pdot - D[:, 15] / h.Jxx, label="actual roll disturbance")
        ax[11].plot(t, D[:, 16], "--", label="LESO estimate z3")
        ax[11].set_ylim(-150, 300)
    else:
        ax[11].text(0.5, 0.5, "no observer (PID)", ha="center", transform=ax[11].transAxes)
    ax[11].set_ylabel("roll disturbance (rad/s^2)")
    for a in ax:
        a.axvline(t_fault, color="r", lw=1)
        a.grid(True)
        a.legend(fontsize=7)
    for a in ax[9:]:
        a.set_xlabel("time (s)")
    m_ = metrics(D, crash)
    if crash:
        end = "CRASHED at t = %.2f s" % t[-1]
    else:
        end = "recovered, settle time %.2f s" % m_["settle_s"]
    fig.suptitle("%s\n%s | peak tilt %.1f deg | altitude dip %.0f cm | peak motor %.1f rad/s"
                 % (title, end, m_["max_tilt_deg"], 100 * (h.z_ref - m_["min_z_m"]), m_["max_motor"]))
    fig.tight_layout()
    fig.savefig(path, dpi=80)
    plt.close(fig)


def plot_full_args(args):
    plot_full(*args)


SETUPS = {"pid_blind": dict(ctrl="pid", ftc=False),
          "pid_realloc": dict(ctrl="pid", ftc=True),
          "adrc_blind": dict(ctrl="adrc", ftc=False),
          "adrc_realloc": dict(ctrl="adrc", ftc=True)}
NAMES = {"pid_blind": "PID (fault unknown)",
         "pid_realloc": "PID + re-allocation (fault known - thesis)",
         "adrc_blind": "ADRC (fault unknown - proposed)",
         "adrc_realloc": "ADRC + re-allocation (fault known - reference)"}
STYLE = {"pid_blind": "C0:", "pid_realloc": "C0-", "adrc_blind": "C3-", "adrc_realloc": "C3:"}
FILE = {"pid_blind": "PID_fault_unknown", "pid_realloc": "PID_realloc_fault_known",
        "adrc_blind": "ADRC_fault_unknown", "adrc_realloc": "ADRC_realloc_fault_known"}


def case_file(setup, eta=None, r=None, ang=None):
    # e.g. M1_loss_50pct__ADRC_fault_unknown.png, CG_4cm_between_M1-M4__PID_realloc_fault_known.png
    # (every CG case has motor 1 failing at t_fault)
    if eta is not None:
        name = "no_fault" if eta == 0 else "M1_loss_%dpct" % round(100 * eta)
    elif r == 0:
        name = "CG_centre"
    else:
        name = "CG_%dcm_%s" % (round(100 * r), dir_name(ang).replace(" ", "_").replace("/", "-"))
    return name + "__" + FILE[setup] + ".png"


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
    cases_dir = os.path.join(folder, "cases")
    os.makedirs(cases_dir, exist_ok=True)
    figs = []
    i = 0
    for e in etas:
        for s in SETUPS:
            D, crash = runs[i]
            i += 1
            title = "%s, motor 1 loses %d%% at t = %.0f s" % (NAMES[s], 100 * e, t_fault)
            figs.append((D, crash, title, os.path.join(cases_dir, case_file(s, eta=e))))
    pool.map(plot_full_args, figs)

    # motor thrusts at total loss of motor 1
    f_max = h.kf * h.w_max**2
    fig, ax = plt.subplots(1, len(SETUPS), figsize=(20, 4.5), sharey=True)
    for k, s in enumerate(SETUPS):
        D, crash = runs[len(SETUPS) * (len(etas) - 1) + k]
        for i in range(6):
            ax[k].plot(D[:, 0], D[:, 17+i], label="M%d" % (i+1))
        ax[k].axhline(f_max, color="k", ls="--", lw=1, label="motor max")
        ax[k].axhline(h.m * h.g / 6, color="k", ls=":", lw=1, label="hover share (W/6)")
        ax[k].axvline(t_fault, color="r", lw=1)
        ax[k].set_title(NAMES[s] + (" (crash)" if crash else ""), fontsize=9)
        ax[k].set_xlabel("time (s)")
        ax[k].grid(True)
    ax[0].set_ylabel("motor thrust (N)")
    ax[0].legend(fontsize=7, ncol=2)
    fig.suptitle("Motor thrusts, motor 1 total failure at t = %.0f s (T/W = %.0f)" % (t_fault, h.twr))
    fig.tight_layout()
    fig.savefig(os.path.join(folder, "fault_thrust.png"), dpi=110)
    plt.close(fig)

    # peak tilt, altitude dip and settle time against fault size
    fig, ax = plt.subplots(1, 3, figsize=(18, 4.5))
    for s in SETUPS:
        loss, tilt, dip, settle = [], [], [], []
        for r in rows:
            if r["setup"] == NAMES[s] and r["eta"] > 0 and not r["crashed"]:
                loss.append(100 * r["eta"])
                tilt.append(r["max_tilt_deg"])
                dip.append(100 * (h.z_ref - r["min_z_m"]))
                settle.append(r["settle_s"])
        if loss:
            ax[0].plot(loss, tilt, STYLE[s], marker="o", label=NAMES[s])
            ax[1].plot(loss, dip, STYLE[s], marker="o")
            ax[2].plot(loss, settle, STYLE[s], marker="o")
    ax[0].set_ylabel("peak tilt after the fault (deg)")
    ax[1].set_ylabel("altitude dip (cm)")
    ax[2].set_ylabel("settle time (s)")
    for a in ax:
        a.set_xlabel("motor 1 thrust loss (%)")
        a.set_xticks([25, 50, 75, 100])
        a.grid(True)
    ax[0].legend(fontsize=7, title="PID (fault unknown) crashes at every level", title_fontsize=7)
    fig.tight_layout()
    fig.savefig(os.path.join(folder, "fault_metrics.png"), dpi=110)
    plt.close(fig)

    # desired vs produced roll torque at total loss: what the healthy mixer wastes
    fig, ax = plt.subplots(1, 2, figsize=(14, 4.5), sharey=True)
    for k, s in enumerate(["adrc_blind", "adrc_realloc"]):
        D, crash = runs[len(SETUPS) * (len(etas) - 1) + list(SETUPS).index(s)]
        ax[k].plot(D[:, 0], D[:, 23], label="desired roll torque (controller)")
        ax[k].plot(D[:, 0], D[:, 24], "--", label="roll torque the motors produce")
        ax[k].axvline(t_fault, color="r", lw=1)
        ax[k].set_xlim(4, 9)
        ax[k].set_title(NAMES[s], fontsize=10)
        ax[k].set_xlabel("time (s)")
        ax[k].grid(True)
        ax[k].legend(fontsize=8)
    ax[0].set_ylabel("roll torque (N m)")
    fig.suptitle("Motor 1 total failure: desired vs produced roll torque")
    fig.tight_layout()
    fig.savefig(os.path.join(folder, "torque_tracking.png"), dpi=110)
    plt.close(fig)
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
    cg_runs = pool.map(run_args, jobs)
    rows = [{**labels[i], **metrics(*cg_runs[i])} for i in range(len(jobs))]
    write_csv(os.path.join(folder, "cg_locations.csv"), rows)
    figs = []
    for i in range(len(jobs)):
        D, crash = cg_runs[i]
        setup = list(SETUPS)[i % len(SETUPS)]
        a = jobs[i]["ang"]
        title = "%s, CG %.0f cm %s, motor 1 fails at t = %.0f s" % (
            labels[i]["setup"], 100 * jobs[i]["r"], labels[i]["cg"], t_fault)
        figs.append((D, crash, title, os.path.join(cases_dir, case_file(setup, r=jobs[i]["r"], ang=a))))
    pool.map(plot_full_args, figs)
    del cg_runs, figs
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
    fig, ax = plt.subplots(4, len(show), figsize=(20, 13), sharex=True)
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
                for m_ in range(6):
                    ax[3, j].plot(D[:, 0], D[:, 17+m_], label="M%d" % (m_+1))
                ax[3, j].axhline(f_max, color="k", ls="--", lw=1, label="motor max")
        ax[0, j].set_title("CG %s (%.1f cm)" % (name, 100 * r))
        ax[0, j].set_ylim(0, 60)
        ax[1, j].set_ylim(9.0, 10.5)
        ax[2, j].set_ylim(-100, 250)
        ax[0, j].legend(fontsize=6)
        ax[2, j].legend(fontsize=7)
        ax[3, j].legend(fontsize=6, ncol=2)
        ax[3, j].set_ylim(0, 1.1 * f_max)
        ax[3, j].set_xlabel("time (s)")
        for a_ in ax[:, j]:
            a_.axvline(t_fault, color="r", lw=1)
            a_.grid(True)
    ax[0, 0].set_ylabel("tilt (deg)")
    ax[1, 0].set_ylabel("z (m)")
    ax[2, 0].set_ylabel("roll disturbance (rad/s^2)\nADRC (fault unknown)")
    ax[3, 0].set_ylabel("motor thrust (N)\nADRC (fault unknown)")
    fig.tight_layout()
    fig.savefig(os.path.join(folder, "cg_response.png"), dpi=110)
    plt.close(fig)

    # 5) PID vs ADRC as the CG offset grows: without a fault, and with motor 1 failing
    offsets = [0.001, 0.002, 0.003, 0.005, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.08, 0.1, 0.12, 0.15]
    panels = [("no motor fault, CG toward M2", 0.0, 0.0, ["pid_realloc", "adrc_blind"]),
              ("motor 1 fails, CG toward M1 (failed side)", 1.0, 180.0, ["pid_realloc", "adrc_blind", "adrc_realloc"]),
              ("motor 1 fails, CG toward M2 (opposite side)", 1.0, 0.0, ["pid_realloc", "adrc_blind", "adrc_realloc"])]
    jobs = []
    keys = []
    for title, eta, a, setups in panels:
        for s in setups:
            for r in offsets:
                jobs.append(dict(SETUPS[s], eta=eta, r=r, ang=a))
                keys.append((title, s, r))
    res = pool.map(run_metrics, jobs)
    rows = []
    fig, ax = plt.subplots(1, 3, figsize=(18, 5), sharey=True)
    for k, (title, eta, a, setups) in enumerate(panels):
        for s in setups:
            tilt = []
            for i in range(len(jobs)):
                if keys[i][0] == title and keys[i][1] == s:
                    m_ = res[i]
                    peak = m_["pre_tilt_deg"]
                    if not np.isnan(m_["max_tilt_deg"]):
                        peak = max(peak, m_["max_tilt_deg"])
                    tilt.append(min(peak, 90))
                    case = "no fault" if eta == 0 else "M1 fails"
                    rows.append({"case": case, "cg_angle_deg": a, "cg_cm": 100 * keys[i][2], "setup": NAMES[s], **m_})
            label = "PID" if eta == 0 and s == "pid_realloc" else ("ADRC" if eta == 0 else NAMES[s])
            ax[k].semilogx(100 * np.array(offsets), tilt, STYLE[s], marker="o", label=label)
        ax[k].axhline(90, color="k", lw=1)
        ax[k].text(0.11, 84, "crash", fontsize=8)
        ax[k].set_title(title, fontsize=10)
        ax[k].set_xlabel("CG offset (cm)")
        ax[k].grid(True, which="both")
        ax[k].legend(fontsize=7)
    ax[0].set_ylabel("peak tilt over the run (deg)")
    ax[0].set_ylim(0, 95)
    fig.tight_layout()
    fig.savefig(os.path.join(folder, "cg_pid_vs_adrc.png"), dpi=110)
    plt.close(fig)
    write_csv(os.path.join(folder, "cg_pid_vs_adrc.csv"), rows)
    print("pid vs adrc done")

    # 6) sensor noise, motor 1 total failure
    rows = []
    noise_runs = {}
    for noise in [0.0, 0.3, 1.0]:
        for s in ["pid_realloc", "adrc_blind"]:
            D, crash = run_case(**SETUPS[s], noise=noise)
            noise_runs[(noise, s)] = (D, crash)
            hover = D[(D[:, 0] > 1) & (D[:, 0] < t_fault)]
            jitter = float(np.std(np.diff(hover[:, 9:15], axis=0)))
            rows.append({"setup": NAMES[s], "noise_deg": noise, "motor_jitter_radps": jitter, **metrics(D, crash)})
    write_csv(os.path.join(folder, "noise.csv"), rows)
    fig, ax = plt.subplots(2, 2, figsize=(16, 8), sharex=True)
    for j, noise in enumerate([0.3, 1.0]):
        for s in ["pid_realloc", "adrc_blind"]:
            D, crash = noise_runs[(noise, s)]
            tilt = np.rad2deg(np.maximum(abs(D[:, 1]), abs(D[:, 2])))
            ax[0, j].plot(D[:, 0], tilt, STYLE[s], label=NAMES[s] + (" (crash)" if crash else ""))
            ax[1, j].plot(D[:, 0], D[:, 11], STYLE[s], lw=0.8, label=NAMES[s])
        ax[1, j].axhline(h.w_max, color="k", ls="--", lw=1, label="motor max")
        ax[0, j].set_title("attitude noise %.1f deg, motor 1 total failure" % noise)
        ax[0, j].set_ylim(0, 60)
        ax[1, j].set_xlabel("time (s)")
        ax[0, j].legend(fontsize=7)
        ax[1, j].legend(fontsize=7)
        for a in ax[:, j]:
            a.axvline(t_fault, color="r", lw=1)
            a.grid(True)
    ax[0, 0].set_ylabel("tilt (deg)")
    ax[1, 0].set_ylabel("motor 3 speed (rad/s)")
    fig.tight_layout()
    fig.savefig(os.path.join(folder, "noise_response.png"), dpi=110)
    plt.close(fig)
    print("done, results in", folder)
