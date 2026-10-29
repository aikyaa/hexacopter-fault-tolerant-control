# hexacopter plant: dynamics + motor fault (total or partial) + off-centre payload
# + disturbance + rviz markers
# rviz scene: drone with FTC, a "no FTC" ghost with the same fault, thrust arrows
# and a live altitude chart, with slow motion around the fault
# the controller is never told about the payload
import os
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float64MultiArray, Float64, Int32
from geometry_msgs.msg import TransformStamped, Vector3, Point
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster
from hexa_ftc_ros import hexa_ftc as h
from hexa_ftc_ros import hexa_adrc as ha

# motor positions (x, y) from the G matrix, x forward y left
s3 = np.sqrt(3) / 2
motor_xy = h.l * np.array([[0, -1], [0, 1], [s3, 0.5], [-s3, -0.5], [s3, -0.5], [-s3, 0.5]])
spin_dir = [1, -1, 1, -1, -1, 1]    # sign of the yaw torque column in G

z_ref = h.z_ref
y_ftc, y_ghost = 0.9, -0.9          # side by side, seen from +x the ftc drone is on the left
chart_x = -1.6                      # chart panels behind the drones
chart_y0, chart_y1 = 2.6, -2.6      # time axis (left to right seen from +x)
# two panels like thesis fig 4.2 / 4.5: (name, bottom z, top z, value lo, value hi, ticks, unit)
panels = [("roll", z_ref + 0.05, z_ref + 1.25, -60.0, 60.0, [-45, 0, 45], "deg"),
          ("altitude", z_ref - 1.55, z_ref - 0.35, z_ref - 1.5, z_ref + 0.25, [z_ref - 1, z_ref], "m")]
chart_ztop = panels[0][2]
slow_window = (-0.1, 1.5)           # slow motion from t_fault-0.1 to t_fault+1.5

# colours
C_BODY = (0.13, 0.14, 0.16, 1.0)
C_ARM = (0.22, 0.23, 0.26, 1.0)
C_HUB = (0.55, 0.57, 0.60, 1.0)
C_THRUST = (0.20, 0.75, 1.00, 0.9)
C_DEAD = (1.00, 0.15, 0.10, 1.0)
C_OFF = (1.00, 0.65, 0.10, 1.0)     # healthy motor that the allocation switched off
C_FTC = (0.25, 0.90, 0.45, 1.0)
C_GHOST = (1.00, 0.30, 0.25, 1.0)
C_TEXT = (0.92, 0.93, 0.95, 1.0)
C_DIM = (0.60, 0.62, 0.66, 1.0)
C_PAYLOAD = (0.95, 0.80, 0.20, 1.0)


def euler_to_quat(phi, th, psi):
    cr, sr = np.cos(phi/2), np.sin(phi/2)
    cp, sp = np.cos(th/2), np.sin(th/2)
    cy, sy = np.cos(psi/2), np.sin(psi/2)
    return [sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy, cr*cp*cy + sr*sp*sy]


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [aw*bx + ax*bw + ay*bz - az*by, aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw, aw*bw - ax*bx - ay*by - az*bz]


def horizontal_quat(ang):
    # rotates a cylinder (z axis) to lie flat pointing at angle ang
    qy = [0.0, np.sin(np.pi/4), 0.0, np.cos(np.pi/4)]
    qz = [0.0, 0.0, np.sin(ang/2), np.cos(ang/2)]
    return quat_mul(qz, qy)


def mk(frame, ns, i, typ, now, color, scale, pos=(0, 0, 0), q=(0, 0, 0, 1)):
    m = Marker()
    m.header.frame_id = frame
    m.header.stamp = now
    m.ns, m.id, m.type = ns, i, typ
    m.pose.position.x, m.pose.position.y, m.pose.position.z = [float(v) for v in pos]
    m.pose.orientation.x, m.pose.orientation.y = float(q[0]), float(q[1])
    m.pose.orientation.z, m.pose.orientation.w = float(q[2]), float(q[3])
    m.scale.x, m.scale.y, m.scale.z = [float(v) for v in scale]
    m.color.r, m.color.g, m.color.b, m.color.a = [float(v) for v in color]
    return m


def text(frame, ns, i, now, s, pos, size, color=C_TEXT):
    m = mk(frame, ns, i, Marker.TEXT_VIEW_FACING, now, color, (0, 0, size), pos)
    m.text = s
    return m


def line(frame, ns, i, now, pts, color, width, typ=Marker.LINE_STRIP):
    m = mk(frame, ns, i, typ, now, color, (width, 0, 0))
    m.points = [Point(x=float(a), y=float(b), z=float(c)) for a, b, c in pts]
    return m


class Plant(Node):
    def __init__(self):
        super().__init__("plant_node")
        self.declare_parameter("fault_motor", 0)
        self.declare_parameter("fault_time", h.t_fault)
        self.declare_parameter("disturbance", False)
        self.declare_parameter("t_end", h.t_end)
        self.declare_parameter("out_dir", "results_ros")
        self.declare_parameter("wait_viewer", False)   # hold sim until rviz is listening
        self.declare_parameter("start_delay", 3.0)     # show the start this long (s) before sim runs
        self.declare_parameter("hold_end", False)      # keep showing final state instead of quitting
        self.declare_parameter("slowmo", 0.15)         # playback speed around the fault (1 = off)
        self.declare_parameter("ghost", True)          # show a drone with the same fault but no FTC
        self.declare_parameter("fault_eta", 1.0)       # thrust loss of the failed motor (1 = total)
        self.declare_parameter("payload_mass", 0.0)    # kg, picked up over the first ha.t_ramp s
        self.declare_parameter("payload_r", 0.0)       # m from the centre
        self.declare_parameter("payload_angle", 0.0)   # deg, 0 = toward M2, 180 = toward M1
        self.declare_parameter("label", "WITH FTC (re-allocation)")   # title of the main drone
        self.declare_parameter("tag", "")              # appended to the result file names
        self.fault_motor = self.get_parameter("fault_motor").value
        self.eta = float(np.clip(self.get_parameter("fault_eta").value, 0.0, 1.0))
        self.pl_m = self.get_parameter("payload_mass").value
        self.pl_r = self.get_parameter("payload_r").value
        self.pl_ang = self.get_parameter("payload_angle").value
        self.label = self.get_parameter("label").value
        self.tag = self.get_parameter("tag").value
        self.t_fault = self.get_parameter("fault_time").value
        self.dist_on = self.get_parameter("disturbance").value
        self.t_end = self.get_parameter("t_end").value
        self.out_dir = self.get_parameter("out_dir").value
        h.t_fault = self.t_fault             # plot_case marks the fault at h.t_fault
        self.wait_viewer = self.get_parameter("wait_viewer").value
        self.start_ticks = int(self.get_parameter("start_delay").value / h.Ts)
        self.hold_end = self.get_parameter("hold_end").value
        slowmo = self.get_parameter("slowmo").value
        # slow motion only makes sense when someone is watching
        self.slow_k = max(1, int(round(1.0 / slowmo))) if (self.wait_viewer and slowmo > 0) else 1
        self.ghost_on = self.get_parameter("ghost").value and self.fault_motor > 0
        self.viewer_seen = not self.wait_viewer
        self.wait_count = 0
        self.sub_tick = 0

        self.y = h.start_state()         # hover at z_ref
        self.t = 0.0
        self.k = 0
        self.w_cmd = None
        self.M_des = np.zeros(4)
        self.data = []
        self.done = False
        self.end_count = 0
        self.finished = False
        self.n = int(round(self.t_end / h.Ts))
        self.f = 0
        self.sent = False                # state k sent, waiting for its reply
        self.t_sent = 0.0
        self.wait_since = None

        # ghost: same plant, same fault, normal mixer (no re-allocation)
        self.yg = self.y.copy()
        self.g_ctrl = None
        self.g_down = False
        self.g_crash_t = None
        self.g_ground_t = None
        # chart history (t, z, roll)
        self.hist_ftc = [(0.0, self.y[6], 0.0)]
        self.hist_ghost = [(0.0, self.y[6], 0.0)]
        self.z_pre = None
        self.min_after = None

        self.state_pub = self.create_publisher(Float64MultiArray, "/state", 10)
        self.fault_pub = self.create_publisher(Int32, "/fault_motor", 10)
        self.ang_pub = self.create_publisher(Vector3, "/angles_deg", 10)
        self.alt_pub = self.create_publisher(Float64, "/altitude", 10)
        self.mot_pub = self.create_publisher(Float64MultiArray, "/motor_speed", 10)
        self.mark_pub = self.create_publisher(MarkerArray, "/hexa_markers", 10)
        self.tf_pub = TransformBroadcaster(self)
        self.create_subscription(Float64MultiArray, "/motor_cmd", self.cmd_cb, 10)
        self.timer = self.create_timer(h.Ts, self.step)
        self.viz_timer = self.create_timer(1.0 / 20, lambda: self.publish_viz(self.fault_now()))   # rviz at 20 Hz
        name = "motor %d loses %.0f%% at %.1f s" % (self.fault_motor, 100 * self.eta, self.t_fault) \
            if self.fault_motor > 0 else "no fault"
        if self.pl_m > 0:
            name += ", payload %.2f kg at %.2f m / %.0f deg" % (self.pl_m, self.pl_r, self.pl_ang)
            if self.fault_motor > 0 and self.t_fault < ha.t_ramp + 1:
                self.get_logger().warn("payload is still being picked up at the fault, "
                                       "use fault_time >= %.0f" % (ha.t_ramp + 3))
        self.get_logger().info("plant started: " + name + (", disturbance on" if self.dist_on else ""))

    def eff_now(self):
        eff = np.ones(6)
        f = self.fault_now()
        if f > 0:
            eff[f-1] = 1 - self.eta
        return eff

    def payload_now(self):
        # payload ramps in over ha.t_ramp seconds (picked up)
        return ha.payload(self.pl_r, self.pl_ang, self.pl_m * min(self.t / ha.t_ramp, 1.0))

    def cmd_cb(self, msg):
        # /motor_cmd = [w (6), M (4), k], only use the reply to the state we just sent
        if self.sent and int(round(msg.data[10])) == self.k:
            self.w_cmd = np.array(msg.data[:6])
            self.M_des = np.array(msg.data[6:10])

    def send_state(self):
        self.f = self.fault_now()
        if self.f > 0 and self.eta >= 1:
            self.y[8+self.f-1] = 0       # failed motor stops at once
        self.w_cmd = None
        self.publish_state(self.f)
        self.sent = True
        self.t_sent = time.time()

    def fault_now(self):
        if self.fault_motor > 0 and self.t >= self.t_fault:
            return self.fault_motor
        return 0

    def in_slowmo(self):
        return self.fault_motor > 0 and \
            self.t_fault + slow_window[0] <= self.t <= self.t_fault + slow_window[1]

    def step(self):
        if self.done:
            self.end_count += 1
            if self.end_count > 200 and not self.hold_end:   # keep showing 2 s, then quit
                self.finished = True
            return
        if not self.viewer_seen:
            if self.mark_pub.get_subscription_count() == 0:
                return                   # wait until rviz is running
            self.viewer_seen = True
            self.get_logger().info("rviz connected, starting in %.1f s" % (self.start_ticks * h.Ts))
        if self.wait_viewer and self.wait_count < self.start_ticks:
            self.wait_count += 1
            return
        if self.state_pub.get_subscription_count() == 0 or self.count_publishers("/motor_cmd") == 0:
            if self.wait_since is None:
                self.wait_since = time.time()
            if time.time() - self.wait_since > 2.0:
                self.get_logger().warn("ftc_node not found yet (on WSL try: ros2 daemon start)",
                                       throttle_duration_sec=5.0)
            return                       # wait until ftc_node is running
        if not self.sent:                # lockstep: send state k, then wait for its reply
            self.send_state()
            return
        if self.w_cmd is None:           # no reply for step k yet
            if time.time() - self.t_sent > 1.0:
                self.get_logger().warn("no motor command for step %d after 1 s, sending the state again" % self.k)
                self.publish_state(self.f)
                self.t_sent = time.time()
            return
        if self.slow_k > 1 and self.in_slowmo():
            self.sub_tick += 1           # slow motion: only advance every slow_k ticks
            if self.sub_tick < self.slow_k:
                return
            self.sub_tick = 0
        f = self.f
        eff = self.eff_now()
        self.data.append(np.r_[self.t, self.y, self.M_des, h.G @ (eff * self.y[8:14]**2)])
        crash = abs(self.y[0]) > np.pi/2 or abs(self.y[1]) > np.pi/2 or self.y[6] < 0
        if crash or self.k >= self.n:
            if crash:
                self.get_logger().warn("CRASH at t=%.2f s" % self.t)
            self.save()
            self.done = True
            if self.hold_end:
                self.get_logger().info("simulation finished, close rviz to exit")
            return
        P = self.payload_now()
        d = h.dist(self.t, self.dist_on)
        self.y = ha.plant_step(self.y, self.w_cmd, eff, P, d)
        if self.ghost_on:
            self.ghost_step(f, eff, P, d)
        self.k += 1
        self.t = self.k * h.Ts
        self.record()
        self.send_state()                # next state right away

    def ghost_step(self, f, eff, P, d):
        # ghost: PID cascade, healthy mixer, fault unknown
        if f == 0:
            self.yg = self.y.copy()      # identical to the real drone until the fault
            return
        if self.g_down:
            return
        if self.g_ctrl is None:
            self.g_ctrl = ha.Loop("pid", self.yg, "normal")
        if self.eta >= 1:
            self.yg[8+f-1] = 0
        w, _, _ = self.g_ctrl.step(self.yg, eff, z_ref)
        self.yg = ha.plant_step(self.yg, w, eff, P, d)
        if self.g_crash_t is None and (abs(self.yg[0]) > np.pi/2 or abs(self.yg[1]) > np.pi/2):
            self.g_crash_t = self.t
            self.get_logger().warn("no-FTC ghost flipped over at t=%.2f s" % self.t)
        if self.yg[6] <= 0.04:           # hit the ground: lie flat (or upside down)
            self.g_down = True
            self.g_ground_t = self.t
            self.yg[0] = np.pi if np.cos(self.yg[0]) < 0 else 0.0
            self.yg[1] = 0.0
            self.yg[3:6] = 0
            self.yg[6] = 0.04
            self.yg[7] = 0
            self.yg[8:14] = 0

    def record(self):
        self.hist_ftc.append((self.t, self.y[6], np.rad2deg(self.y[0])))
        if self.ghost_on:
            self.hist_ghost.append((self.t, self.yg[6], np.rad2deg(np.arctan2(np.sin(self.yg[0]), np.cos(self.yg[0])))))
        if self.fault_motor > 0:
            if self.t < self.t_fault:
                self.z_pre = self.y[6]
            else:
                self.min_after = self.y[6] if self.min_after is None else min(self.min_after, self.y[6])

    def publish_state(self, f):
        msg = Int32()
        msg.data = int(f)
        self.fault_pub.publish(msg)          # fault info (no FDI, known at once)
        msg = Float64MultiArray()
        msg.data = [float(v) for v in self.y] + [float(self.k), float(f), self.eta]   # state, step, fault
        self.state_pub.publish(msg)
        a = np.rad2deg(self.y[:3])
        self.ang_pub.publish(Vector3(x=float(a[0]), y=float(a[1]), z=float(a[2])))
        self.alt_pub.publish(Float64(data=float(self.y[6])))
        msg = Float64MultiArray()
        msg.data = [float(v) for v in self.y[8:14]]
        self.mot_pub.publish(msg)

    def send_tf(self, y, frame, y_off, now):
        tr = TransformStamped()
        tr.header.stamp = now
        tr.header.frame_id = "world"
        tr.child_frame_id = frame
        tr.transform.translation.y = y_off
        tr.transform.translation.z = float(max(y[6], 0.0))
        q = euler_to_quat(y[0], y[1], y[2])
        tr.transform.rotation.x, tr.transform.rotation.y = float(q[0]), float(q[1])
        tr.transform.rotation.z, tr.transform.rotation.w = float(q[2]), float(q[3])
        self.tf_pub.sendTransform(tr)

    def publish_viz(self, f):
        now = self.get_clock().now().to_msg()
        self.send_tf(self.y, "base_link", y_ftc, now)
        if self.ghost_on:
            self.send_tf(self.yg, "ghost_link", y_ghost, now)
        if self.mark_pub.get_subscription_count() == 0:
            return                           # nobody watching (headless), tf is enough
        t0 = rclpy.time.Time().to_msg()      # stamp 0 = use latest tf
        self.viz_count = getattr(self, "viz_count", 0) + 1
        static = self.viz_count % 20 == 1    # panels, grid, labels: once a second is enough
        ma = MarkerArray()
        ma.markers += self.drone_markers("base_link", "ftc", self.y, f, t0, ghost=False)
        if self.ghost_on:
            ma.markers += self.drone_markers("ghost_link", "ghost", self.yg, f, t0, ghost=True)
        ma.markers += self.scene_markers(f, t0, static)
        self.mark_pub.publish(ma)

    def drone_markers(self, frame, ns, y, f, now, ghost):
        out = []
        tint = 0.55 if ghost else 1.0        # ghost is drawn see-through
        a = lambda c: (c[0], c[1], c[2], c[3] * tint)
        out.append(mk(frame, ns + "_body", 0, Marker.CYLINDER, now, a(C_BODY), (0.20, 0.20, 0.06)))
        out.append(mk(frame, ns + "_body", 1, Marker.CYLINDER, now,
                      a(C_GHOST if ghost else C_FTC), (0.12, 0.12, 0.065)))   # coloured top cap
        out.append(mk(frame, ns + "_body", 2, Marker.ARROW, now, a((1.0, 1.0, 1.0, 1.0)),
                      (0.16, 0.025, 0.025), (0.06, 0, 0.04)))                  # nose (x forward)
        hover_thrust = h.kf * h.w_hover**2
        for i in range(6):
            x, yy = motor_xy[i]
            ang = np.arctan2(yy, x)
            w = y[8+i]
            dead = f == i + 1
            off = (not dead) and f > 0 and w < 5
            out.append(mk(frame, ns + "_arm", i, Marker.CYLINDER, now, a(C_ARM),
                          (0.022, 0.022, h.l), (x/2, yy/2, 0), horizontal_quat(ang)))
            out.append(mk(frame, ns + "_hub", i, Marker.CYLINDER, now, a(C_HUB),
                          (0.045, 0.045, 0.05), (x, yy, 0.01)))
            if dead:
                pc = C_DEAD
            elif off:
                pc = C_OFF
            else:
                s = np.clip(w / h.w_max, 0, 1)   # brighter = faster
                pc = (0.35 + 0.3*s, 0.55 + 0.35*s, 0.75 + 0.25*s, 0.35)
            out.append(mk(frame, ns + "_prop", i, Marker.CYLINDER, now,
                          a((pc[0], pc[1], pc[2], 0.55 if (dead or off) else pc[3])),
                          (0.26, 0.26, 0.006), (x, yy, 0.04)))
            # thrust arrow, 0.5 m = hover thrust of one motor
            L = 0.5 * h.kf * w**2 / hover_thrust
            arr = mk(frame, ns + "_thrust", i, Marker.ARROW, now,
                     a(C_THRUST) if L > 0.01 else (0, 0, 0, 0), (0.018, 0.04, 0.05))
            arr.points = [Point(x=float(x), y=float(yy), z=0.05),
                          Point(x=float(x), y=float(yy), z=0.05 + max(L, 0.011))]
            out.append(arr)
            lbl = "M%d" % (i+1)
            col = C_TEXT
            if dead:
                lbl = "M%d FAILED" % (i+1) if self.eta >= 1 else "M%d -%.0f%%" % (i+1, 100 * self.eta)
                col = C_DEAD
            elif off:
                lbl, col = "M%d off" % (i+1), C_OFF
            out.append(text(frame, ns + "_label", i, now, lbl, (x*1.25, yy*1.25, -0.06), 0.06, a(col)))
        # payload box under the frame; angle is measured in the G lever plane
        # (roll lever = y, pitch lever = -x), so 180 deg sits under M1
        if self.pl_m > 0:
            ang = np.deg2rad(self.pl_ang)
            px, py = -self.pl_r * np.sin(ang), self.pl_r * np.cos(ang)
            s = 0.05 + 0.1 * min(self.t / ha.t_ramp, 1.0)
            out.append(mk(frame, ns + "_payload", 0, Marker.CUBE, now, a(C_PAYLOAD), (s, s, s),
                          (px, py, -0.06 - s / 2)))
        return out

    def scene_markers(self, f, now, static):
        out = []
        W = "world"
        if static:
            # ground and target altitude ring for each drone
            out.append(mk(W, "ground", 0, Marker.CYLINDER, now, (0.16, 0.17, 0.19, 1.0), (7.0, 7.0, 0.01),
                          (0, 0, -0.01)))
            for j, yo in enumerate([y_ftc, y_ghost] if self.ghost_on else [y_ftc]):
                out.append(mk(W, "ref_ring", j, Marker.CYLINDER, now, (1, 1, 1, 0.06), (1.1, 1.1, 0.004),
                              (0, yo, z_ref)))
        # drone titles, follow the drones
        out.append(text(W, "title", 0, now, self.label, (0, y_ftc, self.y[6] + 0.55), 0.10, C_FTC))
        if self.ghost_on:
            gt = "PID, NO FTC (fault unknown)"
            if self.g_crash_t is not None:
                gt += " - FLIPPED, FALLING" if not self.g_down else " - CRASHED"
            out.append(text(W, "title", 1, now, gt, (0, y_ghost, max(self.yg[6], 0) + 0.55), 0.10, C_GHOST))
        else:
            out.append(text(W, "title", 1, now, "", (0, 0, 0), 0.1))
        out += self.chart_markers(now, static)
        # status line
        st = "t = %5.2f s" % self.t
        if f > 0:
            st += "   MOTOR %d FAILED" % f if self.eta >= 1 else "   MOTOR %d LOST %.0f%%" % (f, 100 * self.eta)
        else:
            st += "   all motors healthy"
        if self.pl_m > 0:
            st += "   payload %.2f kg @ %.2f m" % (self.pl_m, self.pl_r)
        if self.slow_k > 1 and self.in_slowmo() and not self.done:
            st += "   [SLOW MOTION x%.2f]" % (1.0 / self.slow_k)
        if self.done:
            st += "   (finished - close rviz to exit)" if self.hold_end else "   (finished)"
        out.append(text(W, "hud", 0, now, st, (chart_x, 0, chart_ztop + 0.6), 0.13, C_TEXT))
        speeds = "motor speed (rad/s):  " + "   ".join(
            "M%d %3d" % (i+1, int(round(self.y[8+i]))) for i in range(6))
        out.append(text(W, "hud", 1, now, speeds, (chart_x, 0, chart_ztop + 0.4), 0.09, C_DIM))
        info = "roll %+5.1f  pitch %+5.1f  yaw %+5.1f deg   z = %.2f m" % (
            np.rad2deg(self.y[0]), np.rad2deg(self.y[1]), np.rad2deg(self.y[2]), self.y[6])
        if self.min_after is not None and self.z_pre is not None:
            info += "   altitude dip after fault: %.2f m" % max(self.z_pre - self.min_after, 0)
        out.append(text(W, "hud", 2, now, info, (chart_x, 0, chart_ztop + 0.22), 0.09, C_DIM))
        g = ""
        if self.g_crash_t is not None:
            g = "no FTC: flipped over at t = %.2f s" % self.g_crash_t
            if self.g_ground_t is not None:
                g += ", hit the ground (%.0f m below) at t = %.2f s" % (z_ref, self.g_ground_t)
            else:
                g += ", falling (z = %.1f m)" % self.yg[6]
        out.append(text(W, "hud", 3, now, g, (chart_x, 0, panels[1][1] - 0.3), 0.09, C_GHOST))
        return out

    def chart_markers(self, now, static):
        out = []
        W = "world"
        span = chart_y1 - chart_y0
        ty = lambda t: chart_y0 + span * min(max(t / self.t_end, 0.0), 1.0)
        x = chart_x
        dt_tick = 2 if self.t_end <= 12 else 5
        for k, (name, zb, zt, lo, hi, ticks, unit) in enumerate(panels):
            zv = lambda v: zb + (zt - zb) * (v - lo) / (hi - lo)
            if static:
                # panel, axes, grid, labels
                out.append(mk(W, "chart_bg", k, Marker.CUBE, now, (0.07, 0.075, 0.09, 0.92),
                              (0.02, abs(span) + 0.5, zt - zb + 0.2),
                              (x - 0.02, (chart_y0 + chart_y1) / 2, (zb + zt) / 2)))
                out.append(line(W, "chart_axes", k, now,
                                [(x, chart_y0, zt), (x, chart_y0, zb), (x, chart_y1, zb)], C_DIM, 0.012))
                grid = []
                for v in ticks:
                    grid += [(x, chart_y0, zv(v)), (x, chart_y1, zv(v))]
                out.append(line(W, "chart_grid", k, now, grid, (1, 1, 1, 0.12), 0.006, Marker.LINE_LIST))
                for j, v in enumerate(ticks):
                    out.append(text(W, "chart_vlbl_%d" % k, j, now, "%g %s" % (v, unit),
                                    (x, chart_y0 + 0.25, zv(v)), 0.07, C_DIM))
                out.append(text(W, "chart_name", k, now, name, (x, chart_y0 - 0.35, zt - 0.08), 0.09, C_TEXT))
                if self.fault_motor > 0:
                    yf = ty(self.t_fault)
                    out.append(line(W, "chart_fault", k, now, [(x, yf, zb), (x, yf, zt)], C_OFF, 0.01))
            # traces: stop a trace where it leaves the panel (ghost falling away)
            col = 1 if name == "altitude" else 2
            for nm, hist, c, dx in [("ftc", self.hist_ftc, C_FTC, 0.01),
                                    ("ghost", self.hist_ghost if self.ghost_on else [], C_GHOST, 0.015)]:
                pts = []
                for r in hist[::2] + hist[-1:]:
                    v = r[col]
                    pts.append((x + dx, ty(r[0]), zv(min(max(v, lo), hi))))
                    if not lo <= v <= hi:
                        break
                out.append(line(W, "chart_" + nm, k, now, pts if len(pts) > 1 else [], c, 0.022))
            yc = ty(self.t)
            out.append(line(W, "chart_cursor", k, now, [(x + 0.01, yc, zb), (x + 0.01, yc, zt)],
                            (1, 1, 1, 0.25), 0.006))
        if static:
            # shared time axis under the bottom panel
            zb = panels[-1][1]
            for j in range(int(self.t_end // dt_tick) + 1):
                out.append(text(W, "chart_tlbl", j, now, "%d s" % (dt_tick*j), (x, ty(dt_tick*j), zb - 0.1),
                                0.07, C_DIM))
            if self.fault_motor > 0:
                out.append(text(W, "chart_fault_lbl", 0, now, "motor %d fails" % self.fault_motor,
                                (x, ty(self.t_fault), chart_ztop + 0.0), 0.08, C_OFF))
            # altitude reference line
            name, zb, zt, lo, hi, _, _ = panels[1]
            zr = zb + (zt - zb) * (z_ref - lo) / (hi - lo)
            out.append(line(W, "chart_ref", 0, now, [(x, chart_y0, zr), (x, chart_y1, zr)],
                            (1, 1, 1, 0.35), 0.008))
        return out

    def save(self):
        os.makedirs(self.out_dir, exist_ok=True)
        D = np.array(self.data)
        if self.fault_motor > 0:
            name = "motor%d_fault" % self.fault_motor
            if self.eta < 1:
                name += "_eta%02d" % round(100 * self.eta)
            title = "ROS2: motor %d %s at t=%.0fs" % (
                self.fault_motor, "failure" if self.eta >= 1 else "%.0f%% loss" % (100 * self.eta), self.t_fault)
            tf = self.t_fault
        else:
            name = "no_fault_disturbance" if self.dist_on else "no_fault"
            title = "ROS2: no failure" + (" + disturbance" if self.dist_on else "")
            tf = None
        if self.pl_m > 0:
            name += "_payload_r%02d_a%03d" % (round(100 * self.pl_r), round(self.pl_ang) % 360)
            title += ", payload %.2f kg at %.2f m / %.0f deg" % (self.pl_m, self.pl_r, self.pl_ang)
        name += self.tag
        title = self.label + " - " + title
        h.save_csv(D, os.path.join(self.out_dir, name + ".csv"))
        h.plot_case(D, name, title, tf is not None, None, self.out_dir)
        self.get_logger().info("saved %s/%s.png and .csv" % (self.out_dir, name))


def main():
    rclpy.init()
    node = Plant()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():                   # errors while shutting down (publish on dead context) are fine
            raise
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
