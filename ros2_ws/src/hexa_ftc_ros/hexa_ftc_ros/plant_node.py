# hexacopter plant: dynamics, motor fault, payload, disturbance and the rviz scene
# the plant sends the state, waits for the motor command, then does one step
import os
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TransformStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster
from hexa_ftc_ros import hexa_ftc as h
from hexa_ftc_ros import hexa_adrc as ha

# motor positions (x forward, y left), same order as the G matrix
s3 = np.sqrt(3) / 2
motor_xy = h.l * np.array([[0, -1], [0, 1], [s3, 0.5], [-s3, -0.5], [s3, -0.5], [-s3, 0.5]])

# chart behind the drone: time goes from y0 to y1
chart_x = -1.6
chart_y0 = 2.6
chart_y1 = -2.6
# (title, bottom z, top z, value min, value max, ticks, unit)
z0 = h.z_ref
panels = [("roll / pitch angle (deg)", z0 + 0.05, z0 + 1.25, -60.0, 60.0, [-60, -30, 0, 30, 60], "deg"),
          ("altitude z (m)", z0 - 1.55, z0 - 0.35, z0 - 1.5, z0 + 0.25, [z0 - 1.5, z0 - 1, z0 - 0.5, z0], "m")]
hud_z = z0 + 1.25

GREEN = (0.25, 0.90, 0.45, 1.0)
BLUE = (0.35, 0.65, 1.00, 1.0)
RED = (1.00, 0.15, 0.10, 1.0)
ORANGE = (1.00, 0.65, 0.10, 1.0)
YELLOW = (0.95, 0.80, 0.20, 1.0)
WHITE = (0.92, 0.93, 0.95, 1.0)
GREY = (0.60, 0.62, 0.66, 1.0)
DARK = (0.13, 0.14, 0.16, 1.0)


def euler_to_quat(phi, th, psi):
    cr, sr = np.cos(phi/2), np.sin(phi/2)
    cp, sp = np.cos(th/2), np.sin(th/2)
    cy, sy = np.cos(psi/2), np.sin(psi/2)
    return [sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy, cr*cp*cy + sr*sp*sy]


def marker(frame, ns, i, typ, color, scale, pos=(0, 0, 0)):
    mk = Marker()
    mk.header.frame_id = frame
    mk.ns = ns
    mk.id = i
    mk.type = typ
    mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = [float(v) for v in pos]
    mk.pose.orientation.w = 1.0
    mk.scale.x, mk.scale.y, mk.scale.z = [float(v) for v in scale]
    mk.color.r, mk.color.g, mk.color.b, mk.color.a = [float(v) for v in color]
    return mk


def text(frame, ns, i, s, pos, size, color=WHITE):
    mk = marker(frame, ns, i, Marker.TEXT_VIEW_FACING, color, (0, 0, size), pos)
    mk.text = s
    return mk


def line(frame, ns, i, pts, color, width, typ=Marker.LINE_STRIP):
    mk = marker(frame, ns, i, typ, color, (width, 0, 0))
    mk.points = [Point(x=float(a), y=float(b), z=float(c)) for a, b, c in pts]
    return mk


class Plant(Node):
    def __init__(self):
        super().__init__("plant_node")
        self.declare_parameter("fault_motor", 0)
        self.declare_parameter("fault_time", h.t_fault)
        self.declare_parameter("fault_eta", 1.0)
        self.declare_parameter("disturbance", False)
        self.declare_parameter("t_end", h.t_end)
        self.declare_parameter("payload_mass", 0.0)
        self.declare_parameter("payload_r", 0.0)
        self.declare_parameter("payload_angle", 0.0)
        self.declare_parameter("controller", "pid")
        self.declare_parameter("use_ftc", True)
        self.declare_parameter("out_dir", "results_ros")
        self.declare_parameter("rviz", False)
        self.declare_parameter("start_delay", 3.0)
        self.declare_parameter("slowmo", 0.15)
        self.declare_parameter("hold_time", 0.0)
        self.fault_motor = self.get_parameter("fault_motor").value
        self.t_fault = self.get_parameter("fault_time").value
        self.eta = self.get_parameter("fault_eta").value
        self.dist_on = self.get_parameter("disturbance").value
        self.t_end = self.get_parameter("t_end").value
        self.pl_mass = self.get_parameter("payload_mass").value
        self.pl_r = self.get_parameter("payload_r").value
        self.pl_ang = self.get_parameter("payload_angle").value
        self.ctrl = self.get_parameter("controller").value
        self.use_ftc = self.get_parameter("use_ftc").value
        self.out_dir = self.get_parameter("out_dir").value
        self.rviz = self.get_parameter("rviz").value
        self.start_delay = self.get_parameter("start_delay").value
        self.hold_time = self.get_parameter("hold_time").value
        # slow motion (only with rviz): advance one step every slow_k ticks around the fault
        slowmo = self.get_parameter("slowmo").value
        self.slow_k = int(round(1 / slowmo)) if self.rviz and slowmo > 0 else 1

        if self.use_ftc:
            self.label = self.ctrl.upper() + " + re-allocation"
        else:
            self.label = self.ctrl.upper() + " (fault unknown)"

        self.y = h.start_state()
        self.t = 0.0
        self.k = 0
        self.n = int(round(self.t_end / h.Ts))
        self.w_cmd = None
        self.M_des = np.zeros(4)
        self.data = []
        self.hist = [(0.0, self.y[6], 0.0, 0.0)]     # (t, z, roll, pitch) for the chart
        self.sent = False
        self.t_sent = 0.0
        self.t_start = None
        self.t_done = None
        self.skip = 0
        self.finished = False

        self.state_pub = self.create_publisher(Float64MultiArray, "/state", 10)
        self.mark_pub = self.create_publisher(MarkerArray, "/hexa_markers", 10)
        self.tf_pub = TransformBroadcaster(self)
        self.create_subscription(Float64MultiArray, "/motor_cmd", self.cmd_cb, 10)
        self.create_timer(h.Ts, self.step)
        self.create_timer(0.05, self.publish_viz)
        self.get_logger().info("plant started: motor %d, eta %.2f, payload %.2f kg at %.2f m / %.0f deg"
                               % (self.fault_motor, self.eta, self.pl_mass, self.pl_r, self.pl_ang))

    def fault_now(self):
        if self.fault_motor > 0 and self.t >= self.t_fault:
            return self.fault_motor
        return 0

    def cmd_cb(self, msg):
        if self.sent and int(msg.data[10]) == self.k:
            self.w_cmd = np.array(msg.data[:6])
            self.M_des = np.array(msg.data[6:10])

    def send_state(self):
        eff = h.motor_eff(self.fault_now(), self.eta)
        self.y[8:14][eff == 0] = 0       # failed motor stops at once
        msg = Float64MultiArray()
        msg.data = [float(v) for v in self.y] + [float(self.k), float(self.fault_now()), float(self.eta)]
        self.state_pub.publish(msg)
        self.w_cmd = None
        self.sent = True
        self.t_sent = time.time()

    def step(self):
        if self.t_done is not None:
            # keep showing the end, then quit (with rviz and hold_time 0: stay open)
            if not self.rviz or self.hold_time > 0:
                if time.time() - self.t_done > (self.hold_time if self.rviz else 2.0):
                    self.finished = True
            return
        if self.rviz:
            if self.mark_pub.get_subscription_count() == 0:
                return                   # wait for rviz
            if self.t_start is None:
                self.t_start = time.time()
            if time.time() - self.t_start < self.start_delay:
                return
        if self.state_pub.get_subscription_count() == 0:
            return                       # wait for ftc_node
        if not self.sent:
            self.send_state()
            return
        if self.w_cmd is None:
            if time.time() - self.t_sent > 1.0:
                self.send_state()        # no answer, send again
            return
        if self.slow_k > 1 and self.fault_motor > 0 and self.t_fault - 0.1 <= self.t <= self.t_fault + 1.5:
            self.skip += 1
            if self.skip < self.slow_k:
                return
            self.skip = 0

        eff = h.motor_eff(self.fault_now(), self.eta)
        self.data.append(np.r_[self.t, self.y, self.M_des, h.G @ (eff * self.y[8:14]**2)])
        if h.crashed(self.y) or self.k >= self.n:
            if h.crashed(self.y):
                self.get_logger().warn("CRASH at t=%.2f s" % self.t)
            self.save()
            self.t_done = time.time()
            return
        pl = ha.payload(self.pl_r, self.pl_ang, self.pl_mass * min(self.t / ha.t_ramp, 1.0))
        self.y = h.plant_step(self.y, self.w_cmd, eff, pl, h.dist(self.t, self.dist_on))
        self.k += 1
        self.t = self.k * h.Ts
        self.hist.append((self.t, self.y[6], np.rad2deg(self.y[0]), np.rad2deg(self.y[1])))
        self.send_state()

    def publish_viz(self):
        tr = TransformStamped()
        tr.header.stamp = self.get_clock().now().to_msg()
        tr.header.frame_id = "world"
        tr.child_frame_id = "base_link"
        tr.transform.translation.z = float(max(self.y[6], 0.0))
        q = euler_to_quat(self.y[0], self.y[1], self.y[2])
        tr.transform.rotation.x = float(q[0])
        tr.transform.rotation.y = float(q[1])
        tr.transform.rotation.z = float(q[2])
        tr.transform.rotation.w = float(q[3])
        self.tf_pub.sendTransform(tr)
        if self.mark_pub.get_subscription_count() == 0:
            return
        ma = MarkerArray()
        ma.markers = self.drone_markers() + self.hud_markers() + self.chart_markers()
        self.mark_pub.publish(ma)

    def drone_markers(self):
        out = []
        B = "base_link"
        f = self.fault_now()
        out.append(marker(B, "body", 0, Marker.CYLINDER, DARK, (0.20, 0.20, 0.06)))
        out.append(marker(B, "body", 1, Marker.CYLINDER, GREEN, (0.12, 0.12, 0.065)))
        out.append(marker(B, "body", 2, Marker.ARROW, WHITE, (0.16, 0.025, 0.025), (0.06, 0, 0.04)))
        arms = []
        for i in range(6):
            x, y = motor_xy[i]
            w = self.y[8+i]
            arms += [(0, 0, 0), (x, y, 0)]
            if f == i + 1:
                col = RED
                if self.eta >= 1:
                    lbl = "M%d FAILED" % (i+1)
                else:
                    lbl = "M%d -%.0f%%" % (i+1, 100 * self.eta)
            elif f > 0 and w < 5:
                col = ORANGE              # healthy motor switched off by the controller
                lbl = "M%d off" % (i+1)
            else:
                col = WHITE
                lbl = "M%d" % (i+1)
            prop_col = (0.5, 0.75, 1.0, 0.4) if col == WHITE else col
            out.append(marker(B, "prop", i, Marker.CYLINDER, prop_col, (0.26, 0.26, 0.006), (x, y, 0.04)))
            out.append(text(B, "label", i, lbl, (1.25*x, 1.25*y, -0.06), 0.06, col))
            # thrust arrow, 0.5 m = hover thrust
            L = 0.5 * (w / h.w_hover)**2
            arrow = marker(B, "thrust", i, Marker.ARROW, (0.2, 0.75, 1.0, 0.9 if L > 0.01 else 0.0),
                           (0.018, 0.04, 0.05))
            arrow.points = [Point(x=float(x), y=float(y), z=0.05), Point(x=float(x), y=float(y), z=0.05 + max(L, 0.011))]
            out.append(arrow)
        out.append(line(B, "arms", 0, arms, (0.3, 0.3, 0.33, 1.0), 0.022, Marker.LINE_LIST))
        if self.pl_mass > 0:
            pos, J, mass = ha.payload(self.pl_r, self.pl_ang, 1.0)
            s = 0.05 + 0.1 * min(self.t / ha.t_ramp, 1.0)
            out.append(marker(B, "payload", 0, Marker.CUBE, YELLOW, (s, s, s), (-pos[1], pos[0], -0.06 - s/2)))
        return out

    def hud_markers(self):
        out = []
        W = "world"
        f = self.fault_now()
        out.append(marker(W, "ground", 0, Marker.CYLINDER, (0.16, 0.17, 0.19, 1.0), (7, 7, 0.01), (0, 0, -0.01)))
        out.append(text(W, "title", 0, self.label, (0, 0, self.y[6] + 0.55), 0.10, GREEN))
        st = "t = %5.2f s" % self.t
        if f > 0 and self.eta >= 1:
            st += "   MOTOR %d FAILED" % f
        elif f > 0:
            st += "   MOTOR %d LOST %.0f%%" % (f, 100 * self.eta)
        else:
            st += "   all motors healthy"
        if self.t_done is not None:
            st += "   (finished)"
        out.append(text(W, "hud", 0, st, (chart_x, 0, hud_z + 0.6), 0.13))
        if self.pl_mass > 0:
            if self.pl_r > 0:
                where = "%.0f cm %s" % (100 * self.pl_r, ha.dir_name(self.pl_ang))
            else:
                where = "at the centre"
            out.append(text(W, "hud", 1, "PAYLOAD %.2f kg, %s" % (self.pl_mass, where),
                            (chart_x, 0, hud_z + 0.85), 0.16, YELLOW))
        speeds = "motor speed (rad/s):  " + "   ".join("M%d %3d" % (i+1, round(self.y[8+i])) for i in range(6))
        out.append(text(W, "hud", 2, speeds, (chart_x, 0, hud_z + 0.4), 0.09, GREY))
        ang = np.rad2deg(self.y[:3])
        info = "roll %+5.1f  pitch %+5.1f  yaw %+5.1f deg   z = %.2f m" % (ang[0], ang[1], ang[2], self.y[6])
        out.append(text(W, "hud", 3, info, (chart_x, 0, hud_z + 0.22), 0.09, GREY))
        return out

    def chart_markers(self):
        out = []
        W = "world"
        x = chart_x

        def ty(t):
            return chart_y0 + (chart_y1 - chart_y0) * min(max(t / self.t_end, 0.0), 1.0)

        dt = 2.5 if self.t_end > 8 else 1.0
        t_ticks = np.arange(0, self.t_end + 1e-6, dt)
        for k in range(2):
            title, zb, zt, lo, hi, ticks, unit = panels[k]

            def zv(v):
                return zb + (zt - zb) * (min(max(v, lo), hi) - lo) / (hi - lo)

            out.append(marker(W, "chart_bg", k, Marker.CUBE, (0.07, 0.075, 0.09, 0.92),
                              (0.02, abs(chart_y1 - chart_y0) + 1.2, zt - zb + 0.2),
                              (x - 0.02, (chart_y0 + chart_y1) / 2, (zb + zt) / 2)))
            out.append(line(W, "chart_axes", k, [(x, chart_y0, zt), (x, chart_y0, zb), (x, chart_y1, zb)], GREY, 0.012))
            grid = []
            for v in ticks:
                grid += [(x, chart_y0, zv(v)), (x, chart_y1, zv(v))]
            for t in t_ticks:
                grid += [(x, ty(t), zb), (x, ty(t), zt)]
            out.append(line(W, "chart_grid", k, grid, (1, 1, 1, 0.12), 0.006, Marker.LINE_LIST))
            for j, v in enumerate(ticks):
                out.append(text(W, "chart_vlbl%d" % k, j, "%g %s" % (v, unit), (x, chart_y0 + 0.32, zv(v)), 0.085, GREY))
            out.append(text(W, "chart_title", k, title, (x, chart_y0 - 0.75, zt + 0.06), 0.095))
            if self.fault_motor > 0:
                out.append(line(W, "chart_fault", k, [(x, ty(self.t_fault), zb), (x, ty(self.t_fault), zt)], ORANGE, 0.01))
            # traces: altitude in the bottom panel, roll and pitch in the top one
            if k == 0:
                traces = [(2, GREEN), (3, BLUE)]
            else:
                traces = [(1, GREEN)]
            for col, color in traces:
                pts = [(x + 0.01, ty(r[0]), zv(r[col])) for r in self.hist[::2] + self.hist[-1:]]
                out.append(line(W, "chart_trace%d" % k, col, pts if len(pts) > 1 else [], color, 0.022))
        zb = panels[1][1]
        for j, t in enumerate(t_ticks):
            out.append(text(W, "chart_tlbl", j, "%g" % t, (x, ty(t), zb - 0.12), 0.085, GREY))
        out.append(text(W, "chart_tname", 0, "time (s)", (x, 0, zb - 0.27), 0.095))
        out.append(text(W, "chart_legend", 0, "roll", (x, chart_y1 + 0.25, hud_z - 0.08), 0.085, GREEN))
        out.append(text(W, "chart_legend", 1, "pitch", (x, chart_y1 + 0.25, hud_z - 0.22), 0.085, BLUE))
        if self.fault_motor > 0:
            out.append(text(W, "chart_fault_lbl", 0, "motor %d fails" % self.fault_motor,
                            (x, ty(self.t_fault), hud_z), 0.08, ORANGE))
        return out

    def save(self):
        os.makedirs(self.out_dir, exist_ok=True)
        D = np.array(self.data)
        if self.fault_motor > 0:
            name = "motor%d_fault" % self.fault_motor
            if self.eta < 1:
                name += "_eta%02d" % round(100 * self.eta)
            title = "ROS2: motor %d failure at t=%.0fs" % (self.fault_motor, self.t_fault)
        else:
            name = "no_fault_disturbance" if self.dist_on else "no_fault"
            title = "ROS2: no failure"
        if self.pl_mass > 0:
            name += "_payload_r%02d_a%03d" % (round(100 * self.pl_r), round(self.pl_ang) % 360)
        if self.ctrl != "pid" or not self.use_ftc:
            name += "_" + self.ctrl + ("" if self.use_ftc else "_blind")
        h.save_csv(D, os.path.join(self.out_dir, name + ".csv"))
        h.plot_case(D, name, self.label + " - " + title, self.t_fault if self.fault_motor > 0 else None,
                    None, self.out_dir)
        self.get_logger().info("saved %s/%s.png and .csv" % (self.out_dir, name))


def main():
    rclpy.init()
    node = Plant()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
