# PID controller + control (re)allocation
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float64MultiArray
from hexa_ftc_ros import hexa_ftc as h


class FTC(Node):
    def __init__(self):
        super().__init__("ftc_node")
        self.declare_parameter("z_ref", h.z_ref)
        self.declare_parameter("use_ftc", True)
        self.z_ref = self.get_parameter("z_ref").value
        self.use_ftc = self.get_parameter("use_ftc").value
        self.ctrl = h.Controller()
        self.fault = 0
        self.last_k = -1
        self.last_out = None
        self.cmd_pub = self.create_publisher(Float64MultiArray, "/motor_cmd", 10)
        self.wr_pub = self.create_publisher(Float64MultiArray, "/wrench_des", 10)
        self.create_subscription(Float64MultiArray, "/state", self.state_cb, 10)
        self.get_logger().info("ftc node started, re-allocation = %s" % self.use_ftc)

    def state_cb(self, msg):
        # /state = [y (14), step k, fault f], reply /motor_cmd = [w (6), M (4), k]
        d = msg.data
        y = np.array(d[:14])
        k = int(round(d[14]))
        f = int(round(d[15]))
        if k == self.last_k:             # plant re-sent the same state: same answer again
            self.cmd_pub.publish(self.last_out)
            return
        if f != self.fault:
            self.fault = f
            if f > 0:
                self.get_logger().warn("motor %d failed -> new allocation matrix G%d" % (f, f))
        M = self.ctrl.update(y, self.z_ref)
        w = h.allocate(M, self.fault, self.use_ftc)
        out = Float64MultiArray()
        out.data = [float(v) for v in w] + [float(v) for v in M] + [float(k)]
        self.cmd_pub.publish(out)
        self.last_k, self.last_out = k, out
        out = Float64MultiArray()
        out.data = [float(v) for v in M]
        self.wr_pub.publish(out)


def main():
    rclpy.init()
    node = FTC()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():                   # errors while shutting down are fine
            raise
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
