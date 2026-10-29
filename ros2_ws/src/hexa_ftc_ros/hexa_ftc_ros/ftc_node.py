# controller (PID or ADRC) + control allocation
# use_ftc = true: the fault is known and the allocation is changed
# use_ftc = false: the fault is never looked at
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float64MultiArray
from hexa_ftc_ros import hexa_ftc as h
from hexa_ftc_ros import hexa_adrc as ha


class FTC(Node):
    def __init__(self):
        super().__init__("ftc_node")
        self.declare_parameter("use_ftc", True)
        self.declare_parameter("controller", "pid")
        self.use_ftc = self.get_parameter("use_ftc").value
        self.ctrl = self.get_parameter("controller").value
        self.loop = None
        self.last_k = -1
        self.last_msg = None
        self.pub = self.create_publisher(Float64MultiArray, "/motor_cmd", 10)
        self.create_subscription(Float64MultiArray, "/state", self.state_cb, 10)
        self.get_logger().info("controller = %s, re-allocation = %s" % (self.ctrl, self.use_ftc))

    def state_cb(self, msg):
        # /state = [y (14), k, fault, eta]  ->  /motor_cmd = [w (6), M (4), k]
        y = np.array(msg.data[:14])
        k = int(msg.data[14])
        fault = int(msg.data[15])
        eta = msg.data[16]
        if k == self.last_k:             # same state sent again, send the same answer
            self.pub.publish(self.last_msg)
            return
        if self.loop is None:
            self.loop = ha.Loop(self.ctrl, y)
        if self.use_ftc:
            eff = h.motor_eff(fault, eta)
        else:
            eff = np.ones(6)
        w, M = self.loop.step(y, eff)
        out = Float64MultiArray()
        out.data = [float(v) for v in w] + [float(v) for v in M] + [float(k)]
        self.pub.publish(out)
        self.last_k = k
        self.last_msg = out


def main():
    rclpy.init()
    node = FTC()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
