#!/usr/bin/env python3
# motor_comm.py - listens to /motor_cmd (Twist.linear.x) and sends serial M,<l>,<r>
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import math

class MotorComm(Node):
    def __init__(self):
        super().__init__('motor_comm')
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('use_sim', True)
        self.declare_parameter('cmd_topic', '/motor_cmd')
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.use_sim = self.get_parameter('use_sim').get_parameter_value().bool_value
        self.cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value

        self.get_logger().info('MotorComm use_sim=%s' % str(self.use_sim))
        if not self.use_sim:
            try:
                self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=0.1)
            except Exception as e:
                self.get_logger().error('Failed to open serial port: %s' % str(e))
                raise

        self.sub = self.create_subscription(Twist, self.cmd_topic, self.cmd_cb, 10)
        # if sim: publish to /cmd_vel or to gazebo controllers, but we keep compatibility
        self.sim_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def cmd_cb(self, msg: Twist):
        u = msg.linear.x  # controller output
        # map u (-max..max) into -127..127
        # We assume controller uses max_output=120 in node, so scale accordingly
        max_effort = 120.0
        v = int(max(-127, min(127, int((u / max_effort) * 127))))
        left = v
        right = v
        if self.use_sim:
            # for sim, publish Twist to cmd_vel so a differential drive node can consume
            cmd = Twist()
            cmd.linear.x = (left + right) / 2.0 / 127.0
            cmd.angular.z = 0.0
            self.sim_pub.publish(cmd)
        else:
            try:
                s = f"M,{left},{right}\n"
                self.ser.write(s.encode('utf-8'))
            except Exception as e:
                self.get_logger().error('Serial write error: %s' % str(e))

def main(args=None):
    rclpy.init(args=args)
    node = MotorComm()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
