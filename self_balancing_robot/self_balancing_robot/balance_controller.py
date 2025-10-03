#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from .kalman_filter import ComplementaryFilter
import math
import time

class BalanceController(Node):
    def __init__(self):
        super().__init__('balance_controller')
        self.declare_parameter('kp', 80.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 1.5)
        self.declare_parameter('max_output', 120)
        self.declare_parameter('dt', 0.01)

        self.kp = float(self.get_parameter('kp').value)
        self.ki = float(self.get_parameter('ki').value)
        self.kd = float(self.get_parameter('kd').value)
        self.max_output = float(self.get_parameter('max_output').value)
        self.dt = float(self.get_parameter('dt').value)

        self.filter = ComplementaryFilter(alpha=0.98, dt=self.dt)

        self.imu_sub = self.create_subscription(Imu, '/imu/data_raw', self.imu_cb, 10)
        self.motor_pub = self.create_publisher(Twist, '/motor_cmd', 10)  # linear.x -> control effort
        self.target_pitch = 0.0

        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = self.get_clock().now()

    def imu_cb(self, msg: Imu):
        # extract accel x,z and gyro y (pitch rate)
        ax = msg.linear_acceleration.x
        az = msg.linear_acceleration.z
        gyro_y = msg.angular_velocity.y  # rad/s

        angle = self.filter.update(ax, az, gyro_y)  # rad
        error = angle - self.target_pitch

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            dt = self.dt
        self.last_time = now

        self.integral += error * dt
        derivative = (error - self.last_error) / dt if dt > 0 else 0.0
        self.last_error = error

        u = self.kp * error + self.ki * self.integral + self.kd * derivative
        # clamp
        u = max(min(u, self.max_output), -self.max_output)

        cmd = Twist()
        # Map u (control effort) to linear.x (we will convert to pwm in motor_comm)
        cmd.linear.x = float(u)
        self.motor_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = BalanceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
