#!/usr/bin/env python3
# imu_bridge.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import serial
import math
import time

class IMUBridge(Node):
    def __init__(self):
        super().__init__('imu_bridge')
        # params
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('use_sim', True)
        self.declare_parameter('sim_imu_topic', '/sim_imu')
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.use_sim = self.get_parameter('use_sim').get_parameter_value().bool_value
        self.sim_imu_topic = self.get_parameter('sim_imu_topic').get_parameter_value().string_value

        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        if self.use_sim:
            self.get_logger().info('IMU Bridge in SIM mode, subscribing to: %s' % self.sim_imu_topic)
            self.create_subscription(Imu, self.sim_imu_topic, self.sim_cb, 10)
        else:
            self.get_logger().info('IMU Bridge in HW mode, opening serial: %s' % self.serial_port)
            try:
                self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=0.1)
            except Exception as e:
                self.get_logger().error('Failed to open serial: %s' % str(e))
                raise

            self.timer = self.create_timer(0.01, self.timer_cb)

    def sim_cb(self, msg):
        # pass-through simulated IMU
        imu = Imu()
        imu.header = msg.header
        imu.orientation = msg.orientation
        imu.angular_velocity = msg.angular_velocity
        imu.linear_acceleration = msg.linear_acceleration
        self.imu_pub.publish(imu)

        odom = Odometry()
        odom.header = msg.header
        # odom placeholder
        self.odom_pub.publish(odom)

    def timer_cb(self):
        try:
            line = self.ser.readline().decode('utf-8').strip()
        except Exception:
            return
        if not line:
            return
        parts = line.split(',')
        # expected: t,ax,ay,az,gx,gy,gz,encL,encR
        if len(parts) < 9:
            return
        try:
            t = float(parts[0])
            ax = float(parts[1]); ay = float(parts[2]); az = float(parts[3])
            gx = float(parts[4]); gy = float(parts[5]); gz = float(parts[6])
            encL = float(parts[7]); encR = float(parts[8])
        except:
            return

        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = 'base_link'
        # RAW accel/gyro scaling depends on IMU; here we assume MPU raw units:
        # accel raw -> g: divide by 16384; gyro raw -> deg/s: divide by 131
        imu.linear_acceleration.x = ax / 16384.0 * 9.80665
        imu.linear_acceleration.y = ay / 16384.0 * 9.80665
        imu.linear_acceleration.z = az / 16384.0 * 9.80665
        imu.angular_velocity.x = math.radians(gx / 131.0)
        imu.angular_velocity.y = math.radians(gy / 131.0)
        imu.angular_velocity.z = math.radians(gz / 131.0)
        self.imu_pub.publish(imu)

        odom = Odometry()
        odom.header = imu.header
        # very simple odom: mean encoder counts -> velocity placeholder
        odom.twist.twist.linear.x = (encL + encR) * 0.0
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = IMUBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
