# self_balancing_robot (2-Wheel) — ROS2 + ESP32

## Overview
This project implements a self-balancing two-wheel robot: IMU + encoders on ESP32 -> serial -> ROS2 imu_bridge -> Complementary filter -> Balance PID -> motor_comm -> ESP32 motors.

## Quick start (simulation)
1. Build/publish URDF into robot_state_publisher or spawn in Gazebo.
2. Run:
ros2 launch self_balancing_robot sim.launch.py

3. Use `rviz2` / `rqt_graph` to inspect topics: `/imu/data_raw`, `/motor_cmd`, `/cmd_vel`.

## Quick start (hardware)
1. Flash `microcontroller/src/motor_controller.ino` to ESP32.
2. Connect ESP32 to PC (USB) and note the serial port (e.g., `/dev/ttyUSB0`).
3. Start ROS nodes:
ros2 launch self_balancing_robot hw.launch.py serial_port:=/dev/ttyUSB0 baudrate:=115200

4. Start controller:
ros2 run self_balancing_robot balance_controller

(launch file already starts everything.)

## Tuning
- Start with small gains: `kp=10`, `kd=0.5`. Increase gradually.
- Adjust complementary filter alpha in `kalman_filter.py`.

## File map
- `microcontroller/` — ESP32 Arduino firmware
- `urdf/` — robot description
- `ros2_pkg/` — ROS2 nodes

