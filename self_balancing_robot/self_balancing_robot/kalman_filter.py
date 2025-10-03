# kalman_filter.py
import math

class ComplementaryFilter:
    def __init__(self, alpha=0.98, dt=0.01):
        self.alpha = alpha
        self.dt = dt
        self.angle = 0.0

    def update(self, acc_x, acc_z, gyro_y):
        # acc_x, acc_z in m/s^2 ; gyro_y in rad/s
        angle_acc = math.atan2(acc_x, acc_z)  # pitch estimate from accel
        self.angle = self.alpha * (self.angle + gyro_y * self.dt) + (1.0 - self.alpha) * angle_acc
        return self.angle
