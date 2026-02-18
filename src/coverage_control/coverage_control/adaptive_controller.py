import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import time
import math


class AdaptiveCoverageController(Node):

    def __init__(self):
        super().__init__('adaptive_controller')

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)

        # -------- MAP --------
        self.x_min, self.x_max = -5.0, 5.0
        self.y_min, self.y_max = -5.0, 5.0
        self.res = 0.2

        self.xs = np.arange(self.x_min, self.x_max, self.res)
        self.ys = np.arange(self.y_min, self.y_max, self.res)
        self.X, self.Y = np.meshgrid(self.xs, self.ys)

        # -------- TRUE DENSITY --------
        self.rho_true = np.exp(-0.2 * ((self.X - 2)**2 + (self.Y - 2)**2))

        # -------- ESTIMATED DENSITY (PRIOR) --------
        self.rho_hat = 1e-3 * np.ones_like(self.rho_true)

        self.position = None
        self.yaw = 0.0

        self.start_time = time.time()

        # -------- GAINS --------
        self.kp_lin = 0.4
        self.kp_ang = 1.5
        self.max_v = 0.25
        self.max_w = 1.5

        self.voronoi_radius = 4  # grid cells

        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_cb(self, msg):
        self.position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ])

        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w*q.z + q.x*q.y)
        cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.yaw = math.atan2(siny, cosy)

    def control_loop(self):
        if self.position is None:
            return

        t = time.time() - self.start_time

        # -------- EXPLORATION KICK --------
        if t < 4.0:
            cmd = Twist()
            cmd.linear.x = 0.15
            self.cmd_pub.publish(cmd)
            return

        # -------- GRID INDEX --------
        ix = int((self.position[0] - self.x_min) / self.res)
        iy = int((self.position[1] - self.y_min) / self.res)
        ix = np.clip(ix, 0, self.X.shape[1]-1)
        iy = np.clip(iy, 0, self.X.shape[0]-1)

        # -------- UPDATE LOCAL DENSITY --------
        self.rho_hat[iy, ix] = self.rho_true[iy, ix]

        # -------- LOCAL ADAPTIVE VORONOI --------
        r = self.voronoi_radius
        ix0, ix1 = max(ix-r, 0), min(ix+r+1, self.X.shape[1])
        iy0, iy1 = max(iy-r, 0), min(iy+r+1, self.Y.shape[0])

        local_rho = self.rho_hat[iy0:iy1, ix0:ix1].copy()
        local_X = self.X[iy0:iy1, ix0:ix1]
        local_Y = self.Y[iy0:iy1, ix0:ix1]

        # REMOVE SELF INFLUENCE
        local_rho[iy-iy0, ix-ix0] = 0.0

        if np.sum(local_rho) < 1e-6:
            self.stop()
            return

        cx = np.sum(local_X * local_rho) / np.sum(local_rho)
        cy = np.sum(local_Y * local_rho) / np.sum(local_rho)

        # -------- CONTROL LAW --------
        dx = cx - self.position[0]
        dy = cy - self.position[1]
        dist = math.hypot(dx, dy)

        if dist < 0.05:
            self.stop()
            return

        desired_yaw = math.atan2(dy, dx)
        yaw_error = math.atan2(
            math.sin(desired_yaw - self.yaw),
            math.cos(desired_yaw - self.yaw)
        )

        cmd = Twist()
        cmd.linear.x = np.clip(self.kp_lin * dist, 0.0, self.max_v)
        cmd.angular.z = np.clip(self.kp_ang * yaw_error, -self.max_w, self.max_w)

        self.cmd_pub.publish(cmd)

    def stop(self):
        self.cmd_pub.publish(Twist())


def main():
    rclpy.init()
    node = AdaptiveCoverageController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
