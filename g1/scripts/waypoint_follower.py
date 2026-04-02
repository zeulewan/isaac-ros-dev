#!/usr/bin/env python3
"""Simple waypoint follower for G1.

Receives a goal pose from Lichtblick (/goal_pose or /move_base_simple/goal),
reads current pose from FAST-LIO (/Odometry), and sends velocity commands
to walk there.

Based on g1pilot's nav2point approach: proportional control, ~150 lines.
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        # Gains
        self.pos_kp = 0.8
        self.yaw_kp = 1.5

        # Limits (match training ranges)
        self.vx_max = 0.5
        self.vy_max = 0.3
        self.vyaw_max = 0.5

        # Tolerances
        self.goal_tolerance = 0.3  # meters
        self.yaw_tolerance = 0.15  # radians

        # State
        self.goal = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.active = False

        # Subscribers
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.create_subscription(PoseStamped, '/move_base_simple/goal', self.goal_cb, 10)
        self.create_subscription(Odometry, '/Odometry', self.odom_cb, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Control loop at 20 Hz
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Waypoint follower ready. Click a goal in Lichtblick.')

    def goal_cb(self, msg):
        self.goal = msg.pose
        self.active = True
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        dist = math.sqrt((gx - self.current_x)**2 + (gy - self.current_y)**2)
        self.get_logger().info(f'New goal: ({gx:.2f}, {gy:.2f}), distance: {dist:.2f}m')

    def odom_cb(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        if not self.active or self.goal is None:
            return

        # Error in world frame
        dx = self.goal.position.x - self.current_x
        dy = self.goal.position.y - self.current_y
        dist = math.sqrt(dx * dx + dy * dy)

        # Check if we reached the goal
        if dist < self.goal_tolerance:
            self.get_logger().info(f'Goal reached! (error: {dist:.2f}m)')
            self.active = False
            # Send stop
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            return

        # Desired heading to goal
        desired_yaw = math.atan2(dy, dx)
        yaw_error = desired_yaw - self.current_yaw
        # Normalize to [-pi, pi]
        while yaw_error > math.pi:
            yaw_error -= 2 * math.pi
        while yaw_error < -math.pi:
            yaw_error += 2 * math.pi

        # Transform world error to body frame
        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)
        body_dx = cos_yaw * dx + sin_yaw * dy
        body_dy = -sin_yaw * dx + cos_yaw * dy

        # Proportional control
        vx = self.pos_kp * body_dx
        vy = self.pos_kp * body_dy
        vyaw = self.yaw_kp * yaw_error

        # If facing wrong direction, turn first before walking
        if abs(yaw_error) > 0.5:
            vx *= 0.2  # slow down forward
            vy *= 0.2

        # Clamp
        vx = max(-self.vx_max, min(self.vx_max, vx))
        vy = max(-self.vy_max, min(self.vy_max, vy))
        vyaw = max(-self.vyaw_max, min(self.vyaw_max, vyaw))

        # Publish
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.angular.z = vyaw
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    rclpy.spin(WaypointFollower())


if __name__ == '__main__':
    main()
