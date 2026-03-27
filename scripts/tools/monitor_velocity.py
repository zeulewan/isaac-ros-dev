#!/usr/bin/env python3
"""Monitor cmd_vel and odom to measure actual robot speed and position."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

class VelocityMonitor(Node):
    def __init__(self):
        super().__init__('velocity_monitor')
        self.use_sim_time = True
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.create_subscription(Odometry, '/chassis/odom', self.odom_cb, qos)

        self.last_cmd = None
        self.last_odom = None
        self.peak_cmd_vx = 0.0
        self.peak_odom_vx = 0.0
        self.sample_count = 0

        self.create_timer(1.0, self.print_status)
        self.get_logger().info('Velocity monitor started - watching /cmd_vel and /chassis/odom')

    def cmd_vel_cb(self, msg):
        self.last_cmd = msg
        vx = abs(msg.linear.x)
        if vx > self.peak_cmd_vx:
            self.peak_cmd_vx = vx

    def odom_cb(self, msg):
        self.last_odom = msg
        vx = abs(msg.twist.twist.linear.x)
        if vx > self.peak_odom_vx:
            self.peak_odom_vx = vx
        self.sample_count += 1

    def print_status(self):
        cmd_vx = self.last_cmd.linear.x if self.last_cmd else 0.0
        cmd_wz = self.last_cmd.angular.z if self.last_cmd else 0.0

        if self.last_odom:
            ox = self.last_odom.pose.pose.position.x
            oy = self.last_odom.pose.pose.position.y
            ovx = self.last_odom.twist.twist.linear.x
            owz = self.last_odom.twist.twist.angular.z
        else:
            ox = oy = ovx = owz = 0.0

        print(f'CMD: vx={cmd_vx:+6.2f} wz={cmd_wz:+5.2f} | '
              f'ODOM: vx={ovx:+6.2f} wz={owz:+5.2f} pos=({ox:+7.2f},{oy:+7.2f}) | '
              f'PEAK cmd={self.peak_cmd_vx:.2f} odom={self.peak_odom_vx:.2f}',
              flush=True)

def main():
    rclpy.init()
    node = VelocityMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
