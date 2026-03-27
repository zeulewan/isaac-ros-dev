#!/usr/bin/env python3
"""Bypass Nav2 and send cmd_vel_raw directly to Isaac Sim to find the sim's max velocity."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import sys

class SpeedTest(Node):
    def __init__(self, target_vx):
        super().__init__('speed_test')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.target_vx = target_vx

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.odom_sub = self.create_subscription(Odometry, '/chassis/odom', self.odom_cb, qos)
        # Publish directly to cmd_vel_raw — bypasses Nav2 and relay
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)

        self.peak_vx = 0.0
        self.samples = []
        self.start_time = None

        # Publish cmd_vel_raw at 20 Hz
        self.pub_timer = self.create_timer(0.05, self.publish_cmd)
        # Print status at 1 Hz
        self.print_timer = self.create_timer(1.0, self.print_status)
        # Stop after 15 seconds
        self.stop_timer = self.create_timer(15.0, self.stop)

        self.get_logger().info(f'Speed test: sending vx={target_vx:.1f} directly to /cmd_vel_raw for 15s')

    def publish_cmd(self):
        msg = Twist()
        msg.linear.x = self.target_vx
        self.cmd_pub.publish(msg)

    def odom_cb(self, msg):
        vx = msg.twist.twist.linear.x
        self.samples.append(abs(vx))
        if abs(vx) > self.peak_vx:
            self.peak_vx = abs(vx)

    def print_status(self):
        if self.samples:
            recent = self.samples[-20:]
            avg = sum(recent) / len(recent)
            print(f'  Odom vx: current={self.samples[-1]:.3f}  avg_recent={avg:.3f}  peak={self.peak_vx:.3f}  (target={self.target_vx:.1f})', flush=True)

    def stop(self):
        # Send zero velocity
        msg = Twist()
        self.cmd_pub.publish(msg)
        self.cmd_pub.publish(msg)

        if self.samples:
            # Skip first 3 seconds of acceleration
            skip = min(60, len(self.samples) // 3)
            cruise_samples = self.samples[skip:]
            if cruise_samples:
                avg_cruise = sum(cruise_samples) / len(cruise_samples)
                print(f'\n=== RESULTS ===', flush=True)
                print(f'Target vx:    {self.target_vx:.1f} m/s', flush=True)
                print(f'Peak odom vx: {self.peak_vx:.3f} m/s', flush=True)
                print(f'Avg cruise:   {avg_cruise:.3f} m/s', flush=True)
                print(f'Samples:      {len(self.samples)} total, {len(cruise_samples)} cruise', flush=True)
                print(f'Efficiency:   {self.peak_vx/self.target_vx*100:.1f}%', flush=True)
        raise SystemExit(0)

def main():
    target = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
    rclpy.init()
    node = SpeedTest(target)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
