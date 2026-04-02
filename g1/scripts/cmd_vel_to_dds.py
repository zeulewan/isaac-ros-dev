#!/usr/bin/env python3
"""Bridge /cmd_vel (ROS 2) to rt/run_command/cmd (Unitree DDS).

Smooths commands with ramping. Publishes at steady 50 Hz to DDS.
Holds last command for 0.5s to handle Lichtblick's 10 Hz publish rate.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time


class CmdVelToDDS(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_dds')
        self.dds_pub = self.create_publisher(String, '/run_command/cmd', 10)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cb, 10)

        self.target = [0.0, 0.0, 0.0]
        self.current = [0.0, 0.0, 0.0]
        self.last_msg_time = time.time()

        self.ramp_up = [1.5, 1.0, 1.5]
        self.ramp_down = [3.0, 2.0, 3.0]

        self.timer = self.create_timer(0.02, self.publish_cmd)  # 50 Hz
        self.get_logger().info('Bridging /cmd_vel -> rt/run_command/cmd at 50 Hz')

    def cb(self, msg):
        self.target = [msg.linear.x, msg.linear.y, msg.angular.z]
        self.last_msg_time = time.time()

    def publish_cmd(self):
        dt = 0.02

        # Hold command for 0.5s after last message (Lichtblick sends at 10 Hz)
        if time.time() - self.last_msg_time > 0.5:
            self.target = [0.0, 0.0, 0.0]

        for i in range(3):
            diff = self.target[i] - self.current[i]
            if abs(diff) < 0.01:
                self.current[i] = self.target[i]
            elif diff > 0:
                self.current[i] += min(diff, self.ramp_up[i] * dt)
            else:
                self.current[i] += max(diff, -self.ramp_down[i] * dt)

        cmd = [self.current[0], self.current[1], self.current[2], 0.8]
        dds_msg = String()
        dds_msg.data = str(cmd)
        self.dds_pub.publish(dds_msg)


def main():
    rclpy.init()
    rclpy.spin(CmdVelToDDS())


if __name__ == '__main__':
    main()
