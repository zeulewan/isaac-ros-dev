#!/usr/bin/env python3
"""Relay /cmd_vel → /cmd_vel_raw (pass-through).

Isaac Sim's OmniGraph differential drive is remapped to read /cmd_vel_raw.
This relay bridges Nav2/teleop output on /cmd_vel to the drive input.
No sign changes — base_link +X = physical forward.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cb, 10)

    def cb(self, msg):
        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(CmdVelRelay())


if __name__ == '__main__':
    main()
