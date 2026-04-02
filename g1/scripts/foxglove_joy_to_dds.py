#!/usr/bin/env python3
"""Bridge Foxglove joystick to Unitree DDS.

Runs on the HOST (not in Docker) because it needs unitree_sdk2py.
Subscribes to /cmd_vel (ROS 2, domain 1) and publishes to rt/run_command/cmd (DDS).
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_


class FoxgloveJoyToDDS(Node):
    def __init__(self):
        super().__init__('foxglove_joy_to_dds')
        ChannelFactoryInitialize(1)
        self.dds_pub = ChannelPublisher('rt/run_command/cmd', String_)
        self.dds_pub.Init()
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cb, 10)
        self.get_logger().info('Bridging /cmd_vel (ROS 2) -> rt/run_command/cmd (Unitree DDS)')

    def cb(self, msg):
        cmd = [msg.linear.x, msg.linear.y, msg.angular.z, 0.8]
        dds_msg = String_(data=str(cmd))
        self.dds_pub.Write(dds_msg)


def main():
    rclpy.init()
    node = FoxgloveJoyToDDS()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
