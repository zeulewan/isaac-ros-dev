#!/usr/bin/env python3
"""Bridge /chassis/odom → odom→base_link TF broadcast.

Isaac Sim does not publish the odom→base_link transform via TF.
This node subscribes to /chassis/odom and broadcasts the transform.
base_link +X = physical forward (camera/sensor bar side).
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomTFBridge(Node):
    def __init__(self):
        super().__init__('odom_tf_bridge')
        self.br = TransformBroadcaster(self)
        self.sub = self.create_subscription(Odometry, '/chassis/odom', self.cb, 10)

    def cb(self, msg):
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)


def main():
    rclpy.init()
    rclpy.spin(OdomTFBridge())


if __name__ == '__main__':
    main()
