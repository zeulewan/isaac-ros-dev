#!/usr/bin/env python3
"""Bridge FAST-LIO odometry to Nav2-compatible TF.

FAST-LIO publishes: camera_init -> body
Nav2 expects: map -> odom -> base_link

This node:
- Publishes odom -> base_link from FAST-LIO's /Odometry
- Publishes a static map -> odom (identity, since FAST-LIO is the map frame)
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class OdomTFBridge(Node):
    def __init__(self):
        super().__init__('odom_tf_bridge')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)

        # Static: map -> odom (identity -- FAST-LIO's camera_init IS the map frame)
        static_tf = TransformStamped()
        static_tf.header.stamp = self.get_clock().now().to_msg()
        static_tf.header.frame_id = 'map'
        static_tf.child_frame_id = 'odom'
        static_tf.transform.rotation.w = 1.0
        self.static_tf.sendTransform(static_tf)

        # Subscribe to FAST-LIO odometry
        self.sub = self.create_subscription(Odometry, '/Odometry', self.odom_cb, 10)

        # Also republish as /odom for Nav2
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        self.get_logger().info('Bridging /Odometry -> odom->base_link TF + /odom')

    def odom_cb(self, msg):
        # Publish odom -> base_link TF
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

        # Republish as /odom with correct frame IDs
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose = msg.pose
        odom.twist = msg.twist
        self.odom_pub.publish(odom)


def main():
    rclpy.init()
    rclpy.spin(OdomTFBridge())


if __name__ == '__main__':
    main()
