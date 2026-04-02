#!/usr/bin/env python3
"""Bridge sim lidar (ZMQ PULL) to ROS 2 PointCloud2 topic.

Accumulates partial RTX lidar scans into full 360-degree rotations
before publishing. KISS-ICP needs complete scans to register properly.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import zmq
import numpy as np
import time

FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]


class LidarBridge(Node):
    def __init__(self):
        super().__init__('lidar_bridge')
        self.pub = self.create_publisher(PointCloud2, '/lidar/points', 10)

        self.zmq_ctx = zmq.Context()
        self.zmq_sub = self.zmq_ctx.socket(zmq.PULL)
        self.zmq_sub.setsockopt(zmq.RCVHWM, 3)
        self.zmq_sub.connect('tcp://localhost:55560')

        # Accumulation buffer
        self.accumulated = []
        self.accumulated_points = 0
        self.last_publish_time = time.time()
        self.target_points = 15000  # publish when we have enough points
        self.max_age = 0.5  # or after 0.5 seconds

        self.timer = self.create_timer(0.01, self.poll_frame)  # 100 Hz poll
        self.get_logger().info('Bridging zmq -> /lidar/points (accumulating full scans)')

    def poll_frame(self):
        try:
            # Drain all available frames
            while True:
                try:
                    raw = self.zmq_sub.recv(zmq.NOBLOCK)
                    self.accumulated.append(raw)
                    self.accumulated_points += len(raw) // 12
                except zmq.Again:
                    break

            # Publish when we have enough points or enough time has passed
            now = time.time()
            age = now - self.last_publish_time
            if self.accumulated_points >= self.target_points or (self.accumulated_points > 0 and age >= self.max_age):
                # Merge all accumulated frames
                merged = b''.join(self.accumulated)
                num_points = len(merged) // 12

                msg = PointCloud2()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'lidar'
                msg.height = 1
                msg.width = num_points
                msg.fields = FIELDS
                msg.is_bigendian = False
                msg.point_step = 12
                msg.row_step = len(merged)
                msg.data = merged
                msg.is_dense = True
                self.pub.publish(msg)

                self.accumulated = []
                self.accumulated_points = 0
                self.last_publish_time = now
        except Exception:
            pass


def main():
    rclpy.init()
    rclpy.spin(LidarBridge())


if __name__ == '__main__':
    main()
