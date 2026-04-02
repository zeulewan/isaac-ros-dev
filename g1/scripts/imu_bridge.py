#!/usr/bin/env python3
"""Bridge sim IMU (ZMQ) to ROS 2 sensor_msgs/Imu topic for SLAM.

Reads [ang_vel(3), lin_acc(3), quat(4)] from ZMQ and publishes as Imu messages.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import zmq
import numpy as np
import struct


class ImuBridge(Node):
    def __init__(self):
        super().__init__('imu_bridge')
        self.pub = self.create_publisher(Imu, '/imu/data', 50)

        self.zmq_ctx = zmq.Context()
        self.zmq_sub = self.zmq_ctx.socket(zmq.SUB)
        self.zmq_sub.connect('tcp://localhost:55570')
        self.zmq_sub.setsockopt(zmq.SUBSCRIBE, b'')
        self.zmq_sub.setsockopt(zmq.RCVTIMEO, 1000)
        self.zmq_sub.setsockopt(zmq.CONFLATE, 1)

        self.timer = self.create_timer(1.0 / 200.0, self.poll)  # 200 Hz
        self.get_logger().info('Bridging zmq://localhost:55570 -> /imu/data')

    def poll(self):
        try:
            raw = self.zmq_sub.recv(zmq.NOBLOCK)
            data = np.frombuffer(raw, dtype=np.float32)
            if len(data) == 10:
                msg = Imu()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'imu_link'

                # Angular velocity
                msg.angular_velocity.x = float(data[0])
                msg.angular_velocity.y = float(data[1])
                msg.angular_velocity.z = float(data[2])

                # Linear acceleration
                msg.linear_acceleration.x = float(data[3])
                msg.linear_acceleration.y = float(data[4])
                msg.linear_acceleration.z = float(data[5])

                # Orientation (w,x,y,z)
                msg.orientation.w = float(data[6])
                msg.orientation.x = float(data[7])
                msg.orientation.y = float(data[8])
                msg.orientation.z = float(data[9])

                self.pub.publish(msg)
        except zmq.Again:
            pass


def main():
    rclpy.init()
    rclpy.spin(ImuBridge())


if __name__ == '__main__':
    main()
