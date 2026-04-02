#!/usr/bin/env python3
"""Bridge sim camera (ZMQ) to ROS 2 Image topic for Foxglove.

Reads JPEG frames from unitree_sim_isaaclab's image server via ZMQ
and publishes them as ROS 2 CompressedImage messages.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import zmq


class CameraBridge(Node):
    def __init__(self):
        super().__init__('camera_bridge')
        self.declare_parameter('zmq_host', 'localhost')
        self.declare_parameter('zmq_port', 55555)
        self.declare_parameter('topic', '/head_camera/image/compressed')

        host = self.get_parameter('zmq_host').value
        port = self.get_parameter('zmq_port').value
        topic = self.get_parameter('topic').value

        self.pub = self.create_publisher(CompressedImage, topic, 10)

        self.zmq_ctx = zmq.Context()
        self.zmq_sub = self.zmq_ctx.socket(zmq.SUB)
        self.zmq_sub.connect(f'tcp://{host}:{port}')
        self.zmq_sub.setsockopt(zmq.SUBSCRIBE, b'')
        self.zmq_sub.setsockopt(zmq.RCVTIMEO, 1000)
        self.zmq_sub.setsockopt(zmq.CONFLATE, 1)  # only keep latest frame

        self.timer = self.create_timer(1.0 / 30.0, self.poll_frame)  # 30 Hz
        self.get_logger().info(f'Bridging zmq://localhost:{port} -> {topic}')

    def poll_frame(self):
        try:
            frame_data = self.zmq_sub.recv(zmq.NOBLOCK)
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'head_camera'
            msg.format = 'jpeg'
            msg.data = frame_data
            self.pub.publish(msg)
        except zmq.Again:
            pass


def main():
    rclpy.init()
    rclpy.spin(CameraBridge())


if __name__ == '__main__':
    main()
