#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node

from scout_interfaces_msg.msg import CameraIntrin

class CamInfoFakePub(Node):

    def __init__(self):
        super().__init__('cam_info_fake_pub')
        self.publisher_ = self.create_publisher(CameraIntrin, 'camera_intrin_info', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = CameraIntrin()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.camera_matrix = [608.63231, 0.0, 298.48801, 0.0, 813.4077, 262.04761, 0.0, 0.0, 1.0]
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    node = CamInfoFakePub()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    
    # PixelFakePub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()