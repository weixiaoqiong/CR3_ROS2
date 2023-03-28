#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node

from scout_interfaces_msg.msg import ObjectPixel

import random

from builtin_interfaces.msg import Time

class PixelFakePub(Node):

    def __init__(self):
        super().__init__('pixel_fake_pub')
        self.publisher_ = self.create_publisher(ObjectPixel, 'object_pixel_info', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = ObjectPixel()
        
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.pixel1_label = 'human'
        msg.pixel1_stamp.sec = msg.header.stamp.sec
        dis = random.randint(0,msg.header.stamp.nanosec-1)
        msg.pixel1_stamp.nanosec = dis
        msg.pixel1_u1 = 50.0
        msg.pixel1_v1 = 60.0
        msg.pixel1_u2 = 60.0
        msg.pixel1_v2 = 70.0
                
        msg.pixel2_label = 'human'
        msg.pixel2_stamp = msg.header.stamp
        msg.pixel2_u1 = 40.0
        msg.pixel2_v1 = 50.0
        msg.pixel2_u2 = 50.0
        msg.pixel2_v2 = 60.0
                
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    node = PixelFakePub()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    
    # PixelFakePub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()