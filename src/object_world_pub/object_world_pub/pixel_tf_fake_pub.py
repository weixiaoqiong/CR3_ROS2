#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node

from scout_interfaces_msg.msg import ObjectPixelTfMixed
from geometry_msgs.msg import Transform

import random
import math

class PixelTfFakePub(Node):

    def __init__(self):
        super().__init__('pixel_fake_pub')
        self.publisher_ = self.create_publisher(ObjectPixelTfMixed, 'object_pixel_tf_mixed_info', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = ObjectPixelTfMixed()
        
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.pixel1_label = 'human'
        msg.pixel1_stamp.sec = msg.header.stamp.sec
        dis = random.randint(0,msg.header.stamp.nanosec)
        msg.pixel1_stamp.nanosec = msg.header.stamp.nanosec - dis
        msg.pixel1_u1 = 50.0
        msg.pixel1_v1 = 60.0
        msg.pixel1_u2 = 60.0
        msg.pixel1_v2 = 70.0
        msg.pixel1_w2c = Transform()
        msg.pixel1_w2c.translation.x = 0.0
        msg.pixel1_w2c.translation.y = 0.0
        msg.pixel1_w2c.translation.z = 0.6
        roat_axis = [1, 0, 0] # 旋转轴
        roat_angle = 60 # 旋转角度
        msg.pixel1_w2c.rotation.x = roat_axis[0] * math.sin(roat_angle/2.0*math.pi/180.0)
        msg.pixel1_w2c.rotation.y = roat_axis[1] * math.sin(roat_angle/2.0*math.pi/180.0)
        msg.pixel1_w2c.rotation.z = roat_axis[2] * math.sin(roat_angle/2.0*math.pi/180.0)
        msg.pixel1_w2c.rotation.w = math.cos(roat_angle/2.0*math.pi/180.0)
                
        msg.pixel2_label = 'human'
        msg.pixel2_stamp = msg.header.stamp
        msg.pixel2_u1 = 40.0
        msg.pixel2_v1 = 50.0
        msg.pixel2_u2 = 50.0
        msg.pixel2_v2 = 60.0
        msg.pixel2_w2c = Transform()
        msg.pixel2_w2c.translation.x = 0.0
        msg.pixel2_w2c.translation.y = 0.0
        msg.pixel2_w2c.translation.z = 0.5
        roat_axis2 = [1, 0, 0] # 旋转轴
        roat_angle2 = 30 # 旋转角度
        msg.pixel2_w2c.rotation.x = roat_axis2[0] * math.sin(roat_angle2/2.0*math.pi/180.0)
        msg.pixel2_w2c.rotation.y = roat_axis2[1] * math.sin(roat_angle2/2.0*math.pi/180.0)
        msg.pixel2_w2c.rotation.z = roat_axis2[2] * math.sin(roat_angle2/2.0*math.pi/180.0)
        msg.pixel2_w2c.rotation.w = math.cos(roat_angle2/2.0*math.pi/180.0)        
        
                
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    node = PixelTfFakePub()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    
    # PixelFakePub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()