#!/usr/bin/python3
# -*- coding: utf-8 -*-
from __future__ import print_function
from tokenize import String

from numpy import int32
import cr3_scout_with_srvs.cr3_service_clients
import rclpy
import time
from cr3_scout_with_srvs import pelco_cam

import ast
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
# import os, sys
# sys.path.append(os.getcwd())
from scout_interfaces_msg.msg import CameraIntrin
from std_msgs.msg import String


class CamIntrinInfoDemo(Node):

    def __init__(self):
        super().__init__('camera_intrin_info_pub')
        self.listener_ = self.create_subscription(String, 'current_zoom', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(CameraIntrin, 'camera_intrin_info', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
        self.zoom = '11'
        self.intrin_mat = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.declare_parameter(name='zoom00')         
        self.declare_parameter(name='zoom01')
        self.declare_parameter(name='zoom02')
        self.declare_parameter(name='zoom03')
        self.declare_parameter(name='zoom04')
        self.declare_parameter(name='zoom05')
        self.declare_parameter(name='zoom06')
        self.declare_parameter(name='zoom07')
        self.declare_parameter(name='zoom08')
        self.declare_parameter(name='zoom09')
        self.declare_parameter(name='zoom10')
        self.declare_parameter(name='zoom11')

    def listener_callback(self, msg):
        self.zoom = msg.data
     
    def timer_callback(self):
        
        # 2. yaml;
        para_name = 'zoom'+self.zoom
        # print(para_name)
        para = self.get_parameter(name=para_name).get_parameter_value().string_value
        print("para is: {}".format(para))
        para_dict = {}
        print(para)
        para_dict = ast.literal_eval(para)
        self.zoom = para_dict['zoom']
        self.intrin_mat = para_dict['camera_matrix']  
        
              
        msg = CameraIntrin()
        msg.header.stamp = self.get_clock().now().to_msg()
        # msg.zoom = '00'
        # msg.camera_matrix = [608.63231, 0.0, 298.48801, 0.0, 813.4077, 262.04761, 0.0, 0.0, 1.0]
        msg.zoom = self.zoom
        msg.camera_matrix = self.intrin_mat
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        self.get_logger().info('Publishing zoom: "%s"' % msg.zoom)
        #print(msg.camera_matrix)
        self.get_logger().info('Publishing camera_matrix: "%s"' % str(msg.camera_matrix))


def main(args=None):
    rclpy.init(args=args)

    node = CamIntrinInfoDemo()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    
    # PixelFakePub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
