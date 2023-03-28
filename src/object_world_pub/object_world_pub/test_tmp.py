#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
from scout_interfaces_msg.msg import ObjectPixel
from scout_interfaces_msg.msg import ObjectWorld
from scout_interfaces_msg.msg import CameraIntrin

import message_filters
import ast
import numpy as np
from builtin_interfaces.msg import Time
import time
import tf_transformations
import math

class TestTmp(Node):
    def __init__(self):
        super().__init__('test_tmp_node')
        # b = self.get_clock().now().to_msg()
        # a = Time()
        # a = b
        # print(a)
        # c = tf_transformations.quaternion_matrix([0.06146124, 0, 0, 0.99810947])
        # print(type(c))
        # d = np.zeros((3,3))
        # print(type(d))
        # print(tf_transformations.translation_matrix([0.0, 0.0, 0.6]))
        # a = [-0.707, 0.0, 0.0, 0.707]
        # b = [0.0, 0.707, 0.0, 0.707]
        # print(tf_transformations.quaternion_multiply(b,a))
        # print(self.quatProduct(b,a))
        # q = tf_transformations.quaternion_about_axis(math.pi/2.0,(1,0,0))
        # print(q)
        
        q1 = tf_transformations.quaternion_about_axis(-math.pi/2.0, (1,0,0)) # 绕x旋转-90度
        q2 = tf_transformations.quaternion_about_axis(math.pi/2.0, (0,1,0)) # 绕y旋转90度
        q = tf_transformations.quaternion_multiply(q2, q1) # 先绕x旋转-90，再绕y旋转90    
        print(q1)    
        
        m = tf_transformations.quaternion_matrix(q1)  
        print(m)
    
    # 两个四元数相乘，q1*q2，计算方法来自维基百科；经测试，效果与tf_transformations.quaternion_multiply一样
    def quatProduct(self,q1, q2):
        b, c, d, a = q1 # x y z w
        x, y, z, t = q2
        r2 = q2[0]
        q = []
        q.append(b*t+a*x+d*y-c*z) # x
        q.append(c*t+a*y+b*z-d*x) # y
        q.append(d*t+z*a+c*x-b*y) # z
        q.append(a*t-b*x-c*y-d*z) # w
        return q

def main():
    rclpy.init()
    node = TestTmp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()      
    