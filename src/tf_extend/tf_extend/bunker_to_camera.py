#!/usr/bin/python3
# -*- coding: utf-8 -*-

from numpy import mat
from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

import tf_transformations
import math

class Bunker2CameraDemo(Node):
    def __init__(self):
        super().__init__('Bunker_to_camera_broad')
        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_timer_callback)
        
    def broadcast_timer_callback(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'camera'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.666
        
        q1 = tf_transformations.quaternion_about_axis(-math.pi/2.0, (1,0,0)) # 绕x旋转-90度
        q2 = tf_transformations.quaternion_about_axis(math.pi/2.0, (0,1,0)) # 绕y旋转90度
        q = tf_transformations.quaternion_multiply(q2, q1) # 先绕x旋转-90，再绕y旋转90        
        
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        


        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = Bunker2CameraDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()