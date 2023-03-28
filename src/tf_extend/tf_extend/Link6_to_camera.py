#!/usr/bin/python3
# -*- coding: utf-8 -*-

from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

class Link62CameraDemo(Node):
    def __init__(self):
        super().__init__('Link6_to_camera_broad')
        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.5, self.broadcast_timer_callback)
        
    def broadcast_timer_callback(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'Link6'
        t.child_frame_id = 'camera_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.06
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = Link62CameraDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()