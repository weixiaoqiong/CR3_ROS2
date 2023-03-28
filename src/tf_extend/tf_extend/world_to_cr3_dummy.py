#!/usr/bin/python3
# -*- coding: utf-8 -*-

from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

class World2Cr3DummyDemo(Node):
    def __init__(self):
        super().__init__('world_to_cr3_dummy_broad')
        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.5, self.broadcast_timer_callback)
        
    def broadcast_timer_callback(self):
        world_to_cr3_dummy = TransformStamped()
        world_to_cr3_dummy.header.stamp = self.get_clock().now().to_msg()
        world_to_cr3_dummy.header.frame_id = 'odom'
        world_to_cr3_dummy.child_frame_id = 'base_link'
        world_to_cr3_dummy.transform.translation.x = 0.0
        world_to_cr3_dummy.transform.translation.y = 0.0
        world_to_cr3_dummy.transform.translation.z = 0.0
        world_to_cr3_dummy.transform.rotation.x = 0.0
        world_to_cr3_dummy.transform.rotation.y = 0.0
        world_to_cr3_dummy.transform.rotation.z = 0.0
        world_to_cr3_dummy.transform.rotation.w = 1.0

        self.br.sendTransform(world_to_cr3_dummy)


def main():
    rclpy.init()
    node = World2Cr3DummyDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()