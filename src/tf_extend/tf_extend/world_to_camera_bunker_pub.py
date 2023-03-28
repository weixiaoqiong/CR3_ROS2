#!/usr/bin/python3
# -*- coding: utf-8 -*-
from multiprocessing import dummy
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Time

class W2CPubDemo(Node):
    def __init__(self):
        super().__init__('bunker_w2c_pub')
        
        self.declare_parameter('from_frame', 'odom')
        self.from_frame = self.get_parameter('from_frame').get_parameter_value().string_value
        self.declare_parameter('to_frame', 'camera_link')
        self.to_frame = self.get_parameter('to_frame').get_parameter_value().string_value
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.w2c_publisher = self.create_publisher(TransformStamped, 'w2c_transform', 50)
        
        self.timer = self.create_timer(0.1, self.on_timer)  # 1/50
        
    def on_timer(self):
        
        try:
            # now = self.get_clock().now().to_msg()
            time0 = Time()
            time0.sec = 0
            time0.nanosec = 0            
            #print(now)
            trans = self.tf_buffer.lookup_transform(
                self.to_frame,
                self.from_frame,
                time0,
                timeout=Duration(seconds=5.0)
            )
            # type: TransformStamped()
            # print(trans)
            # print(trans)
           
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {self.from_frame} to {self.to_frame}: {ex}'
            )
            return
        print(trans)
        self.w2c_publisher.publish(trans) 

def main(args=None):
    rclpy.init(args=args)
    cr3_w2c_pub = W2CPubDemo()
    rclpy.spin(cr3_w2c_pub)

if __name__=="__main__":
    main()
    
        
        