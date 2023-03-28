from lib2to3.pytree import Node
from re import M
from turtle import end_fill
import cv2
from matplotlib import image
from matplotlib.pyplot import cla, magma
import numpy as np
from rosidl_generator_py import import_type_support
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError

class ImageUndistorDemo(Node):
    def __init__(self):
        super().__init__('image_undistor_node')
        self.image_subscriber = self.create_subscription(Image, '/image_raw', self.listener_callback, 10)
        self.undistor_image_publisher = self.create_publisher(Image, '/image_undistor', 10)      
        time_period = 0.04
        self.timer = self.create_timer(time_period, self.timer_callback)
        self.to_pub = Image()
        
        self.camera_intrin = np.array([[569.679182, 0.000000, 330.546262], [0.000000, 760.089471, 220.336716], [0.000000, 0.000000, 1.000000]])
        self.img_distor_coeff = np.array([-0.245506, 0.175812, -0.001919, -0.002584])
        
        self.bridge = CvBridge()
        
    def listener_callback(self, msg):
        
        cv_img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR)
        img_undistor = cv2.undistort(img, self.camera_intrin, self.img_distor_coeff)
        
        img_undistor_ros = self.bridge.cv2_to_imgmsg(img_undistor, encoding="bgr8")
        # self.to_pub = msg
        # msg = Image()
        self.to_pub.header.stamp.sec = msg.header.stamp.sec
        self.to_pub.header.stamp.nanosec = msg.header.stamp.nanosec
        self.to_pub.height = img_undistor_ros.height
        self.to_pub.width = img_undistor_ros.width
        self.to_pub.encoding = img_undistor_ros.encoding
        self.to_pub.is_bigendian = img_undistor_ros.is_bigendian
        self.to_pub.step = img_undistor_ros.step
        self.to_pub.data = img_undistor_ros.data
        
        
    def timer_callback(self):
        print(self.to_pub)
        self.undistor_image_publisher.publish(self.to_pub)
        
def main(args=None):
    rclpy.init(args=args)

    node = ImageUndistorDemo()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    
    # PixelFakePub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()