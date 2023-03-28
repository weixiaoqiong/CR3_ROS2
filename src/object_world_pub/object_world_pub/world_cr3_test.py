#!/usr/bin/python3
# -*- coding: utf-8 -*-
from http.client import MOVED_PERMANENTLY
from mimetypes import init
import re
from time import sleep
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
from scout_interfaces_msg.msg import ObjectPixelTfMixed5
from rclpy.duration import Duration

import message_filters
import ast
import numpy as np
import tf_transformations

from builtin_interfaces.msg import Time
import matplotlib.pyplot as plt

class ObjectWorldPubNewDemo(Node):
    def __init__(self):
        super().__init__('object_world_pub')
        
        self.object_subscriber = self.create_subscription(ObjectPixelTfMixed5, 'object_pixel_tf_mixed_info5', self.object_world_compute, 50)
        
        # self.object_subscriber = message_filters.Subscriber(self, ObjectPixelTfMixed, 'object_pixel_tf_mixed_info')      
        # self.intrin_matrix_subscriber = message_filters.Subscriber(self, CameraIntrin, 'camera_intrin_info')
        
        print("ready")
        # tss = message_filters.ApproximateTimeSynchronizer([self.object_subscriber, self.intrin_matrix_subscriber], 10, 0.05)
        # tss.registerCallback(self.object_world_compute)
        
        self.object_world_pub = self.create_publisher(ObjectWorld, 'object_world_info', 10)   
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.object_world_pub_callback)        
        
        self.to_pub = ObjectWorld()  
        self.to_pub.header.stamp = self.get_clock().now().to_msg()
        self.to_pub.object_label = 'test'
        self.to_pub.object_world_x = 0.0
        self.to_pub.object_world_y = 0.0
        self.to_pub.object_world_z = 0.0     
        
        self.para_name = 'zoom00'
        # print(para_name)
        self.declare_parameter(name=self.para_name)    
        # self.object_name = "person_standing_p1"   
        self.object_name = "person_standing_p3"
        self.declare_parameter(name=self.object_name)   
        
        self.pixel_width = 640 
        self.pixel_height = 480

        self.person_ratio = 0.3
        self.pixel_u_upper = 0.75*self.pixel_width
        self.pixel_v_lower = 0.25*self.pixel_width
        
        self.plt_x = []
        self.plt_y = []
        self.count = 0
        self.coeff = np.zeros((2,4))
        self.right = np.zeros((2,1))
    
    
    # def object_sub_callback(self,msg):
    #     print(msg)
        
    # def int_mat_sub_callback(self, msg):
    #     print(msg)
        
    def object_world_compute(self, pixel_msg):
        
        if ((pixel_msg.pixel1_label == '') or (pixel_msg.pixel2_label =='')) or (pixel_msg.pixel1_label != pixel_msg.pixel2_label):
            # self.to_pub.header.stamp = self.get_clock().now().to_msg()
            self.to_pub.header= pixel_msg.header
            self.to_pub.object_label = ""
            self.to_pub.object_world_x = 0.0
            self.to_pub.object_world_y = 0.0
            self.to_pub.object_world_z = 0.0
            # print(self.to_pub)
            self.object_world_pub.publish(self.to_pub)  
            # print("OK")
        else:
            self.count += 1
            if self.count == 1:
                # 计算world_to_camera 齐次变换矩阵  
                # tf_extend.world_to_camera_pub中获取的四元数参考坐标系是"camera"，因此转化的矩阵就是从world到camera的
                w2c_1 = pixel_msg.pixel1_w2c 
                w2c_2 = pixel_msg.pixel2_w2c
                w2c_3 = pixel_msg.pixel3_w2c
                w2c_4 = pixel_msg.pixel4_w2c
                w2c_5 = pixel_msg.pixel5_w2c

                w2c_1_ls = []
                w2c_1_ls.append(w2c_1.rotation.x)
                w2c_1_ls.append(w2c_1.rotation.y)
                w2c_1_ls.append(w2c_1.rotation.z)
                w2c_1_ls.append(w2c_1.rotation.w)
                w2c_2_ls = []
                w2c_2_ls.append(w2c_2.rotation.x)
                w2c_2_ls.append(w2c_2.rotation.y)
                w2c_2_ls.append(w2c_2.rotation.z)
                w2c_2_ls.append(w2c_2.rotation.w)  
                w2c_3_ls = []
                w2c_3_ls.append(w2c_3.rotation.x)
                w2c_3_ls.append(w2c_3.rotation.y)
                w2c_3_ls.append(w2c_3.rotation.z)
                w2c_3_ls.append(w2c_3.rotation.w)    
                w2c_4_ls = []
                w2c_4_ls.append(w2c_4.rotation.x)
                w2c_4_ls.append(w2c_4.rotation.y)
                w2c_4_ls.append(w2c_4.rotation.z)
                w2c_4_ls.append(w2c_4.rotation.w)                                    
                w2c_5_ls = []
                w2c_5_ls.append(w2c_5.rotation.x)
                w2c_5_ls.append(w2c_5.rotation.y)
                w2c_5_ls.append(w2c_5.rotation.z)
                w2c_5_ls.append(w2c_5.rotation.w)                                                
                w2c_1_matrix = tf_transformations.quaternion_matrix(w2c_1_ls) # 已经是numpy数组了
                w2c_2_matrix = tf_transformations.quaternion_matrix(w2c_2_ls)
                w2c_3_matrix = tf_transformations.quaternion_matrix(w2c_3_ls)
                w2c_4_matrix = tf_transformations.quaternion_matrix(w2c_4_ls)
                w2c_5_matrix = tf_transformations.quaternion_matrix(w2c_5_ls)
                # tf_transformations.translation_matrix() 
                # 可以直接将平移向量转化为齐次矩阵，平移齐次矩阵乘以旋转齐次矩阵就得到了最终的齐次变换矩阵
                # 因为这种方法还要先建列表，也并不多简单，因此直接采取下面的方法修改
                w2c_1_matrix[0][3] = w2c_1.translation.x
                w2c_1_matrix[1][3] = w2c_1.translation.y
                w2c_1_matrix[2][3] = w2c_1.translation.z
                w2c_2_matrix[0][3] = w2c_2.translation.x
                w2c_2_matrix[1][3] = w2c_2.translation.y
                w2c_2_matrix[2][3] = w2c_2.translation.z
                w2c_3_matrix[0][3] = w2c_3.translation.x
                w2c_3_matrix[1][3] = w2c_3.translation.y
                w2c_3_matrix[2][3] = w2c_3.translation.z
                w2c_4_matrix[0][3] = w2c_4.translation.x
                w2c_4_matrix[1][3] = w2c_4.translation.y
                w2c_4_matrix[2][3] = w2c_4.translation.z                        
                w2c_5_matrix[0][3] = w2c_5.translation.x
                w2c_5_matrix[1][3] = w2c_5.translation.y
                w2c_5_matrix[2][3] = w2c_5.translation.z                                    
                
                # print("w2c_1_matrix is: {}".format(w2c_1_matrix))
                # print("w2c_2_matrix is: {}".format(w2c_2_matrix))
                    
                mod_matrix = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, -0.065], [0.0, 0.0, 1.0, -0.124], [0.0, 0.0, 0.0, 1.0]]) # 末端与摄像头没有旋转，只有平移
                
                # 处理像素信息
                p = ObjectPixel()
                object_pixel1 = np.zeros((2,1))
                object_pixel1[0][0] = pixel_msg.pixel1_u2 # (pixel_msg.pixel1_u2+pixel_msg.pixel1_u1)/2.0
                object_pixel1[1][0] = pixel_msg.pixel1_v2
                object_pixel2 = np.zeros((2,1))
                object_pixel2[0][0] = pixel_msg.pixel2_u2 # (pixel_msg.pixel2_u2+pixel_msg.pixel2_u1)/2.0
                object_pixel2[1][0] = pixel_msg.pixel2_v2 
                object_pixel3 = np.zeros((2,1))
                object_pixel3[0][0] = pixel_msg.pixel3_u2 # (pixel_msg.pixel3_u2+pixel_msg.pixel3_u1)/2.0
                object_pixel3[1][0] = pixel_msg.pixel3_v2 
                object_pixel4 = np.zeros((2,1))
                object_pixel4[0][0] = pixel_msg.pixel4_u2 # (pixel_msg.pixel4_u2+pixel_msg.pixel4_u1)/2.0
                object_pixel4[1][0] = pixel_msg.pixel4_v2             
                object_pixel5 = np.zeros((2,1))
                object_pixel5[0][0] = pixel_msg.pixel5_u2 # (pixel_msg.pixel5_u2+pixel_msg.pixel5_u1)/2.0
                object_pixel5[1][0] = pixel_msg.pixel5_v2   
                
                # # 处理像素信息
                # p = ObjectPixel()
                # object_pixel1 = np.zeros((2,1))
                # object_pixel1[0][0] = (pixel_msg.pixel1_u2+pixel_msg.pixel1_u1)/2.0
                # object_pixel1[1][0] = (pixel_msg.pixel1_v2+pixel_msg.pixel1_v1)/2.0
                # object_pixel2 = np.zeros((2,1))
                # object_pixel2[0][0] = (pixel_msg.pixel2_u2+pixel_msg.pixel2_u1)/2.0
                # object_pixel2[1][0] = (pixel_msg.pixel2_v2+pixel_msg.pixel1_v1)/2.0
                # object_pixel3 = np.zeros((2,1))
                # object_pixel3[0][0] = (pixel_msg.pixel3_u2+pixel_msg.pixel3_u1)/2.0
                # object_pixel3[1][0] = (pixel_msg.pixel3_v2+pixel_msg.pixel3_v1)/2.0 
                # object_pixel4 = np.zeros((2,1))
                # object_pixel4[0][0] = (pixel_msg.pixel4_u2+pixel_msg.pixel4_u1)/2.0
                # object_pixel4[1][0] = (pixel_msg.pixel4_v2+pixel_msg.pixel4_v1)/2.0             
                # object_pixel5 = np.zeros((2,1))
                # object_pixel5[0][0] = (pixel_msg.pixel5_u2+pixel_msg.pixel5_u1)/2.0
                # object_pixel5[1][0] = (pixel_msg.pixel5_v2+pixel_msg.pixel5_v1)/2.0                                       
                
                pixel1_ratio1 = (pixel_msg.pixel1_u2-pixel_msg.pixel1_u1)/self.pixel_width
                pixel1_ratio2 = (pixel_msg.pixel1_v2-pixel_msg.pixel1_v1)/self.pixel_height
                pixel2_ratio1 = (pixel_msg.pixel2_u2-pixel_msg.pixel2_u1)/self.pixel_width
                pixel2_ratio2 = (pixel_msg.pixel2_v2-pixel_msg.pixel2_v1)/self.pixel_height  
                pixel3_ratio1 = (pixel_msg.pixel3_u2-pixel_msg.pixel3_u1)/self.pixel_width
                pixel3_ratio2 = (pixel_msg.pixel3_v2-pixel_msg.pixel3_v1)/self.pixel_height                       
                pixel4_ratio1 = (pixel_msg.pixel4_u2-pixel_msg.pixel4_u1)/self.pixel_width
                pixel4_ratio2 = (pixel_msg.pixel4_v2-pixel_msg.pixel4_v1)/self.pixel_height                                   
                pixel5_ratio1 = (pixel_msg.pixel5_u2-pixel_msg.pixel5_u1)/self.pixel_width
                pixel5_ratio2 = (pixel_msg.pixel5_v2-pixel_msg.pixel5_v1)/self.pixel_height   
                pixel_ratio_ls = []
                pixel_ratio_ls.append(pixel1_ratio1)
                pixel_ratio_ls.append(pixel1_ratio2)
                pixel_ratio_ls.append(pixel2_ratio1)
                pixel_ratio_ls.append(pixel2_ratio2)
                pixel_ratio_ls.append(pixel3_ratio1)
                pixel_ratio_ls.append(pixel3_ratio2)
                pixel_ratio_ls.append(pixel4_ratio1)
                pixel_ratio_ls.append(pixel4_ratio2)            
                pixel_ratio_ls.append(pixel5_ratio1)
                pixel_ratio_ls.append(pixel5_ratio2)                        
                print("Minimal pixel ratio is: {}".format(min(pixel_ratio_ls)))                                                        

                # # 根据实际世界坐标、变换矩阵、以及像素坐标，计算内参矩阵
                
                # person_ = self.get_parameter(name=self.object_name).get_parameter_value().string_value
                # person_dict = {}
                # person_dict = ast.literal_eval(person_)
                # person_position_ = np.array(person_dict['position'])
                # person_position = person_position_.reshape((3,1))           

                # real_world = np.append(person_position, [[1.0]], axis=0)   
                # real_camera_1 = np.dot(mod_matrix,np.dot(w2c_1_matrix, real_world))
                # real_camera_2 = np.dot(mod_matrix,np.dot(w2c_2_matrix, real_world))
                # real_camera_3 = np.dot(mod_matrix,np.dot(w2c_3_matrix, real_world))
                # real_camera_4 = np.dot(mod_matrix,np.dot(w2c_4_matrix, real_world))
                # real_camera_5 = np.dot(mod_matrix,np.dot(w2c_5_matrix, real_world))
                # print(real_camera_1)
                # print(real_camera_2)
                # print(real_camera_3)
                # print(real_camera_4)
                # print(real_camera_5)
                # intrin_matrix_compute_coeff1 = self.intrin_matrix_compute_coeff(real_camera_1)    
                # intrin_matrix_compute_coeff2 = self.intrin_matrix_compute_coeff(real_camera_2)
                # intrin_matrix_compute_coeff3 = self.intrin_matrix_compute_coeff(real_camera_3)
                # intrin_matrix_compute_coeff4 = self.intrin_matrix_compute_coeff(real_camera_4)
                # intrin_matrix_compute_coeff5 = self.intrin_matrix_compute_coeff(real_camera_5)
                # intrin_matrix_compute_coeff_tmp1 = np.append(intrin_matrix_compute_coeff1, intrin_matrix_compute_coeff2, axis=0)
                # intrin_matrix_compute_coeff_tmp2 = np.append(intrin_matrix_compute_coeff_tmp1, intrin_matrix_compute_coeff3, axis=0)
                # intrin_matrix_compute_coeff_tmp3 = np.append(intrin_matrix_compute_coeff_tmp2, intrin_matrix_compute_coeff4, axis=0)
                # intrin_matrix_compute_coeff = np.append(intrin_matrix_compute_coeff_tmp3, intrin_matrix_compute_coeff5, axis=0)
                # intrin_matrix_compute_right_tmp1 = np.append(object_pixel1, object_pixel2, axis=0)
                # intrin_matrix_compute_right_tmp2 = np.append(intrin_matrix_compute_right_tmp1, object_pixel3, axis=0)
                # intrin_matrix_compute_right_tmp3 = np.append(intrin_matrix_compute_right_tmp2, object_pixel4, axis=0)
                # intrin_matrix_compute_right = np.append(intrin_matrix_compute_right_tmp3, object_pixel5, axis=0)
                # self.coeff = intrin_matrix_compute_coeff
                # self.right = intrin_matrix_compute_right
                
                # intrin_matrix_compute = np.dot(np.linalg.pinv(self.coeff), self.right)
                # print("intrin_matrix is computed as: {}".format(intrin_matrix_compute))
                
                # 根据标定内参、标定畸变系数、坐标变换、真实世界坐标，计算畸变后的像素坐标
                
                # 首先读取内参与畸变系数
                para = self.get_parameter(name=self.para_name).get_parameter_value().string_value
                # print("para is: {}".format(para))
                para_dict = {}
                para_dict = ast.literal_eval(para)
                cam_tmp1 = np.array(para_dict['camera_matrix'])          
                # cam_tmp1 = np.array(camera_msg.camera_matrix)
                cam_tmp2 = cam_tmp1.reshape((3,3))
                intrin_matrix = np.append(cam_tmp2, np.array([[0.0], [0.0], [0.0]]), axis=1)   
                distor_coeff_tmp = np.array(para_dict['distor_coeff'])  
                distor_coeff = distor_coeff_tmp.reshape((5,1))
                fx = intrin_matrix[0][0]
                fy = intrin_matrix[1][1]
                cx = intrin_matrix[0][2]
                cy = intrin_matrix[1][2]
                k1 = distor_coeff[0][0]
                k2 = distor_coeff[1][0]
                p1 = distor_coeff[2][0]
                p2 = distor_coeff[3][0]
                k3 = distor_coeff[4][0]
                
                person_ = self.get_parameter(name=self.object_name).get_parameter_value().string_value
                person_dict = {}
                person_dict = ast.literal_eval(person_)
                person_position_ = np.array(person_dict['position'])
                person_position = person_position_.reshape((3,1))           

                real_world = np.append(person_position, [[1.0]], axis=0)   
                real_camera_1 = np.dot(mod_matrix,np.dot(w2c_1_matrix, real_world))
                real_camera_2 = np.dot(mod_matrix,np.dot(w2c_2_matrix, real_world))
                real_camera_3 = np.dot(mod_matrix,np.dot(w2c_3_matrix, real_world))
                real_camera_4 = np.dot(mod_matrix,np.dot(w2c_4_matrix, real_world))
                real_camera_5 = np.dot(mod_matrix,np.dot(w2c_5_matrix, real_world))
                                
                # 计算理想像素坐标
                ideal_pixel1 = np.zeros((3,1))
                ideal_pixel2 = np.zeros((3,1))
                ideal_pixel3 = np.zeros((3,1))
                ideal_pixel4 = np.zeros((3,1))
                ideal_pixel5 = np.zeros((3,1))
                zc_1 = real_camera_1[2][0]
                zc_2 = real_camera_2[2][0]
                zc_3 = real_camera_3[2][0]
                zc_4 = real_camera_4[2][0]
                zc_5 = real_camera_5[2][0]
                ideal_pixel1 = np.dot(intrin_matrix, real_camera_1)/zc_1
                ideal_pixel2 = np.dot(intrin_matrix, real_camera_2)/zc_2
                ideal_pixel3 = np.dot(intrin_matrix, real_camera_3)/zc_3
                ideal_pixel4 = np.dot(intrin_matrix, real_camera_4)/zc_4
                ideal_pixel5 = np.dot(intrin_matrix, real_camera_5)/zc_5
                
                # 计算畸变像素坐标
                distor_pixel1 = self.distor_pixel_generate(ideal_pixel1, fx, fy, cx, cy, k1, k2, p1, p2, k3)
                distor_pixel2 = self.distor_pixel_generate(ideal_pixel2, fx, fy, cx, cy, k1, k2, p1, p2, k3)
                distor_pixel3 = self.distor_pixel_generate(ideal_pixel3, fx, fy, cx, cy, k1, k2, p1, p2, k3)
                distor_pixel4 = self.distor_pixel_generate(ideal_pixel4, fx, fy, cx, cy, k1, k2, p1, p2, k3)
                distor_pixel5 = self.distor_pixel_generate(ideal_pixel5, fx, fy, cx, cy, k1, k2, p1, p2, k3)
                
                diff1 = distor_pixel1-object_pixel1
                diff1[0][0] = diff1[0][0]/(pixel_msg.pixel1_u2-pixel_msg.pixel1_u1)
                diff1[1][0] = diff1[1][0]/(pixel_msg.pixel1_v2-pixel_msg.pixel1_v1)
                diff2 = distor_pixel2-object_pixel2
                diff2[0][0] = diff2[0][0]/(pixel_msg.pixel2_u2-pixel_msg.pixel2_u1)
                diff2[1][0] = diff2[1][0]/(pixel_msg.pixel2_v2-pixel_msg.pixel2_v1)
                diff3 = distor_pixel3-object_pixel3
                diff3[0][0] = diff3[0][0]/(pixel_msg.pixel3_u2-pixel_msg.pixel3_u1)
                diff3[1][0] = diff3[1][0]/(pixel_msg.pixel3_v2-pixel_msg.pixel3_v1)                                
                diff4 = distor_pixel4-object_pixel4
                diff4[0][0] = diff4[0][0]/(pixel_msg.pixel4_u2-pixel_msg.pixel4_u1)
                diff4[1][0] = diff4[1][0]/(pixel_msg.pixel4_v2-pixel_msg.pixel4_v1)
                diff5 = distor_pixel5-object_pixel5
                diff5[0][0] = diff5[0][0]/(pixel_msg.pixel5_u2-pixel_msg.pixel5_u1)
                diff5[1][0] = diff5[1][0]/(pixel_msg.pixel5_v2-pixel_msg.pixel5_v1)                                
                
                print("distor_pixel1: {}; detect_pixel1: {}; diff1: {}".format(distor_pixel1, object_pixel1, diff1))
                print("distor_pixel2: {}; detect_pixel2: {}; diff2: {}".format(distor_pixel2, object_pixel2, diff2))
                print("distor_pixel3: {}; detect_pixel3: {}; diff3: {}".format(distor_pixel3, object_pixel3, diff3))
                print("distor_pixel4: {}; detect_pixel4: {}; diff4: {}".format(distor_pixel4, object_pixel4, diff4))
                print("distor_pixel5: {}; detect_pixel5: {}; diff5: {}".format(distor_pixel5, object_pixel5, diff5))
                
                # u_ratio1 = (object_pixel1[0][0]-320.0)
                # u_ratio2 = (object_pixel2[0][0]-320.0)
                # u_ratio3 = (object_pixel3[0][0]-320.0)
                # u_ratio4 = (object_pixel4[0][0]-320.0)
                # u_ratio5 = (object_pixel5[0][0]-320.0)
                
                # self.plt_x.append(u_ratio1)
                # self.plt_x.append(u_ratio2)
                # self.plt_x.append(u_ratio3)
                # self.plt_x.append(u_ratio4)
                # self.plt_x.append(u_ratio5)
                
                # self.plt_y.append(diff1[0][0])
                # self.plt_y.append(diff2[0][0])
                # self.plt_y.append(diff3[0][0])
                # self.plt_y.append(diff4[0][0])
                # self.plt_y.append(diff5[0][0])
                
                width = self.person_ratio*(pixel_msg.pixel5_v2-pixel_msg.pixel5_v1)
                self.pixel_u_lower = self.pixel_v_lower+width
                        
                
            else:
                # 计算world_to_camera 齐次变换矩阵  
                w2c_5 = pixel_msg.pixel5_w2c                                   
                w2c_5_ls = []
                w2c_5_ls.append(w2c_5.rotation.x)
                w2c_5_ls.append(w2c_5.rotation.y)
                w2c_5_ls.append(w2c_5.rotation.z)
                w2c_5_ls.append(w2c_5.rotation.w)                                                
                w2c_5_matrix = tf_transformations.quaternion_matrix(w2c_5_ls)                      
                w2c_5_matrix[0][3] = w2c_5.translation.x
                w2c_5_matrix[1][3] = w2c_5.translation.y
                w2c_5_matrix[2][3] = w2c_5.translation.z                                    
                    
                mod_matrix = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.065], [0.0, 0.0, 1.0, -0.124], [0.0, 0.0, 0.0, 1.0]]) # 末端与摄像头没有旋转，只有平移
                
                # 处理像素信息             
                object_pixel5 = np.zeros((2,1))
                object_pixel5[0][0] = pixel_msg.pixel5_u2 # (pixel_msg.pixel5_u2+pixel_msg.pixel5_u1)/2.0
                object_pixel5[1][0] = pixel_msg.pixel5_v2
                
                # # 处理像素信息             
                # object_pixel5 = np.zeros((2,1))
                # object_pixel5[0][0] = (pixel_msg.pixel5_u2+pixel_msg.pixel5_u1)/2.0
                # object_pixel5[1][0] = (pixel_msg.pixel5_v2+pixel_msg.pixel5_v1)/2.0                                                                                               

                # # 根据实际世界坐标、变换矩阵、以及像素坐标，计算内参矩阵
                
                # person_ = self.get_parameter(name=self.object_name).get_parameter_value().string_value
                # person_dict = {}
                # person_dict = ast.literal_eval(person_)
                # person_position_ = np.array(person_dict['position'])
                # person_position = person_position_.reshape((3,1))           

                # real_world = np.append(person_position, [[1.0]], axis=0)   
                # real_camera_5 = np.dot(mod_matrix,np.dot(w2c_5_matrix, real_world))
                # print(real_camera_5)
                # intrin_matrix_compute_coeff5 = self.intrin_matrix_compute_coeff(real_camera_5)
                # intrin_matrix_compute_coeff = np.append(self.coeff, intrin_matrix_compute_coeff5, axis=0)
                # intrin_matrix_compute_right = np.append(self.right, object_pixel5, axis=0)
                # self.coeff = intrin_matrix_compute_coeff
                # self.right = intrin_matrix_compute_right
                
                # intrin_matrix_compute = np.dot(np.linalg.pinv(self.coeff), self.right)
                # print("intrin_matrix is computed as: {}".format(intrin_matrix_compute))  
                
                # 根据标定内参、标定畸变系数、坐标变换、真实世界坐标，计算畸变后的像素坐标
                
                # 首先读取内参与畸变系数
                para = self.get_parameter(name=self.para_name).get_parameter_value().string_value
                # print("para is: {}".format(para))
                para_dict = {}
                para_dict = ast.literal_eval(para)
                cam_tmp1 = np.array(para_dict['camera_matrix'])          
                # cam_tmp1 = np.array(camera_msg.camera_matrix)
                cam_tmp2 = cam_tmp1.reshape((3,3))
                intrin_matrix = np.append(cam_tmp2, np.array([[0.0], [0.0], [0.0]]), axis=1)   
                distor_coeff_tmp = np.array(para_dict['distor_coeff'])  
                distor_coeff = distor_coeff_tmp.reshape((5,1))
                fx = intrin_matrix[0][0]
                fy = intrin_matrix[1][1]
                cx = intrin_matrix[0][2]
                cy = intrin_matrix[1][2]
                k1 = distor_coeff[0][0]
                k2 = distor_coeff[1][0]
                p1 = distor_coeff[2][0]
                p2 = distor_coeff[3][0]
                k3 = distor_coeff[4][0]
                
                person_ = self.get_parameter(name=self.object_name).get_parameter_value().string_value
                person_dict = {}
                person_dict = ast.literal_eval(person_)
                person_position_ = np.array(person_dict['position'])
                person_position = person_position_.reshape((3,1))           

                real_world = np.append(person_position, [[1.0]], axis=0)   
                real_camera_5 = np.dot(mod_matrix,np.dot(w2c_5_matrix, real_world))
                print(real_camera_5)                
                
                if object_pixel5[0][0]<=self.pixel_u_upper and object_pixel5[0][0]>=self.pixel_u_lower:
                    # 计算理想像素坐标
                    ideal_pixel5 = np.zeros((3,1))
                    zc_5 = real_camera_5[2][0]
                    ideal_pixel5 = np.dot(intrin_matrix, real_camera_5)/zc_5
                    # 计算畸变像素坐标
                    distor_pixel5 = self.distor_pixel_generate(ideal_pixel5, fx, fy, cx, cy, k1, k2, p1, p2, k3)
                    
                    diff5 = distor_pixel5-object_pixel5
                    diff5[0][0] = diff5[0][0]/(pixel_msg.pixel5_u2-pixel_msg.pixel5_u1)
                    diff5[1][0] = diff5[1][0]/(pixel_msg.pixel5_v2-pixel_msg.pixel5_v1)
                    
                    print("distor_pixel5: {}; detect_pixel5: {}; diff5: {}".format(distor_pixel5, object_pixel5, diff5))                
                    # print("distor_pixel5: {}; detect_pixel5: {}; diff5: {}".format(distor_pixel5, object_pixel5, distor_pixel5-object_pixel5))                                              
                    
                    if distor_pixel5[0][0]<=640.0 and distor_pixel5[0][0]>=0.0:
                        u_ratio5 = (object_pixel5[0][0]-320.0)
                        
                        self.plt_x.append(u_ratio5)

                        self.plt_y.append(abs(diff5[0][0]))
                        
                        print("Min: {}".format(min(self.plt_y)))  
                        print("Max: {}".format(max(self.plt_y)))
                        print("Ave: {}".format(np.mean(self.plt_y)))
                    
                    if self.count>500: 
                        plt.xlabel('PixelDist')
        
                        plt.ylabel('RelErr')
                        colors1 = '#00CED1' #点的颜色
                        plt.xlim(xmax=350, xmin=-350)
                        plt.ylim(ymax=0.5, ymin=-0.5)        
                        area = np.pi * (2**2)  # 点面积                 
                        plt.scatter(self.plt_x, self.plt_y, s=area, c=colors1, alpha=0.4, label='RelErr/PixelDist')
                        plt.legend()
                        plt.show()                                 
              
                         
            
            
    
    # 消去尺度因子后，得到reduct_coeff*[x;y;z]=right_vec
    
    def reduct_coeff_generate(self, coeff_matrix, object_pixel):
        reduct_matrix = np.zeros((2,3))
        for i in range(2):
            for j in range(3):
                reduct_matrix[i][j] = object_pixel[i][0]*coeff_matrix[2][j]-coeff_matrix[i][j]
        return reduct_matrix
    
    def right_vec_generate(self, coeff_matrix, object_pixel):
        right_vec = np.zeros((2,1))
        right_vec[0][0] = coeff_matrix[0][3]-object_pixel[0][0]*coeff_matrix[2][3]
        right_vec[1][0] = coeff_matrix[1][3]-object_pixel[1][0]*coeff_matrix[2][3]
        return right_vec
    
    # “计算内参矩阵”对应公式的系数矩阵
    def intrin_matrix_compute_coeff(self, object_world):
        coeff = np.zeros((2,4))
        coeff[0][0] = object_world[0][0]/object_world[2][0]
        coeff[0][1] = 1.0
        coeff[1][2] = object_world[1][0]/object_world[2][0]
        coeff[1][3] = 1.0
        return coeff
    
    # 根据理想像素坐标计算畸变像素坐标
    def distor_pixel_generate(self, ideal_pixel, fx, fy, cx, cy, k1, k2, p1, p2, k3):
        distor_pixel = np.zeros((2,1))
        x = (ideal_pixel[0][0]-cx)/fx;
        y = (ideal_pixel[1][0]-cy)/fy;
        r = np.sqrt(x*x+y*y)
        x_d = x*(1+k1*r*r + k2*r*r*r*r + k3*r*r*r*r*r*r)+ 2*p1*x*y + p2*(r*r + 2*x*x)
        y_d = y*(1+k1*r*r + k2*r*r*r*r + k3*r*r*r*r*r*r)+ 2*p2*x*y + p1*(r*r + 2*y*y)
        distor_pixel[0][0] = fx*x_d+cx;
        distor_pixel[1][0] = fy*y_d+cy;
        return distor_pixel
            
def main():
    rclpy.init()
    node = ObjectWorldPubNewDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()       
    
# 问题：lookup_transform时间差问题 ///要不干脆用最新的算了
# 发现：lookup_transform找的时间戳是严格的，如果队列中找不到，就会报错，并提示队列中这个时间戳之前的据此最近的一个时间戳


# 待检验:
# 1. object_pixel_...的坐标变换是否不够准确?(修改近似时间辍同步容忍、将车子运动速度调得更慢些)
# 2. 相机坐标系、像素坐标系之间的关系是否同公式一样?
# 3. 放一些更远或更近的物体，判断误差与物体到车子间距离的关系(还是要将车子运动模式改成原地打转))

# 1. 车子速度调慢一点
# 2. 人与摄像头的距离远近与误差之间的关系
# 3. 误差指标？