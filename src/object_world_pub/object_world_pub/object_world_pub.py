#!/usr/bin/python3
# -*- coding: utf-8 -*-
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

class ObjectWorldPubNewDemo(Node):
    def __init__(self):
        super().__init__('object_world_pub')
        
        self.object_subscriber = message_filters.Subscriber(self, ObjectPixelTfMixed5, 'object_pixel_tf_mixed_info5')      
        self.intrin_matrix_subscriber = message_filters.Subscriber(self, CameraIntrin, 'camera_intrin_info')
        
        print("ready")
        tss = message_filters.ApproximateTimeSynchronizer([self.object_subscriber, self.intrin_matrix_subscriber], 10, 5.0)
        tss.registerCallback(self.object_world_compute)
        
        self.object_world_pub = self.create_publisher(ObjectWorld, 'object_world_info', 10)   
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.object_world_pub_callback)        
        
        self.to_pub = ObjectWorld()  
        self.to_pub.header.stamp = self.get_clock().now().to_msg()
        self.to_pub.object_label = 'test'
        self.to_pub.object_world_x = 0.0
        self.to_pub.object_world_y = 0.0
        self.to_pub.object_world_z = 0.0    
        
        self.pixel_width = 640 
        self.pixel_height = 480 
        self.count = 0    
        self.pixel_ratio_threshold1 = 0.05
        self.pixel_ratio_threshold2 = 0.05
    
    
    def object_sub_callback(self,msg):
        print(msg)
        
    def int_mat_sub_callback(self, msg):
        print(msg)
        
    def object_world_compute(self, pixel_msg, camera_msg):
        
        if ((pixel_msg.pixel1_label == '') or (pixel_msg.pixel2_label =='')) or (pixel_msg.pixel1_label != pixel_msg.pixel2_label):
            # self.to_pub.header.stamp = self.get_clock().now().to_msg()
            self.to_pub.header= pixel_msg.header
            self.to_pub.object_label = "test"
            self.to_pub.object_world_x = 0.0
            self.to_pub.object_world_y = 0.0
            self.to_pub.object_world_z = 0.0
            print("No valid pixel information!")
            self.object_world_pub.publish(self.to_pub)  
            # print("OK")
        else:
            # 处理内参矩阵
            print("hello1")
            
            # cam_tmp1 = np.array(camera_msg.camera_matrix)
            # cam_tmp2 = cam_tmp1.reshape((3,3))
            # intrin_matrix = np.append(cam_tmp2, np.array([[0.0], [0.0], [0.0]]), axis=1)
            
            cam_tmp = np.array([[10547.033944, 0.000000, 247.235898], [0.000000, 14062.331143, 230.506105], [0.000000, 0.000000, 1.000000]]) # 3
            # cam_tmp = np.array([[10966.625969, 0.000000, 495.024429], [0.000000, 14615.524747, 144.568245], [0.000000, 0.000000, 1.000000]]) # 2
            # cam_tmp = np.array([[11685.516756, 0.000000, 382.552219], [0.000000, 15665.852751, 89.031884], [0.000000, 0.000000, 1.000000]]) # 1
            intrin_matrix = np.append(cam_tmp, np.array([[0.0], [0.0], [0.0]]), axis=1)
            print("intrin_matrix is: {}".format(intrin_matrix))
            
            
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
                   
            # 由于lookuptransform函数对各tf变换发布频率的高要求，w2c_transform话题中发布的是机械臂base_link到Link6的坐标变换
            # 因此这里w2c表示的实际上是从base_link到Link6的变换矩阵
            # 假定机械臂base_link坐标系与世界坐标系之间无旋转、无平移，仍需对w2c左乘（Link6到camera_link的）变换矩阵
            mod_matrix = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, -0.124], [0.0, 0.0, 0.0, 1.0]]) # 末端与摄像头没有旋转，只有平移
            # mod_matrix = np.array([[0.0, -1.0, 0.0, 0.0], [0.0, 0.0, -1.0, 0.0], [1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]]) # 摄像头坐标系z向外，y向下
            coeff_matrix1 = np.dot(intrin_matrix, np.dot(mod_matrix, w2c_1_matrix))
            coeff_matrix2 = np.dot(intrin_matrix, np.dot(mod_matrix, w2c_2_matrix))
            coeff_matrix3 = np.dot(intrin_matrix, np.dot(mod_matrix, w2c_3_matrix))
            coeff_matrix4 = np.dot(intrin_matrix, np.dot(mod_matrix, w2c_4_matrix))
            coeff_matrix5 = np.dot(intrin_matrix, np.dot(mod_matrix, w2c_5_matrix))
            # coeff_matrix = np.append(coeff_matrix1, coeff_matrix2, axis=0)
            # 两个尺度因子不同，不能这样求解
            # print("coeff_matrix is: {}".format(coeff_matrix))
            
            # 处理像素信息
            p = ObjectPixel()
            object_pixel1 = np.zeros((2,1))
            object_pixel1[0] = (pixel_msg.pixel1_u1+pixel_msg.pixel1_u2)/2.0 # pixel_msg.pixel1_u2-0.12*(pixel_msg.pixel1_u2-pixel_msg.pixel1_u1)
            object_pixel1[1] = pixel_msg.pixel1_v2
            object_pixel2 = np.zeros((2,1))
            object_pixel2[0] = (pixel_msg.pixel2_u1+pixel_msg.pixel2_u2)/2.0 # pixel_msg.pixel2_u2-0.12*(pixel_msg.pixel2_u2-pixel_msg.pixel2_u1)
            object_pixel2[1] = pixel_msg.pixel2_v2 
            object_pixel3 = np.zeros((2,1))
            object_pixel3[0] = (pixel_msg.pixel3_u1+pixel_msg.pixel3_u2)/2.0 # pixel_msg.pixel3_u2-0.12*(pixel_msg.pixel3_u2-pixel_msg.pixel3_u1)
            object_pixel3[1] = pixel_msg.pixel3_v2 
            object_pixel4 = np.zeros((2,1))
            object_pixel4[0] = (pixel_msg.pixel4_u1+pixel_msg.pixel4_u2)/2.0 # pixel_msg.pixel4_u2-0.12*(pixel_msg.pixel4_u2-pixel_msg.pixel4_u1)
            object_pixel4[1] = pixel_msg.pixel4_v2             
            object_pixel5 = np.zeros((2,1))
            object_pixel5[0] = (pixel_msg.pixel5_u1+pixel_msg.pixel5_u2)/2.0 #pixel_msg.pixel5_u2-0.12*(pixel_msg.pixel5_u2-pixel_msg.pixel5_u1)
            object_pixel5[1] = pixel_msg.pixel5_v2                         
            
            pixel_ratio_ls = []
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
            
            # 计算人到摄像头的三个位置的距离
            world_in_camera1 = np.zeros((3,1))
            world_in_camera1[0][0] = w2c_1.translation.x
            world_in_camera1[1][0] = w2c_1.translation.y
            world_in_camera1[2][0] = w2c_1.translation.z
            c2w_roat_matrix1 = np.linalg.inv(w2c_1_matrix[0:3,0:3])
            camera_in_world1 = -np.dot(c2w_roat_matrix1, world_in_camera1)
                  
            world_in_camera2 = np.zeros((3,1))
            world_in_camera2[0][0] = w2c_2.translation.x
            world_in_camera2[1][0] = w2c_2.translation.y
            world_in_camera2[2][0] = w2c_2.translation.z
            c2w_roat_matrix2 = np.linalg.inv(w2c_2_matrix[0:3,0:3])
            camera_in_world2 = -np.dot(c2w_roat_matrix2, world_in_camera2) 
              
            world_in_camera3 = np.zeros((3,1))
            world_in_camera3[0][0] = w2c_3.translation.x
            world_in_camera3[1][0] = w2c_3.translation.y
            world_in_camera3[2][0] = w2c_3.translation.z
            c2w_roat_matrix3 = np.linalg.inv(w2c_3_matrix[0:3,0:3])
            camera_in_world3 = -np.dot(c2w_roat_matrix3, world_in_camera3)              
            
            world_in_camera4 = np.zeros((3,1))
            world_in_camera4[0][0] = w2c_4.translation.x
            world_in_camera4[1][0] = w2c_4.translation.y
            world_in_camera4[2][0] = w2c_4.translation.z
            c2w_roat_matrix4 = np.linalg.inv(w2c_4_matrix[0:3,0:3])
            camera_in_world4 = -np.dot(c2w_roat_matrix4, world_in_camera4)                          
            
            world_in_camera5 = np.zeros((3,1))
            world_in_camera5[0][0] = w2c_5.translation.x
            world_in_camera5[1][0] = w2c_5.translation.y
            world_in_camera5[2][0] = w2c_5.translation.z
            c2w_roat_matrix5 = np.linalg.inv(w2c_5_matrix[0:3,0:3])
            camera_in_world5 = -np.dot(c2w_roat_matrix5, world_in_camera5)                                      
            
            # 计算世界坐标并发布 v2
            # 此版本根据两个像素/w2c_transform计算3维世界坐标
            self.to_pub = ObjectWorld()
            self.to_pub.header = pixel_msg.header
            self.to_pub.object_label = pixel_msg.pixel1_label
            
            reduct_coeff_matrix1 = self.reduct_coeff_generate(coeff_matrix1, object_pixel1)
            reduct_coeff_matrix2 = self.reduct_coeff_generate(coeff_matrix2, object_pixel2)
            reduct_coeff_matrix3 = self.reduct_coeff_generate(coeff_matrix3, object_pixel3)
            reduct_coeff_matrix4 = self.reduct_coeff_generate(coeff_matrix4, object_pixel4)
            reduct_coeff_matrix5 = self.reduct_coeff_generate(coeff_matrix5, object_pixel5)
            reduct_coeff_matrix_tmp1 = np.append(reduct_coeff_matrix1, reduct_coeff_matrix2, axis=0)
            reduct_coeff_matrix_tmp2 = np.append(reduct_coeff_matrix_tmp1, reduct_coeff_matrix3, axis=0)
            reduct_coeff_matrix_tmp3 = np.append(reduct_coeff_matrix_tmp2, reduct_coeff_matrix4, axis=0)
            reduct_coeff_matrix = np.append(reduct_coeff_matrix_tmp3, reduct_coeff_matrix5, axis=0)
            
            right_vec1 = self.right_vec_generate(coeff_matrix1, object_pixel1)
            right_vec2 = self.right_vec_generate(coeff_matrix2, object_pixel2)
            right_vec3 = self.right_vec_generate(coeff_matrix3, object_pixel3)
            right_vec4 = self.right_vec_generate(coeff_matrix4, object_pixel4)
            right_vec5 = self.right_vec_generate(coeff_matrix5, object_pixel5)
            right_vec_tmp1 = np.append(right_vec1, right_vec2, axis=0)    
            right_vec_tmp2 = np.append(right_vec_tmp1, right_vec3, axis=0)
            right_vec_tmp3 = np.append(right_vec_tmp2, right_vec4, axis=0)
            right_vec = np.append(right_vec_tmp3, right_vec5, axis=0)
            
            trans_dist1 = np.linalg.norm(coeff_matrix1-coeff_matrix2,'fro')
            trans_dist2 = np.linalg.norm(coeff_matrix2-coeff_matrix3,'fro')
            trans_dist3 = np.linalg.norm(coeff_matrix3-coeff_matrix4,'fro')
            trans_dist4 = np.linalg.norm(coeff_matrix4-coeff_matrix5,'fro')
            
            print("To determine")
            if trans_dist1>0.5 and trans_dist2>0.5 and trans_dist3>0.5 and trans_dist4>0.5: 
                self.count += 1
                print(self.count)
                if self.count>-1 and pixel1_ratio1>self.pixel_ratio_threshold1 and pixel1_ratio2>self.pixel_ratio_threshold2 and pixel1_ratio1<0.5 and pixel1_ratio2<0.5: 
                    if pixel2_ratio1>self.pixel_ratio_threshold1 and pixel2_ratio2>self.pixel_ratio_threshold2 and pixel2_ratio1<0.5 and pixel2_ratio2<0.5:  
                        if pixel3_ratio1>self.pixel_ratio_threshold1 and pixel3_ratio2>self.pixel_ratio_threshold2 and pixel3_ratio1<0.5 and pixel3_ratio2<0.5: 
                            if pixel4_ratio1>self.pixel_ratio_threshold1 and pixel4_ratio2>self.pixel_ratio_threshold2 and pixel4_ratio1<0.5 and pixel4_ratio2<0.5: 
                                if pixel5_ratio1>self.pixel_ratio_threshold1 and pixel5_ratio2>self.pixel_ratio_threshold2 and pixel5_ratio1<0.5 and pixel5_ratio2<0.5: 
                                    object_world = np.dot(np.linalg.pinv(reduct_coeff_matrix), right_vec)
                                    print(object_world)
                                    self.to_pub.object_world_x = float(object_world[0])
                                    self.to_pub.object_world_y = float(object_world[1])
                                    self.to_pub.object_world_z = float(object_world[2])
                                    # print("to_sub is computed as: {}".format(self.to_pub))
                                    self.object_world_pub.publish(self.to_pub)                                                   
            else:
                print("The poses are invalid!")                         
            
             
            
            
    
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
            
def main():
    rclpy.init()
    node = ObjectWorldPubNewDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()       
    