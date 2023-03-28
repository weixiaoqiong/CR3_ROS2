#!/usr/bin/python3
# -*- coding: utf-8 -*-
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
from scout_interfaces_msg.msg import ObjectPixelTfMixed
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
        
        self.object_subscriber = self.create_subscription(ObjectPixelTfMixed, 'object_pixel_tf_mixed_info', self.object_world_compute, 50)
        
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
        
        self.para_name = 'gazebo_left'
        # print(para_name)
        self.declare_parameter(name=self.para_name)    
        self.object_name = "person_standing5"   
        self.declare_parameter(name=self.object_name)   
        
        self.pixel_width = 640 
        self.pixel_height = 480
        
        self.plt_x = []
        self.plt_y = []
        self.count = 0
    
    
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
            # 处理内参矩阵
            para = self.get_parameter(name=self.para_name).get_parameter_value().string_value
            # print("para is: {}".format(para))
            para_dict = {}
            para_dict = ast.literal_eval(para)
            cam_tmp1 = np.array(para_dict['camera_matrix'])          
            # cam_tmp1 = np.array(camera_msg.camera_matrix)
            cam_tmp2 = cam_tmp1.reshape((3,3))
            intrin_matrix = np.append(cam_tmp2, np.array([[0.0], [0.0], [0.0]]), axis=1)
            # print("intrin_matrix is: {}".format(intrin_matrix))
            
            
            # 计算world_to_camera 齐次变换矩阵  
            # tf_extend.world_to_camera_pub中获取的四元数参考坐标系是"camera"，因此转化的矩阵就是从world到camera的
            w2c_1 = pixel_msg.pixel1_w2c 
            # print("w2c_1: {}".format(w2c_1))
            w2c_2 = pixel_msg.pixel2_w2c
            # print("w2c_2: {}".format(w2c_1))

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
            w2c_1_matrix = tf_transformations.quaternion_matrix(w2c_1_ls) # 已经是numpy数组了
            w2c_2_matrix = tf_transformations.quaternion_matrix(w2c_2_ls)
            # tf_transformations.translation_matrix() 
            # 可以直接将平移向量转化为齐次矩阵，平移齐次矩阵乘以旋转齐次矩阵就得到了最终的齐次变换矩阵
            # 因为这种方法还要先建列表，也并不多简单，因此直接采取下面的方法修改
            w2c_1_matrix[0][3] = w2c_1.translation.x
            w2c_1_matrix[1][3] = w2c_1.translation.y
            w2c_1_matrix[2][3] = w2c_1.translation.z
            w2c_2_matrix[0][3] = w2c_2.translation.x
            w2c_2_matrix[1][3] = w2c_2.translation.y
            w2c_2_matrix[2][3] = w2c_2.translation.z
            
            # print("w2c_1_matrix is: {}".format(w2c_1_matrix))
            # print("w2c_2_matrix is: {}".format(w2c_2_matrix))
            
            # mod_matrix = np.array([[0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]]) # 摄像头坐标系z向外，y向下        
            mod_matrix = np.array([[0.0, -1.0, 0.0, 0.0], [0.0, 0.0, -1.0, 0.0], [1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]]) # 摄像头坐标系z向外，y向下   
            coeff_matrix1 = np.dot(intrin_matrix, np.dot(mod_matrix, w2c_1_matrix))
            coeff_matrix2 = np.dot(intrin_matrix, np.dot(mod_matrix, w2c_2_matrix))
            # coeff_matrix = np.append(coeff_matrix1, coeff_matrix2, axis=0)
            # 两个尺度因子不同，不能这样求解
            # print("coeff_matrix is: {}".format(coeff_matrix))
            
            # 处理像素信息
            p = ObjectPixel()
            object_pixel1 = np.zeros((2,1))
            object_pixel1[0] = pixel_msg.pixel1_u2-0.12*(pixel_msg.pixel1_u2-pixel_msg.pixel1_u1) #(pixel_msg.pixel1_u1+pixel_msg.pixel1_u2)/2.0
            object_pixel1[1] = pixel_msg.pixel1_v2 # (pixel_msg.pixel1_v1+pixel_msg.pixel1_v2)/2.0
            # object_pixel1[2] = 1.0
            object_pixel2 = np.zeros((2,1))
            object_pixel2[0] = pixel_msg.pixel2_u2-0.12*(pixel_msg.pixel2_u2-pixel_msg.pixel2_u1) # (pixel_msg.pixel2_u1+pixel_msg.pixel2_u2)/2.0
            object_pixel2[1] = pixel_msg.pixel2_v2 # (pixel_msg.pixel2_v1+pixel_msg.pixel2_v2)/2.0
            # object_pixel2[2] = 1.0
            # object_pixel = np.append(object_pixel1, object_pixel2, axis=0)
            # print("object_pixel is: {}".format(object_pixel))
            
            pixel1_ratio1 = (pixel_msg.pixel1_u2-pixel_msg.pixel1_u1)/self.pixel_width
            pixel1_ratio2 = (pixel_msg.pixel1_v2-pixel_msg.pixel1_v1)/self.pixel_height
            pixel2_ratio1 = (pixel_msg.pixel2_u2-pixel_msg.pixel2_u1)/self.pixel_width
            pixel2_ratio2 = (pixel_msg.pixel2_v2-pixel_msg.pixel2_v1)/self.pixel_height            
            
            # 计算人到摄像头的两个位置的距离
            world_in_camera1 = np.zeros((3,1))
            world_in_camera1[0][0] = w2c_1.translation.x
            world_in_camera1[1][0] = w2c_1.translation.y
            world_in_camera1[2][0] = w2c_1.translation.z
            c2w_roat_matrix1 = np.linalg.inv(w2c_1_matrix[0:3,0:3])
            camera_in_world1 = -np.dot(c2w_roat_matrix1, world_in_camera1)
            # print(camera_in_world1)            
            world_in_camera2 = np.zeros((3,1))
            world_in_camera2[0][0] = w2c_2.translation.x
            world_in_camera2[1][0] = w2c_2.translation.y
            world_in_camera2[2][0] = w2c_2.translation.z
            c2w_roat_matrix2 = np.linalg.inv(w2c_2_matrix[0:3,0:3])
            camera_in_world2 = -np.dot(c2w_roat_matrix2, world_in_camera2)
            
            
            '''
            # 计算世界坐标并发布 v1
            # 此版本将z轴坐标直接设置为0
            self.to_pub = ObjectWorld()
            self.to_pub.header = pixel_msg.header
            self.to_pub.object_label = pixel_msg.pixel2_label
            
            reduct_coeff_matrix1 = self.reduct_coeff_generate(coeff_matrix1, object_pixel1)
            reduct_coeff_matrix2 = self.reduct_coeff_generate(coeff_matrix2, object_pixel2)
            reduct_coeff_matrix = np.append(reduct_coeff_matrix1, reduct_coeff_matrix2, axis=0)
            
            right_vec1 = self.right_vec_generate(coeff_matrix1, object_pixel1)
            right_vec2 = self.right_vec_generate(coeff_matrix2, object_pixel2)
            right_vec = np.append(right_vec1, right_vec2, axis=0)
            
            # print(reduct_coeff_matrix1[0:2,0:2])
            object_world1 = np.dot(np.linalg.inv(reduct_coeff_matrix1[0:2, 0:2]), right_vec1)
            # print(object_world)
            self.to_pub.object_world_x = float(object_world1[0])
            self.to_pub.object_world_y = float(object_world1[1])
            self.to_pub.object_world_z = 0.0
            # print("to_sub (1) is computed as: {}".format(self.to_pub))
            # print("object_world (1): {}".format(object_world1))
            
            person_ = self.get_parameter(name=self.object_name).get_parameter_value().string_value
            person_dict = {}
            person_dict = ast.literal_eval(person_)
            person_position_ = np.array(person_dict['position'])
            person_position = person_position_.reshape((3,1))
            # print(np.shape(object_world))
            # print(np.shape(person_position))
            
            comput_1 = np.append(object_world1, [[0.0]], axis=0)
            err_vec1 = comput_1-person_position
            abs_err1 = np.linalg.norm(err_vec1)

            rel_err1 = abs_err1/(np.linalg.norm(camera_in_world1-person_position))
            
            # print("Absolute error (1): {}".format(abs_err1))
            # print("Relative error (1): {}".format(rel_err1))
            
            
            
            object_world2 = np.dot(np.linalg.inv(reduct_coeff_matrix2[0:2, 0:2]), right_vec2)
            # print("object_world (2): {}".format(object_world2))
            
            comput_2 = np.append(object_world2, [[0.0]], axis=0)
            err_vec2 = comput_2-person_position
            abs_err2 = np.linalg.norm(err_vec2)

            rel_err2 = abs_err2/(np.linalg.norm(camera_in_world2-person_position))
            
            # print("Absolute error (2): {}".format(abs_err2))
            # print("Relative error (2): {}".format(rel_err2)) 
            
            trans_dist = np.linalg.norm(coeff_matrix1-coeff_matrix2,'fro')
            self.count += 1

            # 根据实际世界坐标预估像素坐标，检验公式中相关系数的正确性
            # 发现仿真摄像头的内参矩阵认定的摄像头坐标系与仿真中摄像头的坐标系不一致
            
            pixel_predict1 = np.zeros((3,1))    
            pixel_predict2 = np.zeros((3,1))  
            real_world = np.append(person_position, [[1.0]], axis=0)   
            real_camera_1 = np.dot(mod_matrix,np.dot(w2c_1_matrix, real_world))
            real_camera_2 = np.dot(mod_matrix,np.dot(w2c_2_matrix, real_world))
            zc_1 = real_camera_1[2]
            zc_2 = real_camera_2[2]
            pixel_predict1 = np.dot(intrin_matrix, real_camera_1)/zc_1
            pixel_predict2 = np.dot(intrin_matrix, real_camera_2)/zc_2
            # print("zc_1 is: {}".format(zc_1))
            # print("zc_2 is {}".format(zc_2))
            # print("The predicted pixel (1) is computed as: {}, and the detected value is {}".format(pixel_predict1, object_pixel1))
            # print("The predicted pixel (2) is computed as: {}, and the detected value is {}".format(pixel_predict2, object_pixel2))
            # self.plt_y.append(np.abs(pixel_predict1[0][0]-object_pixel1[0][0]))  
            # self.plt_y.append(np.abs(pixel_predict1[1][0]-object_pixel1[1][0]))  
            # self.plt_y.append((pixel_predict1[0][0]-object_pixel1[0][0])/(pixel_msg.pixel1_u2-pixel_msg.pixel1_u1))
            # print((pixel_predict1[0][0]-object_pixel1[0][0])/(pixel_msg.pixel1_u2-pixel_msg.pixel1_u1))
            # print((pixel_predict1[0][0]-object_pixel1[0][0]))
            
            if self.count>2 and pixel1_ratio1>0.05:
                self.plt_x.append(pixel_ratio1)
                # self.plt_x.append(self.count)
                # self.plt_x.append(np.linalg.norm(camera_in_world1-person_position))
                # self.plt_x.append(trans_dist)
                
                self.plt_y.append(rel_err1)            
                
                print("Count is: {}".format(self.count))
                
                print("Maximal AbsErr is: {}".format(max(self.plt_y)))
                print("Minimal AbsErr is: {}".format(min(self.plt_y)))
                print("Average AbsErr is: {}".format(np.mean(self.plt_y)))            
            
                            
                if self.count>=30:
                    
                    # plt.xlabel('C2PDist')
                    # plt.xlabel('TransDist')
                    # plt.xlabel('C2PDist')
                    plt.xlabel('PixelRatio')
                    
                    
                    plt.ylabel('RelErr')
                    colors1 = '#00CED1' #点的颜色
                    plt.xlim(xmax=0.3, xmin=0)
                    plt.ylim(ymax=0.5, ymin=0)        
                    area = np.pi * (2**2)  # 点面积                 
                    # plt.scatter(self.plt_x, self.plt_y, s=area, c=colors1, alpha=0.4, label='AbsErr/TransDist')
                    plt.scatter(self.plt_x, self.plt_y, s=area, c=colors1, alpha=0.4, label='RelErr/PixelRatio')
                    # plt.scatter(self.plt_x, self.plt_y, s=area, c=colors1, alpha=0.4, label='RelErr/Count')
                    plt.legend()
                    plt.show()    
                
            '''
            
              
                
            
            # 计算世界坐标并发布 v2
            # 此版本根据两个像素/w2c_transform计算3维世界坐标
            self.to_pub = ObjectWorld()
            self.to_pub.header = pixel_msg.header
            self.to_pub.object_label = pixel_msg.pixel2_label
            
            reduct_coeff_matrix1 = self.reduct_coeff_generate(coeff_matrix1, object_pixel1)
            reduct_coeff_matrix2 = self.reduct_coeff_generate(coeff_matrix2, object_pixel2)
            reduct_coeff_matrix = np.append(reduct_coeff_matrix1, reduct_coeff_matrix2, axis=0)
            
            right_vec1 = self.right_vec_generate(coeff_matrix1, object_pixel1)
            right_vec2 = self.right_vec_generate(coeff_matrix2, object_pixel2)
            right_vec = np.append(right_vec1, right_vec2, axis=0)    
            
            trans_dist = np.linalg.norm(coeff_matrix1-coeff_matrix2,'fro')
            if trans_dist>0.5: 
                self.count += 1
                if self.count>1 and pixel1_ratio1>0.05 and pixel1_ratio2>0.05 and pixel1_ratio1<0.5 and pixel1_ratio2<0.5: 
                    if pixel2_ratio1>0.05 and pixel2_ratio2>0.05 and pixel2_ratio1<0.5 and pixel2_ratio2<0.5:  
                        object_world = np.dot(np.linalg.pinv(reduct_coeff_matrix), right_vec)
                        # print(object_world)
                        self.to_pub.object_world_x = float(object_world[0])
                        self.to_pub.object_world_y = float(object_world[1])
                        self.to_pub.object_world_z = float(object_world[2])
                        # print("to_sub is computed as: {}".format(self.to_pub))
                        
                        person_ = self.get_parameter(name=self.object_name).get_parameter_value().string_value
                        person_dict = {}
                        person_dict = ast.literal_eval(person_)
                        person_position_ = np.array(person_dict['position']) # 行向量
                        person_position = person_position_.reshape((3,1))
                        # print(np.shape(object_world))
                        # print(np.shape(person_position))
                        
                        err_vec = object_world-person_position
                        abs_err = np.linalg.norm(err_vec)

                        rel_err1 = abs_err/(np.linalg.norm(camera_in_world1-person_position))
                        rel_err2 = abs_err/(np.linalg.norm(camera_in_world2-person_position))            
                        
                        print("Absolute error: {}".format(abs_err))
                        print("Relative error (1): {}".format(rel_err1)) 
                        print("Relative error (2): {}".format(rel_err2)) 
                                    
                        
                        # print("Transforms: {}".format(trans_dist))
                        

                        self.object_world_pub.publish(self.to_pub)   
                        
                        self.plt_x.append(self.count)
                        # self.plt_x.append(trans_dist)
                        # self.plt_x.append(self.count)
                        self.plt_y.append(rel_err1)
                        
                        print("Maximal RelErr is: {}".format(max(self.plt_y)))
                        print("Minimal RelErr is: {}".format(min(self.plt_y)))
                        print("Average RelErr is: {}".format(np.mean(self.plt_y)))  
                                        
                        if self.count>=500:
                            
                            print("Maximal RelErr is: {}".format(max(self.plt_y)))
                            print("Minimal RelErr is: {}".format(min(self.plt_y)))
                            print("Average RelErr is: {}".format(np.mean(self.plt_y)))
                            
                            # plt.xlabel('C2PDist')
                            plt.xlabel('TransDist')
                            # plt.xlabel('Count')
                            
                            
                            plt.ylabel('RelErr')
                            colors1 = '#00CED1' #点的颜色
                            plt.xlim(xmax=max(self.plt_x), xmin=0)
                            plt.ylim(ymax=max(self.plt_y), ymin=min(self.plt_y))        
                            area = np.pi * (2**2)  # 点面积                 
                            # plt.scatter(self.plt_x, self.plt_y, s=area, c=colors1, alpha=0.4, label='AbsErr/TransDist')
                            # plt.scatter(self.plt_x, self.plt_y, s=area, c=colors1, alpha=0.4, label='AbsErr/C2PDist')
                            plt.scatter(self.plt_x, self.plt_y, s=area, c=colors1, alpha=0.4, label='RelErr/Count')
                            plt.legend()
                            plt.show()    
                

                
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
    
# 问题：lookup_transform时间差问题 ///要不干脆用最新的算了
# 发现：lookup_transform找的时间戳是严格的，如果队列中找不到，就会报错，并提示队列中这个时间戳之前的据此最近的一个时间戳


# 待检验:
# 1. object_pixel_...的坐标变换是否不够准确?(修改近似时间辍同步容忍、将车子运动速度调得更慢些)
# 2. 相机坐标系、像素坐标系之间的关系是否同公式一样?
# 3. 放一些更远或更近的物体，判断误差与物体到车子间距离的关系(还是要将车子运动模式改成原地打转))

# 1. 车子速度调慢一点
# 2. 人与摄像头的距离远近与误差之间的关系
# 3. 误差指标？