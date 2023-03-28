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
from std_msgs.msg import String
# import os, sys
# sys.path.append(os.getcwd())



class CamInfoFakePub(Node):

    def __init__(self):
        super().__init__('cam_info_fake_pub')
        self.publishers_ = self.create_publisher(String, 'current_zoom', 10)
        # self.publisher_ = self.create_publisher(CameraIntrin, 'camera_intrin_info', 10)
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
    
        self.zoom = '00'
        # self.intrin_mat = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

        current_clients = cr3_scout_with_srvs.cr3_service_clients.ServiceClientsDemo()

        print(current_clients.enable_client())
        print(current_clients.clear_client())
        
        
        # start! (speed\duration\zoom)
        
        
        # position1
        print(current_clients.speedj_client(3))
        duration = 7.0
        multi = 4
        print("scout_with_zoom_camera: Hello")
        cmd_sem1 = 'SetPosition'
        # cmd_data1 = '00'
        cmd_data1 = str(multi).zfill(2)
        cmd_port1 = '/dev/ttyUSB0'
        cmd_baudrate1 = 9600
        camera_command1 = pelco_cam.cam_cmd_functions(cmd_sem=cmd_sem1,cmd_data=cmd_data1,cmd_port=cmd_port1,cmd_baudrate=cmd_baudrate1)
        camera_command1.cam_cmd_write()
        time.sleep(5.0)
        camera_command2 = pelco_cam.cam_cmd_functions()
        camera_command2.cam_cmd_write()
        time.sleep(0.5)    
        current_zoom_data = str(multi).zfill(2)
        current_zoom_msg = String()
        current_zoom_msg.data = current_zoom_data
        for i in range(10):
            self.publishers_.publish(current_zoom_msg)
        print("Scout: Hello\n")
        print(current_clients.jointmovj_client(0.0,0.0,0.0,0.0,90.0,0.0))
        time.sleep(duration)
        j1_state = 30.0
        j4_state = 0.0
        up_or_down = -1
        iteration_number = 12
        print(current_clients.jointmovj_client(j1_state,0.0,0.0,j4_state, 90.0, 0.0))
        time.sleep(duration)
        for i in range(iteration_number):
            j1_state += 10.0
            if up_or_down==-1:
                j4_state -= 30.0
            else:
                j4_state += 30.0
            print(current_clients.jointmovj_client(j1_state,0.0,0.0,j4_state, 90.0, 0.0))
            time.sleep(duration)
            up_or_down *= -1        
        print(current_clients.jointmovj_client(0.0,0.0,0.0,0.0,0.0,0.0))
        time.sleep(duration)  
             

        # # position2
        # print(current_clients.speedj_client(3))
        # duration = 7.0
        # multi = 4
        # print("scout_with_zoom_camera: Hello")
        # cmd_sem1 = 'SetPosition'
        # # cmd_data1 = '00'
        # cmd_data1 = str(multi).zfill(2)
        # cmd_port1 = '/dev/ttyUSB0'
        # cmd_baudrate1 = 9600
        # camera_command1 = pelco_cam.cam_cmd_functions(cmd_sem=cmd_sem1,cmd_data=cmd_data1,cmd_port=cmd_port1,cmd_baudrate=cmd_baudrate1)
        # camera_command1.cam_cmd_write()
        # time.sleep(5.0)
        # camera_command2 = pelco_cam.cam_cmd_functions()
        # camera_command2.cam_cmd_write()
        # time.sleep(0.5)    
        # current_zoom_data = str(multi).zfill(2)
        # current_zoom_msg = String()
        # current_zoom_msg.data = current_zoom_data
        # for i in range(10):
        #     self.publishers_.publish(current_zoom_msg)
        # print("Scout: Hello\n")
        # print(current_clients.jointmovj_client(0.0,0.0,0.0,0.0,90.0,0.0))
        # time.sleep(duration)
        # j1_state = 30.0
        # j4_state = 0.0
        # up_or_down = -1
        # iteration_number = 12
        # print(current_clients.jointmovj_client(j1_state,0.0,0.0,j4_state, 90.0, 0.0))
        # time.sleep(duration)
        # for i in range(iteration_number):
        #     j1_state += 10.0
        #     if up_or_down==-1:
        #         j4_state -= 30.0
        #     else:
        #         j4_state += 30.0
        #     print(current_clients.jointmovj_client(j1_state,0.0,0.0,j4_state, 90.0, 0.0))
        #     time.sleep(duration)
        #     up_or_down *= -1        
        # print(current_clients.jointmovj_client(0.0,0.0,0.0,0.0,0.0,0.0))
        # time.sleep(duration)  

        '''
        print(current_clients.speedj_client(3))
        duration = 7.0        
        for multi in range(11):
            # # 1. zoom;
            print("scout_with_zoom_camera: Hello")
            cmd_sem1 = 'SetPosition'
            # cmd_data1 = '00'
            cmd_data1 = str(multi).zfill(2)
            cmd_port1 = '/dev/ttyUSB0'
            cmd_baudrate1 = 9600
            camera_command1 = pelco_cam.cam_cmd_functions(cmd_sem=cmd_sem1,cmd_data=cmd_data1,cmd_port=cmd_port1,cmd_baudrate=cmd_baudrate1)
            camera_command1.cam_cmd_write()
            time.sleep(5.0)

            camera_command2 = pelco_cam.cam_cmd_functions()
            camera_command2.cam_cmd_write()
            time.sleep(0.5)    

            current_zoom_data = str(multi).zfill(2)
            current_zoom_msg = String()
            current_zoom_msg.data = current_zoom_data
            for i in range(10):
                self.publishers_.publish(current_zoom_msg)
                
            # 3. scout;
            print("Scout: Hello\n")

            # time.sleep(2.0)
            # time.sleep(2.0)

            # 
            # print(current_clients.sync_client())
            # print("Sync!")
            # rospy.sleep(2.0)

            print(current_clients.jointmovj_client(0.0,0.0,0.0,0.0,90.0,0.0))
            time.sleep(duration)

            j1_state = 30.0
            j4_state = 0.0
            up_or_down = -1
            iteration_number = 12

            print(current_clients.jointmovj_client(j1_state,0.0,0.0,j4_state, 90.0, 0.0))
            time.sleep(duration)

            for i in range(iteration_number):
                j1_state += 10.0
                if up_or_down==-1:
                    j4_state -= 30.0
                else:
                    j4_state += 30.0
                print(current_clients.jointmovj_client(j1_state,0.0,0.0,j4_state, 90.0, 0.0))
                time.sleep(duration)
                up_or_down *= -1
            # time.sleep(3.0)
            # 1. zoom;
        print(current_clients.jointmovj_client(0.0,0.0,0.0,0.0,0.0,0.0))
        time.sleep(duration)            
        '''



def main(args=None):
    rclpy.init(args=args)

    node = CamInfoFakePub()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    
    # PixelFakePub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
