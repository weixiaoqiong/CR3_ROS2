#!/usr/bin/env python

from __future__ import print_function
import imp
from platform import node

import sys
from time import time_ns
from venv import EnvBuilder
import rclpy
from dobot_bringup_srv.srv import *
from rclpy.node import Node

class ServiceClientsDemo(Node):

    def __init__(self):
        super().__init__('cr3_service_client')
    
    def enable_client(self):

        enable_handle = self.create_client(EnableRobot,"/dobot_bringup/srv/EnableRobot")

        enable_req = EnableRobot.Request()

        while not enable_handle.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service /dobot_bringup/srv/EnableRobot not available, waiting again...")
        enable_future = enable_handle.call_async(enable_req)
        rclpy.spin_until_future_complete(self,enable_future)

        enable_res = enable_future.result()

        return enable_res

    def disable_client(self):

        disable_handle = self.create_client(DisableRobot,"/dobot_bringup/srv/DisableRobot")

        disable_req = DisableRobot.Request()

        while not disable_handle.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service /dobot_bringup/srv/DisableRobot not available, waiting again...")
        disable_future = disable_handle.call_async(disable_req)
        rclpy.spin_until_future_complete(self,disable_future)

        disable_res = disable_future.result()

        return disable_res
        
    def clear_client(self):

        clear_handle = self.create_client(ClearError,'/dobot_bringup/srv/ClearError')

        clear_req = ClearError.Request()        

        while not clear_handle.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service /dobot_bringup/srv/ClearError not available, waiting again...")
        clear_future = clear_handle.call_async(clear_req)
        rclpy.spin_until_future_complete(self,clear_future)

        clear_res = clear_future.result()

        return clear_res

    def reset_client(self):

        reset_handle = self.create_client(ResetRobot, '/dobot_bringup/srv/ResetRobot')

        reset_req = ResetRobot.Request()        

        while not reset_handle.wait_for_service(timeout_sec=1.0):
            self.get_logger.info("service /dobot_bringup/srv/ResetRobot not available, waiting again...")    
        reset_future = reset_handle.call_async(reset_req)
        rclpy.spin_until_future_complete(self,reset_future)

        reset_res = reset_future.result()

        return reset_res

    def sync_client(self):

        sync_handle = self.create_client(Sync, '/dobot_bringup/srv/Sync')

        sync_req = Sync.Request()

        while not sync_handle.wait_for_service(timeout_sec=1.0):
            self.get_logger.info("service /dobot_bringup/srv/Sync not available, waiting again...")    
        sync_future = sync_handle.call_async(sync_req)
        rclpy.spin_until_future_complete(self,sync_future)

        sync_res = sync_future.result()

        return sync_res

    def speedj_client(self,r):

        speedj_handle = self.create_client(SpeedJ, '/dobot_bringup/srv/SpeedJ')

        speedj_req = SpeedJ.Request()
        speedj_req.r = r

        while not speedj_handle.wait_for_service(timeout_sec=1.0):
            self.get_logger.info("service /dobot_bringup/srv/SpeedJ not available, waiting again...")    
        speedj_future = speedj_handle.call_async(speedj_req)
        rclpy.spin_until_future_complete(self,speedj_future)

        speedj_res = speedj_future.result()

        return speedj_res

    def speedl_client(self,r):

        speedl_handle = self.create_client(SpeedL, '/dobot_bringup/srv/SpeedL')

        speedl_req = SpeedL.Request()
        speedl_req.r = r

        while not speedl_handle.wait_for_service(timeout_sec=1.0):
            self.get_logger.info("service /dobot_bringup/srv/SpeedL not available, waiting again...")    
        speedl_future = speedl_handle.call_async(speedl_req)
        rclpy.spin_until_future_complete(self,speedl_future)

        speedl_res = speedl_future.result()

        return speedl_res
    
    def jointmovj_client(self,j1,j2,j3,j4,j5,j6):

        jointmovj_handle = self.create_client(JointMovJ, '/dobot_bringup/srv/JointMovJ')

        jointmovj_req = JointMovJ.Request()
        jointmovj_req.j1 = j1
        jointmovj_req.j2 = j2
        jointmovj_req.j3 = j3
        jointmovj_req.j4 = j4
        jointmovj_req.j5 = j5
        jointmovj_req.j6 = j6 

        while not jointmovj_handle.wait_for_service(timeout_sec=1.0):
            self.get_logger.info("service /dobot_bringup/srv/JointMovJ not available, waiting again...")           
        jointmovj_future = jointmovj_handle.call_async(jointmovj_req)
        rclpy.spin_until_future_complete(self,jointmovj_future)

        jointmovj_res = jointmovj_future.result()

        return jointmovj_res


    def servoj_client(self,j1,j2,j3,j4,j5,j6):

        servoj_handle = self.create_client(ServoJ, '/dobot_bringup/srv/ServoJ')

        servoj_req = ServoJ.Request()
        servoj_req.j1 = j1
        servoj_req.j2 = j2
        servoj_req.j3 = j3
        servoj_req.j4 = j4
        servoj_req.j5 = j5
        servoj_req.j6 = j6 

        while not servoj_handle.wait_for_service(timeout_sec=1.0):
            self.get_logger.info("service /dobot_bringup/srv/ServoJ not available, waiting again...")           
        servoj_future = servoj_handle.call_async(servoj_req)
        rclpy.spin_until_future_complete(self,servoj_future)

        servoj_res = servoj_future.result()

        return servoj_res


    def movj_client(self,x, y, z, a, b, c):

        movj_handle = self.create_client(MovJ, '/dobot_bringup/srv/MovJ')

        movj_req = MovJ.Request()
        movj_req.x = x
        movj_req.y = y
        movj_req.z = z
        movj_req.a = a
        movj_req.b = b
        movj_req.c = c

        while not movj_handle.wait_for_service(timeout_sec=1.0):
            self.get_logger.info("service /dobot_bringup/srv/MovJ not available, waiting again...")           
        movj_future = movj_handle.call_async(movj_req)
        rclpy.spin_until_future_complete(self,movj_future)

        movj_res = movj_future.result()

        return movj_res


    def movl_client(self,x, y, z, a, b, c):

        movl_handle = self.create_client(MovL, '/dobot_bringup/srv/MovL')

        movl_req = MovL.Request()
        movl_req.x = x
        movl_req.y = y
        movl_req.z = z
        movl_req.a = a
        movl_req.b = b
        movl_req.c = c

        while not movl_handle.wait_for_service(timeout_sec=1.0):
            self.get_logger.info("service /dobot_bringup/srv/MovL not available, waiting again...")           
        movl_future = movl_handle.call_async(movl_req)
        rclpy.spin_until_future_complete(self,movl_future)

        movl_res = movl_future.result()

        return movl_res

    def arc_client(self,x1,y1,z1,rx1,ry1,rz1,x2,y2,z2,rx2,ry2,rz2):

        arc_handle = self.create_client(Arc, '/dobot_bringup/srv/Arc')

        arc_req = Arc.Request()
        arc_req.x1 = x1
        arc_req.y1 = y1
        arc_req.z1 = z1
        arc_req.rx1 = rx1
        arc_req.ry1 = ry1
        arc_req.rz1 = rz1
        arc_req.x2 = x2
        arc_req.y2 = y2
        arc_req.z2 = z2
        arc_req.rx2 = rx2
        arc_req.ry2 = ry2
        arc_req.rz2 = rz2

        while not arc_handle.wait_for_service(timeout_sec=1.0):
            self.get_logger.info("service /dobot_bringup/srv/Arc not available, waiting again...")           
        arc_future = arc_handle.call_async(arc_req)
        rclpy.spin_until_future_complete(self,arc_future)

        arc_res = arc_future.result()       

    def usage():
        return "SpeedJ speedj_rate;/nServoJ "

if __name__ == "__main__":

    print("Hi")

    current_client = ServiceClientsDemo()
    print("client says hi")

    if sys.argv[1] == "EnableRobot":
        print(current_client.enable_client())

    if sys.argv[1] == "DisableRobot":
        print(current_client.disable_client())

    if sys.argv[1] == "ClearError":
        print(current_client.clear_client())

    if sys.argv[1] == "ResetRobot":
        print(current_client.reset_client())
        
    if sys.argv[1] == "Sync":
        print(current_client.sync_client())

    if sys.argv[1] == "SpeedJ":
        r = int(sys.argv[2])
        print(current_client.speedj_client(r))

    if sys.argv[1] == "SpeedL":
        r = int(sys.argv[2])
        print(current_client.speedl_client(r))        

    if sys.argv[1] == "ServoJ":
        j1 = float(sys.argv[2])
        j2 = float(sys.argv[3])
        j3 = float(sys.argv[4])
        j4 = float(sys.argv[5])
        j5 = float(sys.argv[6])
        j6 = float(sys.argv[7])
        print(current_client.servoj_client(j1, j2, j3, j4, j5, j6))      

    if sys.argv[1] == "JointMovJ":
        j1 = float(sys.argv[2])
        j2 = float(sys.argv[3])
        j3 = float(sys.argv[4])
        j4 = float(sys.argv[5])
        j5 = float(sys.argv[6])
        j6 = float(sys.argv[7])
        print(current_client.jointmovj_client(j1, j2, j3, j4, j5, j6))         

    if sys.argv[1] == "MovJ":
        x = float(sys.argv[2])
        y = float(sys.argv[3])
        z = float(sys.argv[4])
        a = float(sys.argv[5])
        b = float(sys.argv[6])
        c = float(sys.argv[7])
        print(current_client.movj_client(x, y, z, a, b, c))    

    if sys.argv[1] == "MovL":
        x = float(sys.argv[2])
        y = float(sys.argv[3])
        z = float(sys.argv[4])
        a = float(sys.argv[5])
        b = float(sys.argv[6])
        c = float(sys.argv[7])
        print(current_client.movl_client(x, y, z, a, b, c))

    if sys.argv[1] == "Arc":
        x1 = float(sys.argv[2])
        y1 = float(sys.argv[3])
        z1 = float(sys.argv[4])
        rx1 = float(sys.argv[5])
        ry1 = float(sys.argv[6])
        rz1 = float(sys.argv[7])        
        x2 = float(sys.argv[2])
        y2 = float(sys.argv[3])
        z2 = float(sys.argv[4])
        rx2 = float(sys.argv[5])
        ry2 = float(sys.argv[6])
        rz2 = float(sys.argv[7])           
        print(current_client.arc_client(x1, y1, z1, rx1, ry1, rz1, x2, y2, z2, rx2, ry2, rz2))
   

    