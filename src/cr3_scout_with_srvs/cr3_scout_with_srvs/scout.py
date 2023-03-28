#!/usr/bin/python3
# -*- coding: utf-8 -*-
from __future__ import print_function
import cr3_scout_with_srvs.cr3_service_clients
import rclpy
import time

def main(args=None):
    rclpy.init(args=args)
    
    print("Scout: Hello\n")
    current_clients = cr3_scout_with_srvs.cr3_service_clients.ServiceClientsDemo()

    print(current_clients.enable_client())


    print(current_clients.clear_client())

    print(current_clients.speedj_client(20))
    time.sleep(2.0)
    time.sleep(2.0)

    '''
    print(current_clients.sync_client())
    print("Sync!")
    rospy.sleep(2.0)'''

    print(current_clients.jointmovj_client(0.0,0.0,0.0,0.0,90.0,0.0))
    time.sleep(5.0)

    j1_state = -175.0
    j4_state = 60.0
    up_or_down = -1
    iteration_number = 35

    print(current_clients.jointmovj_client(j1_state,0.0,0.0,j4_state, 90.0, 0.0))
    time.sleep(3.0)

    for i in range(iteration_number):
        j1_state += 10.0
        if up_or_down==-1:
            j4_state -= 120.0
        else:
            j4_state += 120.0
        print(current_clients.jointmovj_client(j1_state,0.0,0.0,j4_state, 90.0, 0.0))
        time.sleep(2.0)
        up_or_down *= -1

    print(current_clients.jointmovj_client(0.0,0.0,0.0,0.0,0.0,0.0))
    time.sleep(5.0)
    print(current_clients.jointmovj_client(172.0,0.0,-150.0,-35.0,0.0,0.0))
    time.sleep(2.0)




