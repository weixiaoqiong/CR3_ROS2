#!/usr/bin/python3
# -*- coding: utf-8 -*

from __future__ import division
from unicodedata import decimal
import serial
from serial import rs485
from cr3_scout_with_srvs.pelco_cam_cmd_structs import *
import time
import math

class cam_cmd_functions():

    def __init__(self, cmd_sem='Stop', cmd_data='00',cmd_port='/dev/ttyUSB0',cmd_baudrate=9600):
        
        self.cmd_sem = cmd_sem
        self.cmd_data = cmd_data
        self.cmd_port = cmd_port
        self.cmd_baudrate = cmd_baudrate

    def cam_cmd_write(self):

        cam_cmd_struct = pelco_cam_cmd_structs()
        cam_cmd_struct._frame['synch_byte'] = 'ff'
        cam_cmd_struct._frame['address'] = '01'
        cam_cmd_struct._frame['command1'] = '00'
        cam_cmd_struct._frame['command2'] = '00'
        cam_cmd_struct._frame['data1'] = '00'
        cam_cmd_struct._frame['data2'] = '00'
        cam_cmd_struct._frame['checksum'] = '01'

        ser1 = serial.Serial(port=self.cmd_port, baudrate=self.cmd_baudrate) #选择串口，并设置波特率
        ser = rs485.RS485(ser1.port)

        if self.cmd_sem not in cam_cmd_struct._function_code:
            print(str(self.cmd_sem),"not in cam_cmd_struct._function_code\n")
        else:
            cam_cmd_struct._frame['command2'] = cam_cmd_struct._function_code[self.cmd_sem]
            if self.cmd_sem == 'SetSpeed':
                cam_cmd_struct._frame['data2'] = self.cmd_data
            elif self.cmd_sem == 'SetPosition':
                if '.' in self.cmd_data:
                    data = eval(self.cmd_data)/10*65535
                else:
                    data = int(self.cmd_data)/10*65535
                data = int(math.ceil(data))
                data = hex(data)
                #print(data)
                while len(data)<6:
                    data = data[0:2]+'0'+data[2:]
                #print(data)
                data1 = data[2:4]
                data2 = data[-2:]
                #print(data2)
                cam_cmd_struct._frame['data1'] = data1
                cam_cmd_struct._frame['data2'] = data2
            checksum = 0
            for item in cam_cmd_struct._frame:
                if (item != 'synch_byte') and (item != 'checksum'):
                    #print(item)
                    #print(cam_cmd_struct._frame[item])
                    add = int(cam_cmd_struct._frame[item],16)
                    checksum = checksum+add
            checksum = hex(checksum)
            if len(checksum)==3:
                checksum=checksum[0:2]+'0'+checksum[2:]
            checksum = checksum[-2:]
            cam_cmd_struct._frame['checksum'] = checksum
            send_data = cam_cmd_struct._frame['synch_byte']+\
                cam_cmd_struct._frame['address']+\
                    cam_cmd_struct._frame['command1']+\
                        cam_cmd_struct._frame['command2']+\
                            cam_cmd_struct._frame['data1']+\
                                cam_cmd_struct._frame['data2']+\
                                    cam_cmd_struct._frame['checksum']
            print(send_data)
            to_send = bytes.fromhex(send_data)
            print(to_send)
            ser.write(to_send)
            # msg = '0x'+send_data
            # to_send = hex(eval(msg))
            # ser.write(to_send)
            # send_data = send_data.encdecode('hex')
            # ser.write(send_data)
