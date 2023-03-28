#!/usr/bin/python3
# -*- coding: utf-8 -*
class pelco_cam_cmd_structs():

	_frame = {
		   'synch_byte':'ff',		# Synch Byte, always FF		-	1 byte
		   'address':	'00',		# Address			-	1 byte
		   'command1':	'00',		# Command1			-	1 byte
		   'command2':	'00', 	# Command2			-	1 byte
		   'data1':	'00', 	# Data1	(PAN SPEED):		-	1 byte
		   'data2':	'00', 	# Data2	(TILT SPEED):		- 	1 byte 
		   'checksum':	'00'		# Checksum:			-       1 byte
		 }    
	_function_code = {
        'Stop': '00',
        'SetSpeed': '25',
        'ZoomWide': '40',
        'ZoomTele': '20',
        'SetPosition': '4f'
		}

	'''
	    _function_code = {
        'Stop': 'ff010000000001',
        'SetSpeed0': 'ff010025000026',
		'SetSpeed1': 'ff010025000127',
		'SetSpeed2': 'ff010025000228',
		'SetSpeed3': 'ff010025000329',
        'ZoomWide': 'ff010040000041',
        'ZoomTele': 'ff010020000021',
		'SetPosition00': 'ff01004f000050'
        'SetPosition01': 'ff01004f199a03'
		'SetPosition02': 'ff01004f3333b6'
		'SetPosition03': 'ff01004f4ccd69'
		'SetPosition04': 'ff01004f66661c'
		'SetPosition05': 'ff01004f8000d0'
		'SetPosition06': 'ff01004f999982'
		'SetPosition07': 'ff01004fb33235'
		'SetPosition08': 'ff01004fcccce8'
		'SetPosition09': 'ff01004fe6669c'
		'SetPosition10': 'ff01004fffff4e'
		'ResetCamera': 'ff01002900002a'
    }'''
