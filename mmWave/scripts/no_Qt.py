#!/usr/bin/env python3
# license removed for brevity
import rospy
import rospkg
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray
from mmWave.msg import data_frame
from rospy.numpy_msg import numpy_msg
import os
import time
import sys
import socket
import serial
import pdb
from mmWave_class_noQt import mmWave_Sensor
import queue #Queue
import threading
import pickle
from radar_config import cfg_list_to_dict
import argparse
import json


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("cfg", help="select configuration to apply to the radar")
    parser.add_argument('--cmd_tty', default='/dev/ttyACM0',
                        help='''TTY device or serial port for configuration
                        commands''')
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('radar_collect', anonymous=True)

    time.sleep(2)  # wait for topics to be set up

    mmwave_sensor = mmWave_Sensor(iwr_cmd_tty=args.cmd_tty)
    mmwave_sensor.setupDCA_and_cfgIWR()

    mmwave_sensor.arm_dca()
    time.sleep(2)

    rate = rospy.Rate(8000000)

    while not rospy.is_shutdown():

        user_shutdown = rospy.get_param("user_shutdown")

        if user_shutdown == 'shutdown':
            rospy.signal_shutdown('User shutdown')

            for i in range(5):
                mmwave_sensor.iwr_serial.write('\r'.encode())
                mmwave_sensor.iwr_serial.reset_input_buffer()
                time.sleep(.1)
            cmd = 'sensorStop'
            for i in range(len(cmd)):
                mmwave_sensor.iwr_serial.write(cmd[i].encode('utf-8'))
                time.sleep(0.010)  # 10 ms delay between characters
            mmwave_sensor.iwr_serial.write('\r'.encode())
            mmwave_sensor.iwr_serial.reset_input_buffer()
            time.sleep(0.010)       # 10 ms delay between characters
            time.sleep(0.100)       # 100 ms delay between lines
            response = mmwave_sensor.iwr_serial.read(size=100)
            print('LVDS Stream:/>' + cmd)
            print(response[2:].decode())

            mmwave_sensor.dca_socket.sendto(mmwave_sensor.dca_cmd['RECORD_STOP_CMD_CODE'], mmwave_sensor.dca_cmd_addr)
            mmwave_sensor.collect_response()
            mmwave_sensor.close()
            sys.exit(0)
            
        # try:

        # val = input("Turn off DCA? (y/n)\n\n")
            
        # if val == 'Y' or val == 'y':
            # for i in range(5):
            #     mmwave_sensor.iwr_serial.write('\r'.encode())
            #     mmwave_sensor.iwr_serial.reset_input_buffer()
            #     time.sleep(.1)
            # cmd = 'sensorStop'
            # for i in range(len(cmd)):
            #     mmwave_sensor.iwr_serial.write(cmd[i].encode('utf-8'))
            #     time.sleep(0.010)  # 10 ms delay between characters
            # mmwave_sensor.iwr_serial.write('\r'.encode())
            # mmwave_sensor.iwr_serial.reset_input_buffer()
            # time.sleep(0.010)       # 10 ms delay between characters
            # time.sleep(0.100)       # 100 ms delay between lines
            # response = mmwave_sensor.iwr_serial.read(size=100)
            # print('LVDS Stream:/>' + cmd)
            # print(response[2:].decode())

            # mmwave_sensor.dca_socket.sendto(mmwave_sensor.dca_cmd['RECORD_STOP_CMD_CODE'], mmwave_sensor.dca_cmd_addr)
            # mmwave_sensor.collect_response()
            # mmwave_sensor.close()
            # sys.exit(0)


                # mmwave_sensor.collect_data()
        # rospy.spin()
        rate.sleep()
        

    # for i in range(5):
    #     mmwave_sensor.iwr_serial.write('\r'.encode())
    #     mmwave_sensor.iwr_serial.reset_input_buffer()
    #     time.sleep(.1)
    # cmd = 'sensorStop'
    # for i in range(len(cmd)):
    #     mmwave_sensor.iwr_serial.write(cmd[i].encode('utf-8'))
    #     time.sleep(0.010)  # 10 ms delay between characters
    # mmwave_sensor.iwr_serial.write('\r'.encode())
    # mmwave_sensor.iwr_serial.reset_input_buffer()
    # time.sleep(0.010)       # 10 ms delay between characters
    # time.sleep(0.100)       # 100 ms delay between lines
    # response = mmwave_sensor.iwr_serial.read(size=100)
    # print('LVDS Stream:/>' + cmd)
    # print(response[2:].decode())

    # mmwave_sensor.dca_socket.sendto(mmwave_sensor.dca_cmd['RECORD_STOP_CMD_CODE'], mmwave_sensor.dca_cmd_addr)
    # mmwave_sensor.collect_response()
    # mmwave_sensor.close()
    # sys.exit(0)
        
        
        