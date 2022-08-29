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
        # try:

        val = input("Turn off DCA? (y/n)\n\n")
            
        if val == 'Y' or val == 'y':
            mmwave_sensor.dca_socket.sendto(mmwave_sensor.dca_cmd['RECORD_STOP_CMD_CODE'], mmwave_sensor.dca_cmd_addr)
            mmwave_sensor.collect_response()
            mmwave_sensor.close()
            sys.exit(0)


                # mmwave_sensor.collect_data()
        rospy.spin()
        

        # except rospy.ROSInitException():
        #     pass
            # print('test')
            # mmwave_sensor.dca_socket.sendto(mmwave_sensor.dca_cmd['RECORD_STOP_CMD_CODE'], mmwave_sensor.dca_cmd_addr)
            # mmwave_sensor.collect_response()
            # mmwave_sensor.close()
            # sys.exit(0)
        
        