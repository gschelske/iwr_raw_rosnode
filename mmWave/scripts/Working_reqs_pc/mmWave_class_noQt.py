#!/usr/bin/env python3
# license removed for brevity

#firmware: xwr18xx_mmw_demo.bin that can send commands from rospkg
#firmware: mmwave_Studio_cli_xwr18xx.bin that ken uses to trigger rdr with windows mmwave studio cli
import rospy
from std_msgs.msg import String
import os
import time
import sys
import socket
import serial
import pdb
import struct
import numpy as np
# import RadarRT_lib
from circular_buffer import ring_buffer
import queue #Queue
from  ctypes import *
from radar_config import dict_to_list


class mmWave_Sensor():

    # iwr_rec_cmd = ['sensorStop', 'sensorStart']
    # dca1000evm configuration commands; only the ones used are filled in
    # TODO: hardcoded comand values should be changed
    dca_cmd = { \
        'RESET_FPGA_CMD_CODE'               : b"", \
        'RESET_AR_DEV_CMD_CODE'             : b"", \
        'CONFIG_FPGA_GEN_CMD_CODE'          : b"\x5a\xa5\x03\x00\x06\x00\x01\x01\x01\x02\x03\x1e\xaa\xee", \
        'CONFIG_EEPROM_CMD_CODE'            : b"", \
        'RECORD_START_CMD_CODE'             : b"\x5a\xa5\x05\x00\x00\x00\xaa\xee", \
        'RECORD_STOP_CMD_CODE'              : b"\x5a\xa5\x06\x00\x00\x00\xaa\xee", \
        'PLAYBACK_START_CMD_CODE'           : b"", \
        'PLAYBACK_STOP_CMD_CODE'            : b"", \
        'SYSTEM_CONNECT_CMD_CODE'           : b"\x5a\xa5\x09\x00\x00\x00\xaa\xee", \
        'SYSTEM_ERROR_CMD_CODE'             : b"\x5a\xa5\x0a\x00\x01\x00\xaa\xee", \
        'CONFIG_PACKET_DATA_CMD_CODE'       : b"\x5a\xa5\x0b\x00\x06\x00\xc0\x05\xc4\x09\x00\x00\xaa\xee", \
        'CONFIG_DATA_MODE_AR_DEV_CMD_CODE'  : b"", \
        'INIT_FPGA_PLAYBACK_CMD_CODE'       : b"", \
        'READ_FPGA_VERSION_CMD_CODE'        : b"\x5a\xa5\x0e\x00\x00\x00\xaa\xee", \
    }

    dca_cmd_addr = ('192.168.33.180', 4096)#4096)
    # dca_cmd_addr = ('192.168.1.7', 4096)
    dca_socket = None
    data_socket = None
    iwr_serial = None

    dca_socket_open = False
    data_socket_open = False
    serial_open = False

    capture_started = 0

    data_file = open('/home/user/Desktop/binary','wb')
    

    def __init__(self, iwr_cmd_tty='/dev/ttyACM0', iwr_data_tty='/dev/ttyACM1'):

        self.data_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.data_socket.bind(("192.168.33.30", 4098))#4098))
        # self.data_socket.bind(("192.168.1.7", 4098))
        self.data_socket.settimeout(25e-5)
        #self.data_socket.setblocking(True)
        self.data_socket_open = True

        self.seqn = 0  # this is the last packet index
        self.bytec = 0 # this is a byte counter
        self.q = queue.Queue() #Queue
        frame_len = 393216 #393216 -  2*rospy.get_param('iwr_cfg/profiles')[0]['adcSamples']*rospy.get_param('iwr_cfg/numLanes')*rospy.get_param('iwr_cfg/numChirps')
        self.maxim_len = frame_len * 2
        self.data_array = ring_buffer(int(2*frame_len), int(frame_len))
        self.byte_array = bytearray()


        # self.iwr_cmd_tty=iwr_cmd_tty
        # self.iwr_data_tty=iwr_data_tty

    def close(self):
        self.dca_socket.close()
        self.data_socket.close()
        # self.iwr_serial.close()

    def collect_response(self):
        status = 1
        while status:
            try:
                
                msg, server = self.dca_socket.recvfrom(4096)
                
                import struct
                (status,) = struct.unpack('<H', msg[4:6])
                
                print('response')
                print(status)
                if status == 898:
                    break
            except Exception as e:
                print('stuck')
                print(e)
                continue

    def collect_arm_response(self):
        status = 1
        while status:
            try:
                msg, server = self.dca_socket.recvfrom(4096)
                (status,) = struct.unpack('<H', msg[4:6])
                print('response_arm')
                print(status)
                
                if msg == self.dca_cmd['SYSTEM_ERROR_CMD_CODE']:
                    break
            except Exception as e:
                print(e)
                continue

    def setupDCA_and_cfgIWR(self):
        self.dca_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.dca_socket.bind(("192.168.33.30", 4096))#4096))
        # self.dca_socket.bind(("192.168.1.7", 4096))
        
        self.dca_socket.settimeout(10)#10
        self.dca_socket_open = True

        # self.iwr_serial = serial.Serial(port=self.iwr_cmd_tty, baudrate=115200, bytesize=serial.EIGHTBITS,
        #                                 parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0.100)
        # self.serial_open = self.iwr_serial.is_open

        if not self.dca_socket:# or not self.iwr_serial:
            return

        # Set up DCA
        print("SET UP DCA")
        self.dca_socket.sendto(self.dca_cmd['SYSTEM_CONNECT_CMD_CODE'], self.dca_cmd_addr)#sendto
        import sys
        print(sys.version)
        self.collect_response()
        print('next')
        self.dca_socket.sendto(self.dca_cmd['CONFIG_FPGA_GEN_CMD_CODE'], self.dca_cmd_addr)
        self.collect_response()
        print('next')
        self.dca_socket.sendto(self.dca_cmd['CONFIG_PACKET_DATA_CMD_CODE'], self.dca_cmd_addr)
        self.collect_response()
        print('next')
        
        print("")

        iwr_cfg_cmd = [
        'dfeDataOutputMode 1',
        'channelCfg 15 7 0',
        'adcCfg 2 1',
        'adcbufCfg -1 0 1 1 1',
        'profileCfg 0 77 7 3 39 0 0 100 1 256 7200 0 0 30',
        'chirpCfg 0 0 0 0 0 0 0 1',
        'chirpCfg 1 1 0 0 0 0 0 4',
        'frameCfg 0 1 32 0 100 1 0',
        'lowPower 0 0',
        'calibMonCfg 1 1',
        'monCalibReportCfg 1 1 0',
        'txPowerMonCfg 1 0 0',
        'txPowerMonCfg 1 1 0',
        'txPowerMonCfg 1 2 0',
        'txBallbreakMonCfg 1 0',
        'txBallbreakMonCfg 1 1',
        'txBallbreakMonCfg 1 2',
        'rxGainPhaseMonCfg 1 0',
        'tempMonCfg 1 20',
        'synthFreqMonCfg 1 0',
        'pllConVoltMonCfg 1',
        'dualClkCompMonCfg 1',
        'rxIfStageMonCfg 1 0',
        'extAnaSigMonCfg 0',
        'pmClkSigMonCfg 1 0',
        'rxIntAnaSigMonCfg 1 0',
        'gpadcSigMonCfg 1',
        ]
        # Send and read a few CR to clear things in buffer. Happens sometimes during power on
        # for i in range(5):
        #     self.iwr_serial.write('\n'.encode())
        #     self.iwr_serial.reset_input_buffer()
        #     time.sleep(.1)

        # for cmd in iwr_cfg_cmd:
        #     for i in range(len(cmd)):
        #         self.iwr_serial.write(cmd[i].encode('utf-8'))
        #         time.sleep(0.010)  # 10 ms delay between characters
        #     self.iwr_serial.write('\r'.encode())
        #     self.iwr_serial.reset_input_buffer()
        #     time.sleep(0.010)       # 10 ms delay between characters
        #     time.sleep(0.100)       # 100 ms delay between lines
        #     response = self.iwr_serial.read(size=100)
        #     print('LVDS Stream:/>' + cmd)
        #     print(response[2:].decode())

        # print("")

    def arm_dca(self):
        if not self.dca_socket:
            return

        print("ARM DCA")
        self.dca_socket.sendto(self.dca_cmd['RECORD_START_CMD_CODE'], self.dca_cmd_addr)
        self.collect_arm_response()
        print("success!")
        print("")

    def toggle_capture(self, toggle=0, dir_path=''):
        
        if not self.dca_socket:# or not self.iwr_serial:
            return
        
        # only send command if toggle != status of capture
        if toggle == self.capture_started:
            
            return
        
        # sensor_cmd = self.iwr_rec_cmd[toggle]
        # for i in range(len(sensor_cmd)):
        #     # print(sensor_cmd[i])
        #     # print(sensor_cmd[i].encode('utf-8'))
        #     self.iwr_serial.write(sensor_cmd[i].encode('utf-8'))
        #     time.sleep(0.010)   #  10 ms delay between characters
        # self.iwr_serial.write('\r'.encode())
        # self.iwr_serial.reset_input_buffer()
        
        # time.sleep(0.010)       #  10 ms delay between characters
        # time.sleep(0.100)       # 100 ms delay between lines
        # response = self.iwr_serial.read(size=200)
        # print('LVDS Stream:/>' + sensor_cmd)
        # print(response.decode('utf-8'))

        # if sensor_cmd == 'sensorStop':
        #     self.dca_socket.sendto(self.dca_cmd['RECORD_STOP_CMD_CODE'], self.dca_cmd_addr)
        #     self.collect_response()
        # self.capture_started = toggle

    def collect_data(self):
        try:

            
            
            # a = time.time()

            msg = self.data_socket.recv(4096)
            self.byte_array.extend(msg[10:])

            # print(len(self.byte_array))
            seqn, bytec = struct.unpack('<IIxx', msg[:10])
            print(seqn)
            if(len(self.byte_array) >= self.maxim_len):
                # print(len(self.byte_array))
                a = time.time()
                self.data_file.write(self.byte_array)
                self.byte_array.clear()
                print(time.time()-a)
            # print(time.time()-a)
            # a = time.time()

            # print(sys.getsizeof(msg))

        except Exception as e:
            print(e)
            return

        
        


        

        # self.data_file.write(msg[10:])  # keep to compare rosbag with binary here

        # print(sys.getsizeof(msg[10:]))
        # self.data_array.pad_and_add_msg(self.seqn, seqn, np.frombuffer(msg[10:], dtype=np.int16))

        # self.seqn = seqn
        # self.bytec = bytec
        # print(time.time()-a)

        # 'flushCfg',
        # 'dfeDataOutputMode 1',
        # 'channelCfg 15 7 0',
        # 'adcCfg 2 1',
        # 'adcbufCfg -1 0 1 1 1',
        # 'profileCfg 0 78.315 5.5 3.88 30.6 0 0 44.78 0 128 10000 0 0 30',
        # 'frameCfg 0 2 16 8 100 1 0',
        # 'chirpCfg 0 0 0 0 0 0 0 1',
        # 'chirpCfg 1 1 0 0 0 0 0 4',
        # 'chirpCfg 2 2 0 0 0 0 0 2',
        # 'lowPower 0 0',
        # 'guiMonitor -1 1 1 0 0 0 1',
        # 'cfarCfg -1 0 2 8 4 3 0 15 1',
        # 'cfarCfg -1 1 0 4 2 3 1 15 1',
        # 'multiObjBeamForming -1 1 0.5',
        # 'clutterRemoval -1 0',
        # 'calibDcRangeSig -1 0 -5 8 256',
        # 'extendedMaxVelocity -1 0',
        # 'lvdsStreamCfg -1 0 0 0',
        # 'bpmCfg -1 0 0 0',
        # 'compRangeBiasAndRxChanPhase 0.0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0',
        # 'measureRangeBiasAndRxChanPhase 0 1.5 0.2',
        # 'CQRxSatMonitor 0 3 4 63 0',
        # 'CQSigImgMonitor 0 127 4',
        # 'analogMonitor 0 0',
        # 'aoaFovCfg -1 -90 90 -90 90',
        # 'cfarFovCfg -1 0 0 49.99',
        # 'cfarFovCfg -1 1 -1 1.00',
        # 'calibData 0 0 0',