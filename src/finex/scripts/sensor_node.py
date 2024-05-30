#!/usr/bin/env python3

import sys
import os
import time
import can
import struct
import rclpy
import argparse
import matplotlib.pyplot as plt
import numpy as np

from rclpy.node import Node
from rclpy import qos
from std_msgs.msg import Float32
from std_msgs.msg import UInt16
from scipy import signal
from scipy.fft import rfft, rfftfreq

from collections import deque


class CANbus(object):
    def __init__(self, channel, bustype='socketcan', verbose=False):        
        self.bus = can.interface.Bus(channel=channel, bustype=bustype, bitrate=1000000)
        self.buffer = can.BufferedReader()
        self.verbose = verbose

    def __del__(self):
        self.bus.shutdown()

    """ Send the following data frame to receive a data package from the sensor acquisition board:
        Message type: Standard (11-bit identifier)
        Message identifier (ID): 68
        Pack data: 0
    """
    def send_command(self):
        id = 68
        msg = can.Message(arbitration_id=id, is_extended_id=False, data=[0, 0, 0, 0, 0, 0, 0, 0])   # 11-bit identifier (not-extended)
        if self.verbose: print("Sending initialization message (ID: %d)!" % (id))
        self.bus.send(msg)
        if self.verbose: print(msg)

    """ Keep hearing CAN port until timeout is reached. This function should be
        used after sending the commanding data frame (previous function)
    """
    def receive_data(self):
        msg = self.bus.recv(0.01) # Timeout in seconds. None: Wait until data is received.
        if msg is None: 
            print('Timeout occurred, no message.')
        elif self.verbose:
            print(msg)
        return msg

class SensorNode(Node):

    def __init__(self):
        super().__init__('sensor_node')
        self.init_pubs_()

    def init_pubs_(self):
        qos_sensor = qos.qos_profile_sensor_data
        
        if debug:
            self.pot_pub = self.create_publisher(Float32, '/finex/readings/finex_pot', qos_profile=qos_sensor)
            self.gauge_pub = self.create_publisher(Float32, '/finex/readings/finex_gauge', qos_profile=qos_sensor)

            self.pot_raw = self.create_publisher(UInt16, '/finex/readings/pot_raw', 100)
            self.gauge_raw = self.create_publisher(UInt16, '/finex/readings/gauge_raw', 100)
        else:
            self.pot_pub = self.create_publisher(UInt16, '/finex/readings/finex_pot', qos_profile=qos_sensor)
            self.gauge_pub = self.create_publisher(Float32, '/finex/readings/finex_gauge', qos_profile=qos_sensor)


pot_params = [-0.103806228373702, 99.0311418685121]
gauge_params = [0.0307764705882353, -18.3427764705882]
raw_gauge_msgs = deque()
raw_pot_msgs = deque()
debug = False
sos = signal.butter(2, 15, fs=170, output='sos')

def show_plot(signal):

    # Note the extra 'r' at the front
    yf = rfft(signal)
    xf = rfftfreq(2, 1 / 200)

    plt.title("Raw frequencies analysis")
    plt.plot(xf, np.abs(yf))
    plt.show()

def apply_filter(msgs):

    array_filtered = signal.sosfilt(sos, msgs)

    msg_filt = array_filtered[len(array_filtered) - 1]
    return msg_filt

""" Converts from potentiometer reading to angle value.
    The transference funtion depends on the joint being used."""
def pot2angle(pot):
    # parameters for each joint are pre-encoded in the list for efficiency
    # the output angle can be obtained with the given formula:
    return pot_params[0]*pot + pot_params[1]

""" Process the potentiometer readings and returns the
    corresponding angle in degrees. """
def pot_proc(msg):

    if len(raw_pot_msgs) > 340:
        raw_pot_msgs.popleft()

    raw_pot_msgs.append(msg)

    if debug:
        return apply_filter(raw_pot_msgs)

    return max(0, round(pot2angle(apply_filter(raw_pot_msgs))))

""" Process the gauge readings. """
def gauge_proc(msg):

    if len(raw_gauge_msgs) > 340:
        raw_gauge_msgs.popleft()

    raw_gauge_msgs.append(msg)

    if debug:
        return apply_filter(raw_gauge_msgs)

    return gauge_params[0]*apply_filter(raw_gauge_msgs) + gauge_params[1]

def get_sensor_data(msg):
    ''' Bytearray decoding returns tuple. Meaning of arguments:
    >: big endian, <: little endian, H: unsigned short (2 bytes), B: unsigned char'''
    pot_msg = struct.unpack('<H', msg[:2])[0]    # 2 most significant bytes correspond to potentiometer reading
    gauge_msg = struct.unpack('<H', msg[2:4])[0]   # 3-4 bytes correspond to strain_gauge reading

    return pot_msg, gauge_msg

def main(args=None):
    global debug

    rclpy.init(args=args)

    parser = argparse.ArgumentParser()
    parser.add_argument('-D', action='store_true')
    debug = parser.parse_args().D

    sensor_node = SensorNode()
    rate = sensor_node.create_rate(170)
    can_bus = CANbus(channel="can0")

    if debug:
        sensor_node.get_logger().info("[finex] Programm in debug mode...")
    sensor_node.get_logger().info("[finex] Reading sensor data...")

    start = time.time()
    counter = 0
    while rclpy.ok():
        ''' Need to send a msg with the correct ID to receive the
            sensor readings. '''
        can_bus.send_command()
        msg = can_bus.receive_data()

        if debug:
            pot_msg = Float32()
            gauge_msg = Float32()

            pot_msg_raw = UInt16()
            gauge_msg_raw = UInt16()
        else:
            pot_msg = UInt16()
            gauge_msg = Float32()

        if msg is not None:

            pot_raw, gauge_raw = get_sensor_data(msg.data)

            pot_msg.data = pot_proc(pot_raw)
            gauge_msg.data = gauge_proc(gauge_raw)

            if pot_msg.data is not None:
                sensor_node.pot_pub.publish(pot_msg)
            if gauge_msg.data is not None:
                sensor_node.gauge_pub.publish(gauge_msg)

            if debug:
                counter += 1
                if (time.time() - start) > 0.99999:
                    print("Hz:", counter)
                    counter = 0
                    start = time.time()

                pot_msg_raw.data = pot_raw
                gauge_msg_raw.data = gauge_raw

                if pot_msg_raw.data is not None:
                    sensor_node.pot_raw.publish(pot_msg_raw)
                if gauge_msg_raw.data is not None:
                    sensor_node.gauge_raw.publish(gauge_msg_raw)

        rclpy.spin_once(sensor_node)
        rate.sleep()

    sensor_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt as e:
        print("Program finished\n")
        sys.stdout.close()
        os.system('clear')
        raise
