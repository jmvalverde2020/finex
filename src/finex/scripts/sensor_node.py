#!/usr/bin/env python3

import sys
import os
import time
import can
import struct
import rclpy
import numpy as np
import argparse

from rclpy.node import Node
from rclpy import qos
from std_msgs.msg import Float32
from std_msgs.msg import UInt16

from numpy import convolve as np_convolve
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
        self.pot_pub = self.create_publisher(UInt16, '/finex/readings/finex_pot', qos_profile=qos_sensor)
        self.gauge_pub = self.create_publisher(Float32, '/finex/readings/finex_gauge', qos_profile=qos_sensor)

        if debug:
            self.pot_raw = self.create_publisher(UInt16, '/finex/readings/pot_raw', 100)
            self.gauge_raw = self.create_publisher(UInt16, '/finex/readings/gauge_raw', 100)

filter_states = [0.0]*2
gauge_data  = [0] * 318 # gauge_data  = [None] * 318
pot_params = [-0.10483871, 99.701613]
gauge_params = [-0.0206526316, 12.8046316]
debug = False

"""FIR filter for gauge readings (Values exported from Simulink)"""
def FIR_filter(raw_gauge):
    global gauge_data

    lst = gauge_data
    lst = deque(lst)
    lst.rotate(1)
    lst = list(lst)
    gauge_data = lst
    gauge_data[0] = raw_gauge
    '''Filter parameters'''
    b = [-7.4029787723175851e-05 , 1.7335114024242748e-05 , 1.6714489931227413e-05 , 1.6914664698078393e-05 , 1.7833760715256158e-05 , 1.936753284143648e-05 , 2.1470175773501382e-05 , 2.40808761912665e-05 , 2.7190109935793652e-05 , 3.076222082586599e-05 , 3.4808770994987535e-05 , 3.9309720691864923e-05 , 4.4291245517772101e-05 , 4.9739932687524062e-05 , 5.5692601758028553e-05 , 6.2140388562404174e-05 , 6.9128401103763559e-05 , 7.6645878781258207e-05 , 8.4749185850655908e-05 , 9.3420483263056495e-05 , 0.00010273336557745961 , 0.00011265121488255643 , 0.0001232480108380004 , 0.00013459636946903071 , 0.00014650139507745382 , 0.00015929463718936134 , 0.00017284015891234515 , 0.00018715745983810908 , 0.00020226733820720105 , 0.0002182498128765064 , 0.00023512578116460171 , 0.00025292651954488888 , 0.00027165870771194241 , 0.00029134995579086707 , 0.00031202217045904067 , 0.00033370999234300623 , 0.00035644549456481851 , 0.00038025899861406803 , 0.00040518081307119023 , 0.0004312340532734016 , 0.0004584503987046613 , 0.00048684430822789812 , 0.00051645354569522907 , 0.0005472828402515276 , 0.00057938190608542079 , 0.00061275090975397421 , 0.0006474127548303588 , 0.00068343468905098371 , 0.00072076580050502625 , 0.00075947631501319265 , 0.00079958110240744553 , 0.0008410975840678226 , 0.00088402254995322081 , 0.00092838867936580823 , 0.00097421127156387751 , 0.0010215107156041454 , 0.0010702884133779744 , 0.0011205575804490652 , 0.0011723248197248871 , 0.0012256044570541449 , 0.0012804046232005564 , 0.0013367332506084143 , 0.0013945919856421184 , 0.0014539800651395143 , 0.001514901520821024 , 0.0015773496022817167 , 0.0016413334084849302 , 0.0017068354011726512 , 0.0017738657468903503 , 0.0018424107865905371 , 0.0019124473527039961 , 0.0019839905437757511 , 0.0020570044913864856 , 0.0021314772554310206 , 0.0022073935243709774 , 0.0022847435564257133 , 0.0023634945231977844 , 0.0024436264036744332 , 0.0025251133220893333 , 0.0026079357701856605 , 0.0026920587401893263 , 0.0027774521193260461 , 0.0028640791633547311 , 0.0029519081765462122 , 0.0030409006777833555 , 0.0031310205981640729 , 0.0032222250469656809 , 0.0033144685281284786 , 0.0034077089852878432 , 0.0035018951402814484 , 0.003596988201224169 , 0.0036929278229154137 , 0.0037896678870492553 , 0.0038871566241322287 , 0.0039853261730970861 , 0.0040841294169390663 , 0.0041835059838843079 , 0.0042833933387723512 , 0.0043837261368993262 , 0.0044844488974304479 , 0.0045854932957565195 , 0.0046867921209211068 , 0.0047882735931617116 , 0.0048898745755680383 , 0.0049915231788431611 , 0.0050931497285813497 , 0.0051946791297664465 , 0.0052960417422814373 , 0.0053971612955007083 , 0.0054979658306310002 , 0.0055983797994516738 , 0.0056983253408818286 , 0.0057977277900710658 , 0.0058965061241457476 , 0.0059945906110298215 , 0.0060918990398812472 , 0.0061883544421090396 , 0.0062838835100025494 , 0.0063784040723008813 , 0.0064718392683248841 , 0.0065641156880257012 , 0.0066551575218083692 , 0.0067448837961064734 , 0.0068332211341646538 , 0.0069200954455324835 , 0.0070054334691378312 , 0.0070891574289693657 , 0.0071711976967911263 , 0.0072514833710739506 , 0.0073299462238696437 , 0.0074065136345697734 , 0.0074811207162423544 , 0.0075536986382993528 , 0.00762418337802596 , 0.0076925122443991002 , 0.0077586234882858086 , 0.0078224582372105698 , 0.0078839551310774785 , 0.0079430619696299314 , 0.0079997234806384564 , 0.0080538863175640511 , 0.0081055017546585971 , 0.0081545228891952252 , 0.0082009008815607833 , 0.008244594453582119 , 0.0082855652960434623 , 0.0083237733919798405 , 0.0083591810162194566 , 0.0083917558821141533 , 0.0084214702626896412 , 0.0084482939561720712 , 0.0084722007770392724 , 0.0084931682078880754 , 0.0085111792876931842 , 0.0085262133908074918 , 0.0085382578791410729 , 0.0085473009945351702 , 0.0085533341831194563 , 0.0085563512135527313 , 0.0085563512135527313 , 0.0085533341831194563 , 0.0085473009945351702 , 0.0085382578791410729 , 0.0085262133908074918 , 0.0085111792876931842 , 0.0084931682078880754 , 0.0084722007770392724 , 0.0084482939561720712 , 0.0084214702626896412 , 0.0083917558821141533 , 0.0083591810162194566 , 0.0083237733919798405 , 0.0082855652960434623 , 0.008244594453582119 , 0.0082009008815607833 , 0.0081545228891952252 , 0.0081055017546585971 , 0.0080538863175640511 , 0.0079997234806384564 , 0.0079430619696299314 , 0.0078839551310774785 , 0.0078224582372105698 , 0.0077586234882858086 , 0.0076925122443991002 , 0.00762418337802596 , 0.0075536986382993528 , 0.0074811207162423544 , 0.0074065136345697734 , 0.0073299462238696437 , 0.0072514833710739506 , 0.0071711976967911263 , 0.0070891574289693657 , 0.0070054334691378312 , 0.0069200954455324835 , 0.0068332211341646538 , 0.0067448837961064734 , 0.0066551575218083692 , 0.0065641156880257012 , 0.0064718392683248841 , 0.0063784040723008813 , 0.0062838835100025494 , 0.0061883544421090396 , 0.0060918990398812472 , 0.0059945906110298215 , 0.0058965061241457476 , 0.0057977277900710658 , 0.0056983253408818286 , 0.0055983797994516738 , 0.0054979658306310002 , 0.0053971612955007083 , 0.0052960417422814373 , 0.0051946791297664465 , 0.0050931497285813497 , 0.0049915231788431611 , 0.0048898745755680383 , 0.0047882735931617116 , 0.0046867921209211068 , 0.0045854932957565195 , 0.0044844488974304479 , 0.0043837261368993262 , 0.0042833933387723512 , 0.0041835059838843079 , 0.0040841294169390663 , 0.0039853261730970861 , 0.0038871566241322287 , 0.0037896678870492553 , 0.0036929278229154137 , 0.003596988201224169 , 0.0035018951402814484 , 0.0034077089852878432 , 0.0033144685281284786 , 0.0032222250469656809 , 0.0031310205981640729 , 0.0030409006777833555 , 0.0029519081765462122 , 0.0028640791633547311 , 0.0027774521193260461 , 0.0026920587401893263 , 0.0026079357701856605 , 0.0025251133220893333 , 0.0024436264036744332 , 0.0023634945231977844 , 0.0022847435564257133 , 0.0022073935243709774 , 0.0021314772554310206 , 0.0020570044913864856 , 0.0019839905437757511 , 0.0019124473527039961 , 0.0018424107865905371 , 0.0017738657468903503 , 0.0017068354011726512 , 0.0016413334084849302 , 0.0015773496022817167 , 0.001514901520821024 , 0.0014539800651395143 , 0.0013945919856421184 , 0.0013367332506084143 , 0.0012804046232005564 , 0.0012256044570541449 , 0.0011723248197248871 , 0.0011205575804490652 , 0.0010702884133779744 , 0.0010215107156041454 , 0.00097421127156387751 , 0.00092838867936580823 , 0.00088402254995322081 , 0.0008410975840678226 , 0.00079958110240744553 , 0.00075947631501319265 , 0.00072076580050502625 , 0.00068343468905098371 , 0.0006474127548303588 , 0.00061275090975397421 , 0.00057938190608542079 , 0.0005472828402515276 , 0.00051645354569522907 , 0.00048684430822789812 , 0.0004584503987046613 , 0.0004312340532734016 , 0.00040518081307119023 , 0.00038025899861406803 , 0.00035644549456481851 , 0.00033370999234300623 , 0.00031202217045904067 , 0.00029134995579086707 , 0.00027165870771194241 , 0.00025292651954488888 , 0.00023512578116460171 , 0.0002182498128765064 , 0.00020226733820720105 , 0.00018715745983810908 , 0.00017284015891234515 , 0.00015929463718936134 , 0.00014650139507745382 , 0.00013459636946903071 , 0.0001232480108380004 , 0.00011265121488255643 , 0.00010273336557745961 , 9.3420483263056495e-05 , 8.4749185850655908e-05 , 7.6645878781258207e-05 , 6.9128401103763559e-05 , 6.2140388562404174e-05 , 5.5692601758028553e-05 , 4.9739932687524062e-05 , 4.4291245517772101e-05 , 3.9309720691864923e-05 , 3.4808770994987535e-05 , 3.076222082586599e-05 , 2.7190109935793652e-05 , 2.40808761912665e-05 , 2.1470175773501382e-05 , 1.936753284143648e-05 , 1.7833760715256158e-05 , 1.6914664698078393e-05 , 1.6714489931227413e-05 , 1.7335114024242748e-05 , -7.4029787723175851e-05]
    y = np.multiply(b, gauge_data)
    return y.sum()

"""Biquad filter for potentiometer readings (Exported from Simulink)"""
def biquad_filter(pot):
    '''Filter parameters'''
    a0 = 0.000956278151630223
    a1 = -1.9106427906788
    a2 = 0.914467903285316
    b0 = 2.0
    den_accum = (a0*pot - a1*filter_states[0]) - a2*filter_states[1]
    filt_pot = (b0*filter_states[0] + den_accum) + filter_states[1]
    '''Updating filter states'''
    filter_states[1] = filter_states[0]
    filter_states[0] = den_accum
    return filt_pot

""" Converts from potentiometer reading to angle value.
    The transference funtion depends on the joint being used."""
def pot2angle(pot):
    # parameters for each joint are pre-encoded in the list for efficiency
    # the output angle can be obtained with the given formula:
    return pot_params[0]*pot + pot_params[1]

""" Process the potentiometer readings and returns the
    corresponding angle in degrees. """
def pot_proc(msg):
    return pot2angle(biquad_filter(msg))

""" Process the gauge readings. """
def gauge_proc(msg):

    if debug:
        return FIR_filter(msg)
    
    return gauge_params[0]*FIR_filter(msg) + gauge_params[1]

def get_sensor_data(msg):
    ''' Bytearray decoding returns tuple. Meaning of arguments:
    >: big endian, <: little endian, H: unsigned short (2 bytes), B: unsigned char'''
    pot_msg = struct.unpack('<H', msg[:2])[0]    # 2 most significant bytes correspond to potentiometer reading
    gauge_msg = struct.unpack('<H', msg[2:4])[0]   # 3-4 bytes correspond to strain_gauge reading

    return max(0, round(pot_proc(pot_msg))), gauge_proc(gauge_msg)

def get_data_raw(msg):
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
        if msg is not None:
            pot_msg = UInt16()
            gauge_msg = Float32()

            pot_msg.data, gauge_msg.data = get_sensor_data(msg.data)
            if pot_msg.data is not None:
                #print("sending:", pot_msg)
                sensor_node.pot_pub.publish(pot_msg)
            if gauge_msg.data is not None:
                #print("sending:", gauge_msg)
                sensor_node.gauge_pub.publish(gauge_msg)

            
    
            if debug:
                counter += 1
                if (time.time() - start) > 0.99999:
                    print("Hz:", counter)
                    counter = 0
                    start = time.time()

                pot_msg_raw = UInt16()
                gauge_msg_raw = UInt16()
                pot_msg_raw.data, gauge_msg_raw.data = get_data_raw(msg.data)
                if pot_msg_raw.data is not None:
                    sensor_node.pot_raw.publish(pot_msg_raw)
                if gauge_msg_raw.data is not None:
                    sensor_node.gauge_raw.publish(gauge_msg_raw)
    
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