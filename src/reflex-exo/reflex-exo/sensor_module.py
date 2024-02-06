#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from std_msgs.msg import UInt16

class SensorNode(Node):

    def __init__(self):
        super().__init__('sensor_node')
        self.init_pubs_()

    def init_pubs_(self):
        self.pot_pub = self.create_publisher(Float32, '/reflex/readings/reflex_pot', 100)
        self.gauge_pub = self.create_publisher(Float32, '/reflex/readings/reflex_gauge', 100)