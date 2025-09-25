#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

class coordinates(Node):
    def __init__(self):
        super().__init__('coordinates')
        self.create_subscription