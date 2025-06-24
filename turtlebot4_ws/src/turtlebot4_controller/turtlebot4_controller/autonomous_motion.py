#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
import time

class autonomous_motion(Node):
    def __init__(self):
        super().__inti__('autonomous_motion')
        self.publisher=self.create_publisher(TwistStamped,'/cmd_vel',10)
        self.state=0
        self.start_time=time.time()
        self.get_logger("Started autonomous motion")
        self.timer=self.create_timer(0.1,self.turtlebot4_motion)

    def turtlebot4_motion(self):
        msg=TwistStamped
        msg.header.stamp=self.get_clock().now().to_msg()
        msg.header.frame_id='base_link'

        elapsed=time.time() - self.start_time
        
        