#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import TwistStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
import time

class autonomous_motion(Node):
    def __init__(self):
        super().__init__('autonomous_motion')
        self.subscriber=self.create_subscription(Odometry,'/odom',self.odom_callback, 10)
        self.get_logger().info("Started autonomous motion")

    def odom_callback(self, msg):
        x=msg.pose.pose.position.x
        y=msg.pose.pose.position.y
        self.get_logger().info(f"Robot Position: x={x:.2f}, y={y:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node=autonomous_motion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()