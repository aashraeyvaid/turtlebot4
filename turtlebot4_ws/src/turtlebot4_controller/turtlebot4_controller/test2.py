#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class AutoMover(Node):
    def __init__(self):
        super().__init__('auto_mover')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move_turtlebot)
        self.state = 0
        self.start_time = time.time()
        self.get_logger().info('AutoMover node has started now')
        self.i=0

    def move_turtlebot(self):
        self.get_logger().info(f"Hello {self.i}")
        self.i+=1
        msg = Twist()

        elapsed = time.time() - self.start_time

        if self.state % 2 == 0:
            # Move forward for 3 seconds
            if elapsed < 3.0:
                msg.linear.x = 0.2
                msg.angular.z = 0.0
            else:
                self.state += 1
                self.start_time = time.time()
        else:
            # Rotate for 1.5 seconds
            if elapsed < 1.5:
                msg.linear.x = 0.0
                msg.angular.z = 0.6
            else:
                self.state += 1
                self.start_time = time.time()

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AutoMover()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()