#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import time

class AutoMover(Node):
    def __init__(self):
        super().__init__('auto_mover')
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move_turtlebot)
        self.state = 0
        self.start_time = time.time()
        self.get_logger().info('AutoMover node has started successfully')

    def move_turtlebot(self):
        msg = TwistStamped()

        # Set header timestamp and frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        elapsed = time.time() - self.start_time

        if self.state % 2 == 0:
            if elapsed < 3.0:
                msg.twist.linear.x = 0.2
                msg.twist.angular.z = 0.0
                self.get_logger().info('Moving forward...')
            else:
                self.state += 1
                self.start_time = time.time()
        else:
            if elapsed < 1.5:
                msg.twist.linear.x = 0.0
                msg.twist.angular.z = 0.6
                self.get_logger().info('Rotating...')
            else:
                self.state += 1
                self.start_time = time.time()

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AutoMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
