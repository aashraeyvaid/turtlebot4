#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
import numpy as np
import time


class EscapeMover(Node):
    def __init__(self):
        super().__init__('escape_mover')
        self.close_threshold = 0.75
        self.far_threshold = 2.5
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.state = "forward"
        self.state_start_time = time.time()
        self.rotation_time = 8.0   # seconds to approximate 360° rotation
        self.step_time = 2.0       # move forward/back duration
        self.get_logger().info("EscapeMover node started")

    def sector_average(self, sector, default=10.0):
        valid = [r for r in sector if 0.0 < r < self.far_threshold]
        return np.mean(valid) if valid else default

    def change_state(self, new_state):
        self.state = new_state
        self.state_start_time = time.time()
        self.get_logger().info(f"State changed to: {new_state}")

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        num_readings = len(ranges)
        center = num_readings // 2

        # Shift forward sector slightly to right
        shift = -150
        center = max(0, min(num_readings - 1, center + shift))

        sector_size = 60
        front = ranges[center - sector_size : center + sector_size]
        left = ranges[center + sector_size : center + 3*sector_size]
        right = ranges[center - 3*sector_size : center - sector_size]

        cmd = TwistStamped()
        front_blocked = np.any((front > 0.0) & (front < self.close_threshold))

        if self.state == "forward":
            if front_blocked:
                avg_left = self.sector_average(left)
                avg_right = self.sector_average(right)

                if avg_left > avg_right and avg_left > self.close_threshold:
                    self.change_state("turn_left")
                elif avg_right > self.close_threshold:
                    self.change_state("turn_right")
                else:
                    self.change_state("searching")
            else:
                cmd.twist.linear.x = 0.2

        elif self.state == "turn_left":
            if not front_blocked:
                self.change_state("forward")
            else:
                cmd.twist.angular.z = 0.6

        elif self.state == "turn_right":
            if not front_blocked:
                self.change_state("forward")
            else:
                cmd.twist.angular.z = -0.6

        elif self.state == "searching":
            elapsed = time.time() - self.state_start_time
            if elapsed < self.rotation_time:
                cmd.twist.angular.z = 0.6
            else:
                self.change_state("step_forward")

        elif self.state == "step_forward":
            elapsed = time.time() - self.state_start_time
            if elapsed < self.step_time:
                cmd.twist.linear.x = 0.1
            else:
                self.change_state("searching")

        elif self.state == "step_back":
            elapsed = time.time() - self.state_start_time
            if elapsed < self.step_time:
                cmd.twist.linear.x = -0.1
            else:
                self.change_state("searching")

        elif self.state == "stopped":
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0

        # If search fails after step_forward, fallback to step_back
        if self.state == "step_forward" and not front_blocked:
            self.change_state("forward")
        elif self.state == "searching" and front_blocked and (time.time() - self.state_start_time > self.rotation_time * 1.5):
            self.change_state("step_back")

        self.publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = EscapeMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
