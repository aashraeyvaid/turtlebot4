#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
import numpy as np
import time
import math


class ExplorerBot(Node):
    def __init__(self):
        super().__init__('explorer_bot')

        # Distance thresholds
        self.very_close = 0.5
        self.close = 0.75
        self.far = 2.5
        self.very_far = 4.0

        # Motion settings
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.state = "forward"
        self.state_start_time = time.time()
        self.rotation_time = 8.0   # seconds for ~360° spin
        self.step_time = 2.0       # move forward/back duration

        self.best_direction = None
        self.best_distance = 0.0
        self.laser_angle_increment = None
        self.laser_angle_min = None

        self.get_logger().info("ExplorerBot with dynamic sectors started")

    def sector_average(self, sector, default=10.0):
        """Average distance of valid readings in a sector"""
        valid = [r for r in sector if 0.0 < r < self.very_far]
        return np.mean(valid) if valid else default

    def change_state(self, new_state):
        """Switch robot state"""
        self.state = new_state
        self.state_start_time = time.time()
        self.get_logger().info(f"State → {new_state}")

    def find_best_direction(self, msg):
        """Scan → check sectors with adaptive width → pick best direction"""
        ranges = np.array(msg.ranges)
        num_readings = len(ranges)
        angle_increment = msg.angle_increment
        angle_min = msg.angle_min

        self.laser_angle_increment = angle_increment
        self.laser_angle_min = angle_min

        best_avg = 0
        best_angle = None

        for i in range(0, num_readings, 10):  # sample every 10 readings
            distance = ranges[i]

            # Dynamic sector size
            if 0.0 < distance <= self.very_close:
                sector_size = 60
            elif distance <= self.close:
                sector_size = 40
            elif distance <= self.far:
                sector_size = 20
            else:
                sector_size = 10

            start = max(0, i - sector_size // 2)
            end = min(num_readings, i + sector_size // 2)
            sector = ranges[start:end]

            avg_dist = self.sector_average(sector)

            if avg_dist > best_avg:
                best_avg = avg_dist
                best_angle = angle_min + i * angle_increment  # radians absolute

        return best_angle, best_avg

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        num_readings = len(ranges)
        center = num_readings // 2

        # Sector check straight ahead
        narrow_sector = ranges[center - 30:center + 30]
        front_blocked = np.any((narrow_sector > 0.0) & (narrow_sector < self.close))

        cmd = TwistStamped()

        # ---------------- State Machine ----------------
        if self.state == "forward":
            if front_blocked:
                self.best_direction, self.best_distance = self.find_best_direction(msg)
                if self.best_direction is not None and self.best_distance > self.close:
                    self.change_state("choose_direction")
                else:
                    self.change_state("searching")
            else:
                cmd.twist.linear.x = 0.2

        elif self.state == "choose_direction":
            if self.best_direction is None:
                self.change_state("searching")
            else:
                # Convert absolute laser angle → relative to straight ahead (0 rad)
                angle_error = self.best_direction  # already radians
                # Center forward at 0 rad: in LaserScan, forward is usually angle=0
                # If your laser publishes [-pi, pi], forward = 0
                if abs(angle_error) < math.radians(10):  # aligned
                    self.change_state("moving_to_goal")
                elif angle_error > 0:
                    cmd.twist.angular.z = 0.5
                else:
                    cmd.twist.angular.z = -0.5

        elif self.state == "moving_to_goal":
            cmd.twist.linear.x = 0.2
            # Stop if blocked again
            if front_blocked:
                self.change_state("forward")

        elif self.state == "searching":
            elapsed = time.time() - self.state_start_time
            if elapsed < self.rotation_time:
                cmd.twist.angular.z = 0.6
            else:
                self.change_state("step_back")

        elif self.state == "step_back":
            elapsed = time.time() - self.state_start_time
            if elapsed < self.step_time:
                cmd.twist.linear.x = -0.1
            else:
                self.change_state("searching")

        elif self.state == "stopped":
            self.change_state("searching")

        # ---------------- Publish command ----------------
        self.publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ExplorerBot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
