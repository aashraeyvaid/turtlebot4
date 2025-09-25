#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
import numpy as np
import time
import subprocess


class AutoMapper(Node):
    def __init__(self):
        super().__init__('auto_mapper')

        # Thresholds
        self.close_threshold = 0.75
        self.far_threshold = 2.5

        # Publisher
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # Subscribers
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        # Robot state
        self.state = "forward"
        self.state_start_time = time.time()
        self.rotation_time = 8.0  # approximate 360° rotation
        self.step_time = 2.0      # small forward/back step

        # Robot pose
        self.robot_x = 0.0
        self.robot_y = 0.0

        # Map
        self.map_received = False

        # Map save management
        self.exploration_start = time.time()
        self.map_save_duration = 300  # seconds (5 minutes)
        self.map_saved = False

        # Timer to periodically check if map should be saved
        self.create_timer(1.0, self.check_save_map)

        self.get_logger().info("AutoMapper node started")

    # --- Helper Functions ---
    def sector_average(self, sector, default=10.0):
        valid = [r for r in sector if 0.0 < r < self.far_threshold]
        return np.mean(valid) if valid else default

    def change_state(self, new_state):
        self.state = new_state
        self.state_start_time = time.time()
        self.get_logger().info(f"State changed to: {new_state}")

    def save_map(self, filename="~/turtlebot_map"):
        self.get_logger().info("Saving map...")
        subprocess.run(["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", filename])
        self.get_logger().info("Map saved successfully")

    def stop_and_save_map(self):
        """Stop the robot and save map once."""
        if not self.map_saved:
            self.save_map()
            self.map_saved = True
            self.change_state("stopped")

    # --- Callbacks ---
    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.get_logger().info(f"Robot pose: x={self.robot_x:.2f}, y={self.robot_y:.2f}")

    def map_callback(self, msg: OccupancyGrid):
        if not self.map_received:
            self.get_logger().info("Map received, SLAM active")
            self.map_received = True

    def scan_callback(self, msg: LaserScan):
        cmd = TwistStamped()

        # If robot is stopped, ensure map is saved and halt motion
        if self.state == "stopped":
            self.stop_and_save_map()
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0
            self.publisher.publish(cmd)
            return

        ranges = np.array(msg.ranges)
        num_readings = len(ranges)
        center = num_readings // 2

        # Shift forward sector (-150) to rotate scanning sector
        shift = -150
        center = max(0, min(num_readings - 1, center + shift))
        sector_size = 60

        front = ranges[center - sector_size : center + sector_size]
        left = ranges[center + sector_size : center + 3*sector_size]
        right = ranges[center - 3*sector_size : center - sector_size]

        front_blocked = np.any((front > 0.0) & (front < self.close_threshold))

        # --- State Machine ---
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

        # Fallbacks to avoid getting stuck
        if self.state == "step_forward" and not front_blocked:
            self.change_state("forward")
        elif self.state == "searching" and front_blocked and (time.time() - self.state_start_time > self.rotation_time * 1.5):
            self.change_state("step_back")

        # Publish movement command
        self.publisher.publish(cmd)

    # --- Timer callback to save map ---
    def check_save_map(self):
        """Automatically save map after duration if not saved yet."""
        if not self.map_saved and (time.time() - self.exploration_start >= self.map_save_duration):
            self.stop_and_save_map()


def main(args=None):
    rclpy.init(args=args)
    node = AutoMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received! Saving map before shutdown...")
    finally:
        # Save map on shutdown if not already saved
        if not node.map_saved:
            node.stop_and_save_map()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
