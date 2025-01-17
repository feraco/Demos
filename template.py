#!/usr/bin/env python3

"""
Demo: Empty Template for Omni-Wheel Robot Commands
===================================================
This file is a blank template for testing and creating custom movement patterns
for the omni-wheel robot.

How to Use:
1. Add your commands in the marked sections below.
2. Use the `OmniWheelControlNode` methods to control the robot:
    - `move_in_direction(degrees, speed, duration)`
        - Move the robot in a specific direction.
        - Example: `node.move_in_direction(90, 1.0, 2.0)` (move right at 1 m/s for 2 seconds).
    - `rotate_left(angular_speed, duration)`
        - Rotate the robot counterclockwise.
        - Example: `node.rotate_left(0.5, 2.0)` (rotate CCW at 0.5 rad/s for 2 seconds).
    - `rotate_right(angular_speed, duration)`
        - Rotate the robot clockwise.
        - Example: `node.rotate_right(0.5, 2.0)` (rotate CW at 0.5 rad/s for 2 seconds).
    - `stop_all_motors()`
        - Stop the robot immediately.
"""

import rclpy
from omni_robot_controller import OmniWheelControlNode
import time


def main(args=None):
    """
    Template for controlling the omni-wheel robot.
    """
    rclpy.init(args=args)
    node = OmniWheelControlNode()

    try:
        # --- Setup Parameters ---
        speed = 1.0  # Speed in m/s
        duration = 1.0  # Duration in seconds
        angular_speed = 0.5  # Angular speed in radians per second

        # --- Add Commands Below ---
        
        # Example Command: Move forward
        # node.get_logger().info('Moving forward...')
        # node.move_in_direction(0, speed, duration)
        
        # Example Command: Stop all motors
        # node.get_logger().info('Stopping all motors...')
        # node.stop_all_motors()

        # Example Command: Rotate left
        # node.get_logger().info('Rotating left...')
        # node.rotate_left(angular_speed, duration)

        # --- Add More Commands as Needed ---
        
        # Example Command: Move diagonally at 45°
        # node.get_logger().info('Moving diagonally at 45°...')
        # node.move_in_direction(45, speed, duration)

        # --- End of Commands ---

        node.get_logger().info('Demo completed.')

    except KeyboardInterrupt:
        # Handle user interruption gracefully
        node.get_logger().info('Demo interrupted. Stopping motors...')
        node.stop_all_motors()

    finally:
        # Ensure the motors stop and cleanup is done
        node.stop_all_motors()
        node.destroy_node()
        rclpy.shutdown()
        print("Omni-Wheel Robot Demo completed.")


if __name__ == '__main__':
    main()
