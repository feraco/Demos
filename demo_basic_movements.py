#!/usr/bin/env python3

"""
Demo: Basic Movements
======================
This script demonstrates basic movement commands for the omni-wheel robot.

Commands:
- Move forward
- Move backward
- Move left
- Move right
- Move diagonally
"""

import rclpy
from omni_robot_controller import OmniWheelControlNode
import time


def main(args=None):
    """
    Run a demo of basic movements.
    """
    rclpy.init(args=args)
    node = OmniWheelControlNode()

    try:
        speed = 1.0  # Speed in m/s
        duration = 1.0  # Duration in seconds

        # Move forward
        node.get_logger().info('Moving forward...')
        node.move_in_direction(0, speed, duration)

        # Move backward
        node.get_logger().info('Moving backward...')
        node.move_in_direction(180, speed, duration)

        # Move left
        node.get_logger().info('Moving left...')
        node.move_in_direction(270, speed, duration)

        # Move right
        node.get_logger().info('Moving right...')
        node.move_in_direction(90, speed, duration)

        # Move diagonally (45°)
        node.get_logger().info('Moving diagonally (45°)...')
        node.move_in_direction(45, speed, duration)

    except KeyboardInterrupt:
        node.get_logger().info('Demo interrupted. Stopping motors...')
        node.stop_all_motors()
    finally:
        node.stop_all_motors()
        node.destroy_node()
        rclpy.shutdown()
        print("Basic Movement Demo completed.")


if __name__ == '__main__':
    main()

