#!/usr/bin/env python3

"""
Demo: 45° Movement Pattern (Command by Command)
================================================
This script demonstrates a movement pattern in 45° increments without a loop.

Commands:
1. Move forward at 0° and back to center.
2. Move forward at 45° and back to center.
3. Repeat for 90°, 135°, ..., 315°.
"""

import rclpy
from omni_robot_controller import OmniWheelControlNode
import time


def main(args=None):
    """
    Run a demo of 45° movement pattern command by command.
    """
    rclpy.init(args=args)
    node = OmniWheelControlNode()

    try:
        speed = 1.0  # Speed in m/s
        duration = 1.0  # Movement duration in seconds

        # Move forward to 0° and back to center
        node.get_logger().info('Moving forward to 0°...')
        node.move_in_direction(0, speed, duration)
        node.get_logger().info('Returning to center from 180°...')
        node.move_in_direction(180, speed, duration)

        # Move forward to 45° and back to center
        node.get_logger().info('Moving forward to 45°...')
        node.move_in_direction(45, speed, duration)
        node.get_logger().info('Returning to center from 225°...')
        node.move_in_direction(225, speed, duration)

        # Move forward to 90° and back to center
        node.get_logger().info('Moving forward to 90°...')
        node.move_in_direction(90, speed, duration)
        node.get_logger().info('Returning to center from 270°...')
        node.move_in_direction(270, speed, duration)

        # Move forward to 135° and back to center
        node.get_logger().info('Moving forward to 135°...')
        node.move_in_direction(135, speed, duration)
        node.get_logger().info('Returning to center from 315°...')
        node.move_in_direction(315, speed, duration)

        # Move forward to 180° and back to center
        node.get_logger().info('Moving forward to 180°...')
        node.move_in_direction(180, speed, duration)
        node.get_logger().info('Returning to center from 0°...')
        node.move_in_direction(0, speed, duration)

        # Move forward to 225° and back to center
        node.get_logger().info('Moving forward to 225°...')
        node.move_in_direction(225, speed, duration)
        node.get_logger().info('Returning to center from 45°...')
        node.move_in_direction(45, speed, duration)

        # Move forward to 270° and back to center
        node.get_logger().info('Moving forward to 270°...')
        node.move_in_direction(270, speed, duration)
        node.get_logger().info('Returning to center from 90°...')
        node.move_in_direction(90, speed, duration)

        # Move forward to 315° and back to center
        node.get_logger().info('Moving forward to 315°...')
        node.move_in_direction(315, speed, duration)
        node.get_logger().info('Returning to center from 135°...')
        node.move_in_direction(135, speed, duration)

        node.get_logger().info('45° Movement Pattern (Command by Command) completed.')

    except KeyboardInterrupt:
        node.get_logger().info('Demo interrupted. Stopping motors...')
        node.stop_all_motors()

    finally:
        node.stop_all_motors()
        node.destroy_node()
        rclpy.shutdown()
        print("45° Movement Pattern Demo (Command by Command) completed.")


if __name__ == '__main__':
    main()

