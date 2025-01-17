#!/usr/bin/env python3

"""
Demo: Rotational Movements
===========================
This script demonstrates rotational commands for the omni-wheel robot.

Commands:
- Rotate left (counterclockwise)
- Rotate right (clockwise)
"""

import rclpy
from omni_robot_controller import OmniWheelControlNode
import time


def main(args=None):
    """
    Run a demo of rotational movements.
    """
    rclpy.init(args=args)
    node = OmniWheelControlNode()

    try:
        angular_speed = 0.5  # Angular speed in radians per second
        duration = 2.0  # Duration in seconds

        # Rotate left (counterclockwise)
        node.get_logger().info('Rotating left (CCW)...')
        node.rotate_left(angular_speed, duration)

        # Rotate right (clockwise)
        node.get_logger().info('Rotating right (CW)...')
        node.rotate_right(angular_speed, duration)

    except KeyboardInterrupt:
        node.get_logger().info('Demo interrupted. Stopping motors...')
        node.stop_all_motors()
    finally:
        node.stop_all_motors()
        node.destroy_node()
        rclpy.shutdown()
        print("Rotational Movement Demo completed.")


if __name__ == '__main__':
    main()

