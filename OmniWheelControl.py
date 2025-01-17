
#!/usr/bin/env python3
# encoding: utf-8

"""
**********************************************************
** Omni-Wheel Motor Control in ROS2                     **
** Developed by: Frederick Feraco                       **
** InnovatED STEM / DroneBlocks Land, Air and Sea       **
**********************************************************

Description:
- Provides motor control for an omni-wheel robot in ROS2.
- Supports movement in all directions (forward, backward, left, right, diagonal).
- Includes rotational movement (rotate left, rotate right) and stop.
"""

import rclpy  # ROS2 Python library
from rclpy.node import Node
from ros_robot_controller_msgs.msg import MotorsState, MotorState
import math
import time
from mecanum import MecanumChassis  # Import the MecanumChassis class

class OmniWheelControlNode(Node):
    def __init__(self):
        super().__init__('omni_wheel_control_node')  # Initialize the node

        # Initialize MecanumChassis with default parameters
        self.mecanum = MecanumChassis(wheelbase=0.1368, track_width=0.1446, wheel_diameter=0.065)
        
        # Publisher for motor commands
        self.motor_pub = self.create_publisher(
            MotorsState, '/ros_robot_controller/set_motor', 10
        )
        self.get_logger().info('Omni-Wheel Control Node initialized and ready for commands.')

    def publish_motor_command(self, motor_states):
        """
        Publishes motor commands to the control topic.
        :param motor_states: List of MotorState messages.
        """
        msg = MotorsState(data=motor_states)
        self.motor_pub.publish(msg)

    def stop_all_motors(self):
        """Stops all motors by setting their speed to 0."""
        motor_states = [MotorState(id=i, rps=0.0) for i in range(1, 5)]
        self.publish_motor_command(motor_states)
        self.get_logger().info('Stopped all motors.')

    def move_with_duration(self, motor_states, duration):
        """
        Moves the robot by publishing motor commands for a specified duration.
        :param motor_states: List of MotorState messages.
        :param duration: Duration (in seconds) for the movement.
        """
        self.publish_motor_command(motor_states)
        self.get_logger().info(f'Movement started for {duration:.2f} seconds...')
        time.sleep(duration)  # Wait for the specified duration
        self.stop_all_motors()

    def move_forward(self, speed, duration):
        """
        Moves the robot forward by setting all motors to drive forward.
        :param speed: Speed of the movement.
        :param duration: Duration (in seconds) for the movement.
        """
        motor_states = [
            MotorState(id=1, rps=speed),  # Front-left forward
            MotorState(id=2, rps=speed),  # Back-left forward
            MotorState(id=3, rps=speed),  # Front-right forward
            MotorState(id=4, rps=speed)   # Back-right forward
        ]
        self.move_with_duration(motor_states, duration)

    def move_backward(self, speed, duration):
        """
        Moves the robot backward by setting all motors to reverse.
        :param speed: Speed of the movement.
        :param duration: Duration (in seconds) for the movement.
        """
        motor_states = [
            MotorState(id=1, rps=-speed),  # Front-left backward
            MotorState(id=2, rps=-speed),  # Back-left backward
            MotorState(id=3, rps=-speed),  # Front-right backward
            MotorState(id=4, rps=-speed)   # Back-right backward
        ]
        self.move_with_duration(motor_states, duration)

    def move_left(self, speed, duration):
        """
        Moves the robot left by setting appropriate motor directions.
        :param speed: Speed of the movement.
        :param duration: Duration (in seconds) for the movement.
        """
        motor_states = [
            MotorState(id=1, rps=-speed),  # Front-left backward
            MotorState(id=2, rps=speed),   # Back-left forward
            MotorState(id=3, rps=speed),   # Front-right forward
            MotorState(id=4, rps=-speed)   # Back-right backward
        ]
        self.move_with_duration(motor_states, duration)

    def move_right(self, speed, duration):
        """
        Moves the robot right by setting appropriate motor directions.
        :param speed: Speed of the movement.
        :param duration: Duration (in seconds) for the movement.
        """
        motor_states = [
            MotorState(id=1, rps=speed),   # Front-left forward
            MotorState(id=2, rps=-speed),  # Back-left backward
            MotorState(id=3, rps=-speed),  # Front-right backward
            MotorState(id=4, rps=speed)    # Back-right forward
        ]
        self.move_with_duration(motor_states, duration)

    def move_in_direction(self, degrees, speed, duration):
        """
        Moves the robot in a specified direction using degrees.
        :param degrees: Direction in degrees (0 = forward, 90 = right, etc.).
        :param speed: Speed of the movement.
        :param duration: Duration (in seconds) for the movement.
        """
        radians = math.radians(degrees)

        # Decompose the speed into x and y components
        linear_x = speed * math.cos(radians)
        linear_y = speed * math.sin(radians)
        angular_z = 0.0  # No rotation for directional movement

        # Use mecanum kinematics to calculate motor speeds
        speeds = self.mecanum.set_velocity(linear_x, linear_y, angular_z)

        # Publish motor speeds for the specified duration
        self.publish_motor_command(speeds.data)
        self.get_logger().info(f'Moving in direction {degrees}° at {speed} m/s for {duration} seconds...')
        time.sleep(duration)
        self.stop_all_motors()

    def rotate_left(self, angular_speed, duration):
        """
        Rotates the robot left (counterclockwise) by applying opposite motor speeds.
        :param angular_speed: Speed of rotation.
        :param duration: Duration (in seconds) for the rotation.
        """
        motor_states = [
            MotorState(id=1, rps=-angular_speed),  # Front-left backward
            MotorState(id=2, rps=-angular_speed),  # Back-left backward
            MotorState(id=3, rps=angular_speed),   # Front-right forward
            MotorState(id=4, rps=angular_speed)    # Back-right forward
        ]
        self.move_with_duration(motor_states, duration)
        self.get_logger().info(f'Rotating left (CCW) at {angular_speed:.2f} RPS for {duration:.2f} seconds.')

    def rotate_right(self, angular_speed, duration):
        """
        Rotates the robot right (clockwise) by applying opposite motor speeds.
        :param angular_speed: Speed of rotation.
        :param duration: Duration (in seconds) for the rotation.
        """
        motor_states = [
            MotorState(id=1, rps=angular_speed),   # Front-left forward
            MotorState(id=2, rps=angular_speed),   # Back-left forward
            MotorState(id=3, rps=-angular_speed),  # Front-right backward
            MotorState(id=4, rps=-angular_speed)   # Back-right backward
        ]
        self.move_with_duration(motor_states, duration)
        self.get_logger().info(f'Rotating right (CW) at {angular_speed:.2f} RPS for {duration:.2f} seconds.')


def main(args=None):
    """
    Main function to demonstrate robot control.
    """
    rclpy.init(args=args)
    node = OmniWheelControlNode()

    try:
        # Example commands
        node.move_forward(0.5, 3.0)  # Forward for 3 seconds
        node.move_backward(0.5, 3.0)  # Backward for 3 seconds
        node.move_left(0.5, 2.0)  # Left for 2 seconds
        node.move_right(0.5, 2.0)  # Right for 2 seconds
        node.move_in_direction(45, 0.5, 2.0)  # Diagonal (45°) for 2 seconds
        node.rotate_left(0.5, 2.0)  # Rotate CCW for 2 seconds
        node.rotate_right(0.5, 2.0)  # Rotate CW for 2 seconds
    finally:
        node.stop_all_motors()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
