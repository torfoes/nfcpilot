#!/usr/bin/env python3

import os
import sys
import pygame
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick_node')

        # Set SDL environment variable for background events (important for macOS)
        os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"

        # Initialize Pygame and Joystick
        pygame.init()
        pygame.joystick.init()

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.joy_publisher = self.create_publisher(Joy, '/joystick_data', 10)

        self.get_logger().info('Joystick Node has been started.')

        # Attempt to initialize the joystick
        try:
            joystick_count = pygame.joystick.get_count()
            if joystick_count == 0:
                self.get_logger().error('No joystick connected.')
                sys.exit(1)

            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            self.get_logger().info(f'Joystick connected: {self.joystick.get_name()}')

            # Get the number of axes and buttons
            self.num_axes = self.joystick.get_numaxes()
            self.num_buttons = self.joystick.get_numbuttons()
            self.get_logger().info(f'Number of axes: {self.num_axes}')
            self.get_logger().info(f'Number of buttons: {self.num_buttons}')

        except Exception as e:
            self.get_logger().error(f'Failed to initialize joystick: {e}')
            pygame.quit()
            sys.exit(1)

        # Create a timer to call the joystick callback periodically
        self.timer_period = 0.05  # seconds (20 Hz)
        self.timer = self.create_timer(self.timer_period, self.joystick_callback)

        # Dead zone threshold (adjust based on joystick idle values)
        self.DEAD_ZONE = 0.05  # Adjust as needed

        # Axis indices for linear and angular movement (adjust based on your joystick)
        self.axis_linear_index = 3  # Adjusted based on your provided output
        self.axis_angular_index = 0  # Adjusted as needed

    def joystick_callback(self):
        try:
            # Process Pygame events
            pygame.event.pump()

            # Read all axes and buttons
            axes = [self.joystick.get_axis(i) for i in range(self.num_axes)]
            buttons = [self.joystick.get_button(i) for i in range(self.num_buttons)]

            # Log raw axes values
            self.get_logger().debug(f'Raw axes: {axes}')

            # Apply dead zone to axes
            def apply_dead_zone(value):
                return 0.0 if abs(value) < self.DEAD_ZONE else value

            axes = [apply_dead_zone(value) for value in axes]

            self.get_logger().debug(f'Axes after dead zone: {axes}')

            # Create and publish Joy message
            joy_msg = Joy()
            joy_msg.header.stamp = self.get_clock().now().to_msg()
            joy_msg.axes = axes
            joy_msg.buttons = buttons
            self.joy_publisher.publish(joy_msg)

            # Get linear and angular values
            axis_linear = axes[self.axis_linear_index] if len(axes) > self.axis_linear_index else 0.0
            axis_angular = axes[self.axis_angular_index] if len(axes) > self.axis_angular_index else 0.0

            # Invert axes if necessary
            axis_linear = -axis_linear
            axis_angular = -axis_angular

            # Log the selected axes
            self.get_logger().debug(f'Selected axes - Linear: {axis_linear}, Angular: {axis_angular}')

            # Create and publish Twist message
            twist = Twist()
            twist.linear.x = axis_linear
            twist.angular.z = axis_angular

            self.cmd_vel_publisher.publish(twist)

        except Exception as e:
            self.get_logger().error(f'Error in joystick_callback: {e}')

    def destroy_node(self):
        pygame.quit()
        super().destroy_node()


def main(args=None):
    # Set SDL environment variable for background events
    os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"

    rclpy.init(args=args)
    node = JoystickNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Unhandled exception: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
