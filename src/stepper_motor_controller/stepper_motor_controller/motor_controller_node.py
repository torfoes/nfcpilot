#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .hardware_interface import StepperMotorInterface

import platform

if platform.system() == 'Darwin':
    from .mock_stepper_motor import MockStepperMotor as StepperMotorImplementation
else:
    from .gpio_stepper_motor import GPIOStepperMotor as StepperMotorImplementation


class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller_node')
        self.get_logger().info('Stepper Motor Controller Node has been started.')

        # motor configurations
        motor_configs = {
            1: {
                'step_pin': 17,
                'dir_pin': 4,
                'enable_pin': 22,
                'initial_speed': 0,
            },
            # 2: {
            #     'step_pin': 23,
            #     'dir_pin': 24,
            #     'enable_pin': 25,
            #     'initial_speed': 0,
            # },
        }

        # initialize the hardware interface with the node's logger
        self.motor_controller = StepperMotorImplementation(self.get_logger())
        self.motor_controller.initialize_motors(motor_configs)

        self.subscription = self.create_subscription(
            String,
            'motor_commands',
            self.command_callback,
            10
        )

    def command_callback(self, msg):
        command = msg.data.strip()
        self.get_logger().info(f'Received command: {command}')

        parts = command.split(',')
        if not parts:
            self.get_logger().error('Invalid command format.')
            return

        cmd_type = parts[0].upper()

        try:
            if cmd_type == 'START':
                motor_id = int(parts[1])
                speed = int(parts[2])
                self.motor_controller.start_motor(motor_id, speed)
            elif cmd_type == 'STOP':
                motor_id = int(parts[1])
                self.motor_controller.stop_motor(motor_id)
            elif cmd_type == 'SET_SPEED':
                motor_id = int(parts[1])
                speed = int(parts[2])
                self.motor_controller.set_speed(motor_id, speed)
            else:
                self.get_logger().warn(f'Unknown command: {cmd_type}')
        except (IndexError, ValueError) as e:
            self.get_logger().error(f'Error parsing command: {e}')

    def destroy_node(self):
        self.motor_controller.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Motor Controller Node stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
