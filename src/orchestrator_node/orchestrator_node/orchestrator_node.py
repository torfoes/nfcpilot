#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class OrchestratorNode(Node):
    def __init__(self):
        super().__init__('orchestrator_node')
        self.get_logger().info('Orchestrator Node has been started.')

        # Create a subscriber to listen for NFC card data
        self.subscription = self.create_subscription(
            String,
            'nfc_card_data',
            self.nfc_callback,
            10
        )

        # Create a publisher to send commands to the motor controller
        self.publisher_ = self.create_publisher(
            String,
            'motor_commands',
            10
        )

        # State variable to prevent multiple triggers
        self.processing = False

        # Timer placeholder
        self.timer = None

    def nfc_callback(self, msg):
        if not self.processing:
            self.processing = True
            self.get_logger().info('NFC card detected. Starting motor sequence.')

            # Start the motor (motor_id=1, speed=1000)
            start_command = String()
            start_command.data = 'START,1,1000'
            self.publisher_.publish(start_command)

            # Schedule the motor to stop after 1 second
            self.timer = self.create_timer(1.0, self.stop_motor_callback)

    def stop_motor_callback(self):
        # Stop the motor
        stop_command = String()
        stop_command.data = 'STOP,1'
        self.publisher_.publish(stop_command)
        self.get_logger().info('Motor stopped.')

        # Reset processing state
        self.processing = False

        # Cancel and destroy the timer
        self.timer.cancel()
        self.destroy_timer(self.timer)
        self.timer = None

def main(args=None):
    rclpy.init(args=args)
    node = OrchestratorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Orchestrator Node stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
