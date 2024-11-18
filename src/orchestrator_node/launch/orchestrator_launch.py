from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # NFC Reader Node
        Node(
            package='nfc_card_reader',
            executable='nfc_reader_node',
            name='nfc_reader_node',
            output='screen',
        ),
        # Motor Controller Node
        Node(
            package='stepper_motor_controller',
            executable='motor_controller_node',
            name='motor_controller_node',
            output='screen',
        ),
        # Orchestrator Node
        Node(
            package='orchestrator_node',
            executable='orchestrator_node',
            name='orchestrator_node',
            output='screen',
        ),
    ])
