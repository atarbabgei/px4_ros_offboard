from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
import os

def generate_launch_description():
    # Paths to the configuration files
    config_file_path = os.path.join(
        get_package_share_directory('px4_ros_offboard'),
        'config',
        'joy_config.yaml'
    )

    keyboard_config_file_path = os.path.join(
        get_package_share_directory('px4_ros_offboard'),
        'config',
        'keyboard_joy_config.yaml'
    )

    return LaunchDescription([
        # Declare the 'input' argument ('joystick' or 'keyboard')
        DeclareLaunchArgument(
            'input',
            default_value='keyboard',  # Default to keyboard
            description='Select the input device: joystick or keyboard'
        ),

        # Node for the joy_node, launched if input is 'joystick'
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            condition=IfCondition(
                PythonExpression([
                    "'",
                    LaunchConfiguration('input'),
                    "' == 'joystick'"
                ])
            ),  # Launch joy_node if input is 'joystick'
        ),

        # Node for the keyboard_joy, launched if input is 'keyboard'
        Node(
            package='keyboard_joy',
            executable='joy_node',
            name='keyboard_joy_node',
            output='screen',
            condition=IfCondition(
                PythonExpression([
                    "'",
                    LaunchConfiguration('input'),
                    "' == 'keyboard'"
                ])
            ),  # Launch keyboard_joy_node if input is 'keyboard'
        ),

        # Node for the altitude_hold_control with joystick config, if input is 'joystick'
        Node(
            package='px4_ros_offboard',
            executable='altitude_hold_control',
            name='altitude_hold_control',
            output='screen',
            parameters=[{'config': config_file_path}],  # Pass joystick config file
            condition=IfCondition(
                PythonExpression([
                    "'",
                    LaunchConfiguration('input'),
                    "' == 'joystick'"
                ])
            ),  # Launch with joystick config
        ),

        # Node for the altitude_hold_control with keyboard config, if input is 'keyboard'
        Node(
            package='px4_ros_offboard',
            executable='altitude_hold_control',
            name='altitude_hold_control_keyboard',
            output='screen',
            parameters=[{'config': keyboard_config_file_path}],  # Pass keyboard config file
            condition=IfCondition(
                PythonExpression([
                    "'",
                    LaunchConfiguration('input'),
                    "' == 'keyboard'"
                ])
            ),  # Launch with keyboard config
        ),
    ])
