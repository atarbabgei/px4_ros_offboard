#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class JoystickInputs:
    def __init__(self, node: Node):
        """
        Initializes the JoystickInputs with a ROS2 node.

        :param node: The ROS2 node that will handle the subscription.
        """
        self.node = node

        # Declare a parameter for the configuration file path
        self.node.declare_parameter('config', '')

        # Load joystick configuration from YAML file
        self.load_config()

        # Subscribe to the Joy topic
        self.node.create_subscription(
            Joy, '/joy', self.joy_callback, 10
        )  # Using default QoS profile for Joy

        # Initialize joystick state
        self.joy_axes = []
        self.joy_buttons = []

    def load_config(self):
        """Load the joystick configuration from a YAML file."""
        # Get the config parameter
        config_file_path = self.node.get_parameter('config').get_parameter_value().string_value

        if not config_file_path:
            # Use default file path if no parameter provided
            package_share_directory = get_package_share_directory('px4_ros_offboard')
            config_file_path = os.path.join(package_share_directory, 'config', 'keyboard_joy_config.yaml')

        # Load the YAML file
        try:
            with open(config_file_path, 'r') as file:
                config = yaml.safe_load(file)
                self.node.get_logger().info(f"Joystick Configuration loaded from: {config_file_path}")
        except FileNotFoundError:
            self.node.get_logger().error(f"Joystick Configuration file not found: {config_file_path}")
            config = {}

        # Extract axes and buttons mappings from the loaded YAML file
        self.axes_mapping = config.get('axes', {})
        self.buttons_mapping = config.get('buttons', {})

    def joy_callback(self, joy_msg):
        """Callback function for joy topic subscriber."""
        self.joy_axes = joy_msg.axes
        self.joy_buttons = joy_msg.buttons

    def get_axes(self):
        """Get the current joystick axes."""
        return self.joy_axes

    def get_buttons(self):
        """Get the current joystick buttons."""
        return self.joy_buttons

    def is_kill_pressed(self):
        """Check if the kill button is pressed based on the configuration."""
        kill_button_index = self.buttons_mapping.get('kill', -1)
        return self.joy_buttons[kill_button_index] == 1 if kill_button_index >= 0 and len(self.joy_buttons) > kill_button_index else False

    def is_arm_pressed(self):
        """Check if the arm button is pressed based on the configuration."""
        arm_button_index = self.buttons_mapping.get('arm', -1)
        return self.joy_buttons[arm_button_index] == 1 if arm_button_index >= 0 and len(self.joy_buttons) > arm_button_index else False

    def is_takeoff_pressed(self):
        """Check if the takeoff button is pressed based on the configuration."""
        takeoff_button_index = self.buttons_mapping.get('takeoff', -1)
        return self.joy_buttons[takeoff_button_index] == 1 if takeoff_button_index >= 0 and len(self.joy_buttons) > takeoff_button_index else False

    def is_start_trajectory_pressed(self):
        """Check if the start trajectory button is pressed based on the configuration."""
        start_trajectory_button_index = self.buttons_mapping.get('start_trajectory', -1)
        return self.joy_buttons[start_trajectory_button_index] == 1 if start_trajectory_button_index >= 0 and len(self.joy_buttons) > start_trajectory_button_index else False

    def is_stop_trajectory_pressed(self):
        """Check if the stop trajectory button is pressed based on the configuration."""
        stop_trajectory_button_index = self.buttons_mapping.get('stop_trajectory', -1)
        return self.joy_buttons[stop_trajectory_button_index] == 1 if stop_trajectory_button_index >= 0 and len(self.joy_buttons) > stop_trajectory_button_index else False

    def get_roll(self):
        """Get the roll axis value based on the configuration."""
        roll_axis_index = self.axes_mapping.get('roll', -1)
        return self.joy_axes[roll_axis_index] if roll_axis_index >= 0 and len(self.joy_axes) > roll_axis_index else 0.0

    def get_pitch(self):
        """Get the pitch axis value based on the configuration."""
        pitch_axis_index = self.axes_mapping.get('pitch', -1)
        return self.joy_axes[pitch_axis_index] if pitch_axis_index >= 0 and len(self.joy_axes) > pitch_axis_index else 0.0

    def get_yaw(self):
        """Get the yaw axis value based on the configuration."""
        yaw_axis_index = self.axes_mapping.get('yaw', -1)
        return self.joy_axes[yaw_axis_index] if yaw_axis_index >= 0 and len(self.joy_axes) > yaw_axis_index else 0.0

    def get_throttle(self):
        """Get the throttle axis value based on the configuration."""
        throttle_axis_index = self.axes_mapping.get('throttle', -1)
        return self.joy_axes[throttle_axis_index] if throttle_axis_index >= 0 and len(self.joy_axes) > throttle_axis_index else 0.0
