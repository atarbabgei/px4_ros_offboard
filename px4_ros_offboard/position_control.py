#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import OffboardControlMode, GotoSetpoint, VehicleCommand, VehicleLocalPosition, TrajectorySetpoint, VehicleStatus
from geometry_msgs.msg import Twist
import numpy as np
from math import pi


from px4_ros_offboard.joy_inputs import JoystickInputs  # Ensure this is correctly imported


class PositionControl(Node):
    """Node for controlling a vehicle in position control mode using joystick inputs."""

    def __init__(self) -> None:
        super().__init__('position_control')

        # Declare the use_world_frame parameter (default is True)
        self.declare_parameter('use_world_frame', True)
        # Fetch the parameter value
        self.use_world_frame = self.get_parameter('use_world_frame').get_parameter_value().bool_value
        self.get_logger().info(f"Position control frame set to {'World frame (NED)' if self.use_world_frame else 'Vehicle-relative frame (FLU)'}")


        # Initialize joystick inputs
        self.joystick_inputs = JoystickInputs(self)

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        # In the __init__ method, replace the publisher for trajectory setpoint with goto_setpoint
        self.goto_setpoint_publisher = self.create_publisher(
            GotoSetpoint, '/fmu/in/goto_setpoint', qos_profile)

        # Create subscribers
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_status = VehicleStatus()
        self.current_yaw = 0.0  # Initialize the current yaw angle
        self.current_position = [0.0, 0.0, 0.0]  # Initialize current position

        # Control parameters
        self.yaw_setpoint = 0.0  # Desired yaw setpoint
        self.position_setpoint = [0.0, 0.0, 0.0]  # Desired position setpoint
        self.max_position_change = 0.5  # Maximum position change per input in meters
        self.max_position_rate = 1.0  # Limit rate of position change in meters per second
        self.yaw_gain = 0.1  # Maximum yaw rate in rad/s

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.02, self.timer_callback)  # Increased frequency for smoother control

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.current_position = [
            vehicle_local_position.x,
            vehicle_local_position.y,
            vehicle_local_position.z
        ]
        self.current_yaw = vehicle_local_position.heading

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def kill_vehicle(self):
        """Send a kill command to the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_FLIGHTTERMINATION, param1=1.0)
        self.get_logger().info('Kill command sent')

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True  # Enable position control
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_control_mode_publisher.publish(msg)

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()

        # Handle arming and kill commands using joystick buttons
        if self.joystick_inputs.is_kill_pressed() and not getattr(self, 'kill_sent', False):
            self.kill_vehicle()
            self.kill_sent = True
        elif not self.joystick_inputs.is_kill_pressed():
            self.kill_sent = False  # Reset kill flag when the kill button is not pressed

        if self.joystick_inputs.is_arm_pressed() and not getattr(self, 'arm_sent', False):
            if not self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                self.arm()
                self.arm_sent = True
        elif not self.joystick_inputs.is_arm_pressed():
            self.arm_sent = False  # Reset arm flag when the arm button is not pressed

        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            # Read joystick inputs for position control
            pitch_input = self.joystick_inputs.get_pitch()  # Forward/backward
            roll_input = self.joystick_inputs.get_roll()    # Left/right
            throttle_input = self.joystick_inputs.get_throttle()  # Up/down
            yaw_input = self.joystick_inputs.get_yaw()      # Yaw rate

            # Calculate shortest angular distance for yaw setpoint adjustment
            yaw_delta = -yaw_input * self.yaw_gain
            yaw_difference = self.shortest_angular_distance(self.current_yaw, self.current_yaw + yaw_delta)
            self.yaw_setpoint = self.current_yaw + yaw_difference
            self.yaw_setpoint = self.normalize_angle(self.yaw_setpoint)

            # Update position setpoints with rate limiting
            self.position_setpoint[0] += self.limit_position_change(self.current_position[0], roll_input * self.max_position_change)
            self.position_setpoint[1] += self.limit_position_change(self.current_position[1], pitch_input * self.max_position_change)
            # Ensure Z setpoint is always negative or zero
            self.position_setpoint[2] += self.limit_position_change(self.current_position[2], throttle_input * self.max_position_change)
            if self.position_setpoint[2] < 0:
                self.position_setpoint[2] = 0

            # Publish the new setpoint
            #self.publish_trajectory_setpoint(self.position_setpoint[0], self.position_setpoint[1], -self.position_setpoint[2], self.current_yaw)
            # Publish_trajectory_setpoint with publish_goto_setpoint
            self.publish_goto_setpoint(self.position_setpoint[1], -self.position_setpoint[0], -self.position_setpoint[2], self.yaw_setpoint)

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

    def normalize_angle(self, angle):
        """Normalize an angle to the range -pi to pi."""
        return (angle + pi) % (2 * pi) - pi
    
    def shortest_angular_distance(self, from_angle, to_angle):
        """Calculate the shortest distance between two angles."""
        delta_angle = to_angle - from_angle
        return (delta_angle + pi) % (2 * pi) - pi
    
    def limit_position_change(self, current_value, desired_change):
        """Limit the rate of change for the position setpoint."""
        max_change = self.max_position_rate * 0.02  # Limit rate to max_position_rate m/s
        return np.clip(desired_change, -max_change, max_change)

    def publish_trajectory_setpoint(self, position_x: float, position_y: float, position_z: float, yaw: float):
        """Publish the position setpoint as a TrajectorySetpoint."""
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [float(position_x), float(position_y), float(position_z)]
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.jerk = [float('nan'), float('nan'), float('nan')]
        msg.yaw = float(yaw)
        msg.yawspeed = float('nan')
        self.trajectory_setpoint_publisher.publish(msg)
    def publish_goto_setpoint(self, position_x: float, position_y: float, position_z: float, heading: float):
        """Publish the position setpoint as a GotoSetpoint."""
        msg = GotoSetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [float(position_x), float(position_y), float(position_z)]
        msg.flag_control_heading = True
        msg.heading = heading
        msg.flag_set_max_horizontal_speed = False
        msg.max_horizontal_speed = 0.0
        msg.flag_set_max_vertical_speed = False
        msg.max_vertical_speed = 0.0
        msg.flag_set_max_heading_rate = False
        msg.max_heading_rate = 0.0
        self.goto_setpoint_publisher.publish(msg)

def main(args=None) -> None:
    print('Starting offboard position control node with joystick inputs...')
    rclpy.init(args=args)
    position_control = PositionControl()
    rclpy.spin(position_control)
    position_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
