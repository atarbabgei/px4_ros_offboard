#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleAttitude, TrajectorySetpoint, VehicleStatus
from geometry_msgs.msg import Twist
import numpy as np
from math import pi, atan2

from px4_ros_offboard.joy_inputs import JoystickInputs  # Ensure this is correctly imported


class VelocityControl(Node):
    """Node for controlling a vehicle in velocity control mode using joystick inputs."""

    def __init__(self) -> None:
        super().__init__('velocity_control')

        # Declare the use_world_frame parameter (default is True)
        self.declare_parameter('use_world_frame', True)
        # Fetch the parameter value
        self.use_world_frame = self.get_parameter('use_world_frame').get_parameter_value().bool_value
        self.get_logger().info(f"Velocity frame set to {'World frame (NED)' if self.use_world_frame else 'Vehicle-relative frame (FLU)'}")

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

        # Create subscribers
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.attitude_subscriber = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', self.attitude_callback, qos_profile)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_status = VehicleStatus()
        self.trueYaw = 0.0  # Initialize the current yaw angle

        # Control parameters
        self.max_velocity = 2.0  # Maximum velocity in m/s for x, y
        self.max_velocity_vertical = 2.0  # Maximum velocity in m/s for z (vertical speed)
        self.yaw_gain = pi / 4  # Maximum yaw rate in rad/s

        # Joystick velocity and yaw inputs
        self.velocity = Twist()  # Use Twist message for x, y, z and yaw values

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.02, self.timer_callback)  # Increased frequency for smoother control

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def attitude_callback(self, msg):
        """Callback to update the yaw (trueYaw) from the VehicleAttitude quaternion."""
        orientation_q = msg.q

        # Extract yaw from quaternion
        self.trueYaw = -(np.arctan2(2.0*(orientation_q[3]*orientation_q[0] + orientation_q[1]*orientation_q[2]), 
                                  1.0 - 2.0*(orientation_q[0]*orientation_q[0] + orientation_q[1]*orientation_q[1])))

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
        msg.position = False
        msg.velocity = True  # Enable velocity control
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
            # Read joystick inputs and apply NED to FLU transformation
            pitch_input = self.joystick_inputs.get_pitch()  # Forward/backward
            roll_input = self.joystick_inputs.get_roll()    # Left/right
            throttle_input = self.joystick_inputs.get_throttle()  # Up/down
            yaw_input = self.joystick_inputs.get_yaw()      # Yaw rate

            # Apply joystick inputs and apply the FLU -> NED conversion logic
            self.velocity.linear.x = -pitch_input * self.max_velocity     
            self.velocity.linear.y = roll_input * self.max_velocity     
            self.velocity.linear.z = -throttle_input * self.max_velocity_vertical  
            self.velocity.angular.z = - yaw_input * self.yaw_gain  # Yaw rate control

            if self.use_world_frame:
                # Publish the velocity setpoint in the world frame (NED)
                velocity_x = self.velocity.linear.y
                velocity_y = - self.velocity.linear.x
                velocity_z = self.velocity.linear.z
                self.publish_trajectory_setpoint(velocity_x, velocity_y, velocity_z, self.velocity.angular.z)
           
            else:
                # Publish the velocity setpoint in the vehicle-relative frame (FLU)
                cos_yaw = np.cos(self.trueYaw)
                sin_yaw = np.sin(self.trueYaw)
                velocity_x = self.velocity.linear.x * cos_yaw - self.velocity.linear.y * sin_yaw
                velocity_y = self.velocity.linear.x * sin_yaw + self.velocity.linear.y * cos_yaw
                velocity_z = self.velocity.linear.z
                self.publish_trajectory_setpoint(velocity_x, velocity_y, velocity_z, self.velocity.angular.z)
 
        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

    def publish_trajectory_setpoint(self, velocity_x: float, velocity_y: float, velocity_z: float, yaw_rate: float):
        """Publish the velocity setpoint as a TrajectorySetpoint."""
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [float('nan'), float('nan'), float('nan')]
        msg.velocity = [velocity_x, velocity_y, velocity_z]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.jerk = [float('nan'), float('nan'), float('nan')]
        msg.yaw = float('nan')
        msg.yawspeed = yaw_rate
        self.trajectory_setpoint_publisher.publish(msg)


def main(args=None) -> None:
    print('Starting offboard velocity control node with joystick inputs...')
    rclpy.init(args=args)
    velocity_control = VelocityControl()
    rclpy.spin(velocity_control)
    velocity_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
