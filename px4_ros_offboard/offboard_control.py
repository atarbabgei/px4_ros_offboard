#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Joy
from px4_msgs.msg import OffboardControlMode, VehicleAttitudeSetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
import numpy as np
import math
import time

class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

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
        self.attitude_setpoint_publisher = self.create_publisher(
            VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.joy_subscriber = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)  # Using default QoS profile for Joy

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.joy_axes = [0.0, 0.0, 0.0, 0.0]  # Initialize joystick axes
        self.joy_buttons = [0] * 12  # Initialize joystick buttons, assuming 12 buttons
        self.current_yaw = 0.0  # Initialize the current yaw angle
        self.desired_altitude = 0.0  # Initialize the desired altitude to 0 meters

        # PID controller variables
        self.altitude_error_previous = 0.0
        self.altitude_error_integral = 0.0
        self.altitude_control_time = self.get_clock().now().nanoseconds

        # Control gains
        self.roll_angle_max = 0.3 # in radians (max roll angle)
        self.pitch_angle_max = 0.3  # in radians (max pitch angle)

        self.yaw_gain = 1.0 # in radians (yaw rate gain)
        self.throttle_gain = 0.3 # in meters (max altitude change per second)

        # PID gains
        self.Kp = 0.1
        self.Ki = 0.01
        self.Kd = 0.05

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position
        self.current_yaw = vehicle_local_position.heading

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def joy_callback(self, joy_msg):
        """Callback function for joy topic subscriber."""
        self.joy_axes = joy_msg.axes
        self.joy_buttons = joy_msg.buttons

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")
    
    def kill_vehicle(self):
        """Send a kill command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_FLIGHTTERMINATION, param1=1.0)
        self.get_logger().info('Kill command sent')

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = True
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def euler_to_quaternion(self, roll: float, pitch: float, yaw: float):
        """Convert Euler angles to quaternion."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0.0, 0.0, 0.0, 0.0]
        q[0] = cr * cp * cy + sr * sp * sy
        q[1] = sr * cp * cy - cr * sp * sy
        q[2] = cr * sp * cy + sr * cp * sy
        q[3] = cr * cp * sy - sr * sp * cy
        return q

    def publish_attitude_setpoint(self, roll: float, pitch: float, yaw: float, thrust: float):
        """Publish the attitude setpoint."""
        msg = VehicleAttitudeSetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.roll_body = roll
        msg.pitch_body = pitch
        msg.yaw_body = yaw
        msg.yaw_sp_move_rate = 0.0
        msg.q_d = np.array(self.euler_to_quaternion(roll, pitch, yaw), dtype=np.float32)
        msg.thrust_body = np.array([0.0, 0.0, -thrust], dtype=np.float32)
        msg.reset_integral = False
        msg.fw_control_yaw_wheel = False
        self.attitude_setpoint_publisher.publish(msg)
        #self.get_logger().info(f"Publishing attitude setpoint: roll={roll}, pitch={pitch}, yaw={yaw}, thrust={thrust}")

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
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

            # Kill the vehicle if axis 7 is pressed
        if self.joy_axes[2] == -1.0 and not getattr(self, 'kill_sent', False):
            self.kill_vehicle()
            self.kill_sent = True
        elif self.joy_axes[7] != -1.0:
            self.kill_sent = False  # Reset kill flag when axis 7 is not pressed

        # Arm/disarm based on button 5 state with logic to ensure command is sent only once
        if self.joy_axes[5] == -1 and not getattr(self, 'arm_sent', False):
            if not self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                self.arm()
                self.arm_sent = True
        #elif self.joy_axes[5] != -1 and self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
        #    if not getattr(self, 'disarm_sent', False):
        #        self.disarm()
        #        self.disarm_sent = True
        else:
            self.arm_sent = False
            self.disarm_sent = False  # Reset flags when conditions are not met

        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            # Get joystick inputs
            roll =  - self.joy_axes[3] * self.roll_angle_max  
            pitch = - self.joy_axes[4] * self.pitch_angle_max 
            yaw_rate =  - self.joy_axes[0] * self.yaw_gain 


            # Throttle control for altitude
            throttle = self.joy_axes[1]  

            # Adjust desired altitude if throttle is outside the dead zone of ±0.1
            if abs(throttle) > 0.1:
                self.desired_altitude += throttle * self.throttle_gain  # Adjust scaling factor as needed

            # Ensure desired altitude is non-negative
            self.desired_altitude = max(self.desired_altitude, 0.0)

            # Update the current yaw angle based on yaw rate
            self.current_yaw += yaw_rate 

            # Altitude control
            current_altitude = - self.vehicle_local_position.z
            altitude_error = self.desired_altitude - current_altitude
            thrust = self.calculate_thrust(altitude_error)

            # Send attitude setpoint based on joystick input
            self.publish_attitude_setpoint(roll, pitch, self.current_yaw, thrust)

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

    def calculate_thrust(self, altitude_error):
        """PID controller to calculate thrust based on altitude error."""
        current_time = self.get_clock().now().nanoseconds
        dt = (current_time - self.altitude_control_time) / 1e9  # Convert to seconds
        self.altitude_control_time = current_time

        # Proportional term
        P = self.Kp * altitude_error

        # Integral term
        self.altitude_error_integral += altitude_error * dt
        I = self.Ki * self.altitude_error_integral

        # Derivative term
        D = self.Kd * (altitude_error - self.altitude_error_previous) / dt
        self.altitude_error_previous = altitude_error

        # Calculate total thrust
        thrust = P + I + D
        thrust = np.clip(thrust, 0.0, 1.0)  # Ensure thrust is within valid range [0, 1]
        #self.get_logger().info(f"Calculated thrust: {thrust}, altitude error: {altitude_error}")
        return thrust

def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
