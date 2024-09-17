#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, GotoSetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from px4_ros_offboard.joy_inputs import JoystickInputs  # Ensure this is correctly imported
import pandas as pd
import numpy as np

class TrajectoryPlanner(Node):
    """Node for controlling a vehicle in position control mode using joystick inputs."""

    def __init__(self) -> None:
        super().__init__('position_control')

        # Declare the use_world_frame parameter (default is True)
        self.declare_parameter('use_world_frame', False)
        self.use_world_frame = self.get_parameter('use_world_frame').get_parameter_value().bool_value
        self.get_logger().info(f"Position control frame set to {'World frame (NED)' if self.use_world_frame else 'Vehicle-relative frame (FLU)'}")

        # Initialize joystick inputs
        self.joystick_inputs = JoystickInputs(self)

        # QoS and Publishers
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        
        # Subscribers
        self.vehicle_status_subscriber = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.vehicle_local_position_subscriber = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_status = VehicleStatus()
        self.current_yaw = 0.0
        self.current_position = [0.0, 0.0, 0.0]
        self.current_state = "IDLE"
        self.last_state = self.current_state

        self.height_offset = 0.9  # Offset to add to the z position

        self.start_x = 1.0
        self.start_y = 0.0
        self.start_z = 3.0 - self.height_offset   # Altitude to takeoff to 3.0

        # Load trajectory data from CSV
        data = pd.read_csv('/home/atar/rolling_drone_ws/src/px4_ros_offboard/config/trajectory_reduced.csv')
        x = data['Drone_X']
        y = data['Drone_Y']
        z = data['Drone_Z'] - self.height_offset  # Adjust z with height offset

        # Convert to numpy array and then to list of tuples
        data = np.array([x, y, z])
        data_transposed = data.T
        self.trajectory_points = [tuple(point) for point in data_transposed]

        self.current_trajectory_point = 0
        self.trajectory_wait_counter = 0

        # Dynamically calculate the number of ticks to wait at each point
        self.timer_period = 0.01  # Timer callback interval (in seconds)
        self.total_execution_time = 2.0  # Total time to complete the trajectory (in seconds)
        self.num_trajectory_points = len(self.trajectory_points)  # Number of points in the trajectory

        # Calculate total ticks for full trajectory and max wait ticks per point
        self.total_ticks = int(self.total_execution_time / self.timer_period)  # Total ticks for the entire trajectory
        self.max_wait_ticks = int(self.total_ticks / self.num_trajectory_points)  # Number of ticks per point
    
        self.get_logger().info(f"max_wait_ticks dynamically set to {self.max_wait_ticks} for {self.num_trajectory_points} trajectory points")

        # Create a timer to publish control commands
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

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

    def land(self):
        """Send a land command to the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND, param1=0.0)
        self.get_logger().info('Land command sent')

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
        msg.velocity = True
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
            self.current_state = "IDLE"
            self.get_logger().info("State changed to IDLE")
        elif not self.joystick_inputs.is_kill_pressed():
            self.kill_sent = False  # Reset kill flag

        # Run the state machine
        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            if self.current_state == "IDLE":
                if self.joystick_inputs.is_arm_pressed() and not getattr(self, 'arm_sent', False):
                    if not self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                        self.current_state = "ARMED"
                        self.arm()
                        self.get_logger().info("State changed to ARMED")
                        self.arm_sent = True
                elif not self.joystick_inputs.is_arm_pressed():
                    self.arm_sent = False  # Reset arm flag when the arm button is not pressed

            elif self.current_state == "ARMED":
                if self.joystick_inputs.is_takeoff_pressed() and not getattr(self, 'takeoff_sent', False):
                    if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                        self.current_state = "TAKEOFF"
                        self.get_logger().info("State changed to TAKEOFF")
                        self.takeoff_sent = True
                elif not self.joystick_inputs.is_takeoff_pressed():
                    self.takeoff_sent = False

            elif self.current_state == "TAKEOFF":
                # Set velocity to NaN during takeoff
                self.publish_trajectory_setpoint(self.start_x, self.start_y, self.start_z, self.current_yaw,
                                                 velocity_x=float('nan'), velocity_y=float('nan'), velocity_z=float('nan'))
                if self.joystick_inputs.is_start_trajectory_pressed() and not getattr(self, 'start_trajectory_sent', False):
                    self.current_state = "TRAJECTORY"
                    self.get_logger().info("State changed to TRAJECTORY")
                    self.current_trajectory_point = 0  # Start at the first point in the trajectory
                    self.trajectory_wait_counter = 0  # Reset counter for waiting between points
                    self.start_trajectory_sent = True
                elif not self.joystick_inputs.is_start_trajectory_pressed():
                    self.start_trajectory_sent = False

            elif self.current_state == "TRAJECTORY":
                self.execute_trajectory()

            elif self.current_state == "HOLD":
                # Hold position at the last trajectory point
                if self.trajectory_points:
                    last_point = self.trajectory_points[-1]
                    self.publish_trajectory_setpoint(last_point[0], last_point[1], last_point[2], self.current_yaw)

                if self.joystick_inputs.is_stop_trajectory_pressed() and not getattr(self, 'stop_trajectory_sent', False):
                    self.current_state = "IDLE"
                    self.get_logger().info("LANDING and State changed to IDLE")
                    self.land()
                    self.stop_trajectory_sent = True
                elif not self.joystick_inputs.is_stop_trajectory_pressed():
                    self.stop_trajectory_sent = False

        # Increment offboard counter for enabling offboard mode
        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

        # Log state transition
        if self.last_state != self.current_state:
            self.last_state = self.current_state
            self.get_logger().info(f"State changed to {self.current_state}")

    def execute_trajectory(self):
        """Execute the trajectory with dynamic interpolation for both position and velocity."""
        if self.trajectory_wait_counter == 0 and self.current_trajectory_point < len(self.trajectory_points):
            # Send the first point of the trajectory
            self.trajectory_wait_counter += 1  # Start counting
        elif self.trajectory_wait_counter < self.max_wait_ticks:
            # Update the interpolated position and velocity
            (target_position, target_velocity) = self.update_target_position()
            self.publish_trajectory_setpoint(target_position[0], target_position[1], target_position[2], self.current_yaw, 
                                             target_velocity[0], target_velocity[1], target_velocity[2])
            self.get_logger().info(f"Target position: {target_position}, Target velocity: {target_velocity}")
            self.trajectory_wait_counter += 1
        else:
            # Move to the next point in the trajectory
            self.current_trajectory_point += 1
            self.trajectory_wait_counter = 0  # Reset wait counter

            # If we've completed all points, hold the position
            if self.current_trajectory_point >= len(self.trajectory_points):
                self.current_state = "HOLD"
                self.get_logger().info("Trajectory complete, switching to HOLD")

    def publish_trajectory_setpoint(self, position_x: float, position_y: float, position_z: float, yaw: float, 
                                    velocity_x: float = float('nan'), velocity_y: float = float('nan'), velocity_z: float = float('nan')):
        """Publish the position and velocity setpoint as a TrajectorySetpoint."""
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        # Set the position
        msg.position = [float(position_x), float(position_y), - float(position_z)]  # NED frame (z is negative)

        # Set the velocity
        msg.velocity = [velocity_x, velocity_y, velocity_z]  # Velocity in X, Y, Z

        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.jerk = [float('nan'), float('nan'), float('nan')]
        msg.yaw = yaw
        msg.yawspeed = float('nan')

        self.trajectory_setpoint_publisher.publish(msg)

    def update_target_position(self):
        """Update the target position and velocity for smooth transitions."""

        total_time_steps = self.max_wait_ticks  # Use the max_wait_ticks to define total time for each point
        current_time_step = self.trajectory_wait_counter

        # Calculate fraction of time elapsed
        if total_time_steps == 0 or current_time_step >= total_time_steps:
            fraction = 1  # Avoid division by zero or ensure full progress when time is up
        else:
            fraction = current_time_step / total_time_steps

        # Interpolating position based on the fraction of total time elapsed
        prev_point = self.trajectory_points[self.current_trajectory_point - 1]
        current_point = self.trajectory_points[self.current_trajectory_point]

        target_x = prev_point[0] + fraction * (current_point[0] - prev_point[0])
        target_y = prev_point[1] + fraction * (current_point[1] - prev_point[1])
        target_z = prev_point[2] + fraction * (current_point[2] - prev_point[2])

        # Calculate dynamic velocity during the first half of the trajectory
        if fraction < 0.5:
            time_remaining = (1 - fraction) * (total_time_steps * self.timer_period)
            if time_remaining > 0:
                velocity_x = (current_point[0] - self.current_position[0]) / time_remaining
                velocity_y = (current_point[1] - self.current_position[1]) / time_remaining
                velocity_z = (current_point[2] - self.current_position[2]) / time_remaining
            else:
                velocity_x, velocity_y, velocity_z = float('nan'), float('nan'), float('nan')
        else:
            # As we approach the end, let the velocity slow down naturally
            velocity_x, velocity_y, velocity_z = float('nan'), float('nan'), float('nan')

        return (target_x, target_y, target_z), (velocity_x, velocity_y, velocity_z)


def main(args=None) -> None:
    print('Starting offboard position control node with joystick inputs...')
    rclpy.init(args=args)
    trajectory_planner = TrajectoryPlanner()
    rclpy.spin(trajectory_planner)
    trajectory_planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
