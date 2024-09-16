#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import OffboardControlMode, GotoSetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
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
        self.goto_setpoint_publisher = self.create_publisher(GotoSetpoint, '/fmu/in/goto_setpoint', qos_profile)

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

        self.height_offset = 0.5  # Offset to add to the z position
        
        self.start_x = 2.0
        self.start_y = 0.0
        self.start_z = 3.0 - self.height_offset   # Altitude to takeoff to 3.0



        data = pd.read_csv('/home/atar/rolling_drone_ws/src/px4_ros_offboard/config/trajectory_reduced_smallest.csv')

        x = data['Drone_X']
        y = data['Drone_Y']
        z = data['Drone_Z'] - self.height_offset 
        
        # Convert to numpy array
        data = np.array([x, y, z])
        
        # Transpose the array to get the structure (x, y, z) for each point
        data_transposed = data.T
        
        # Convert to list of tuples
        self.trajectory_points = [tuple(point) for point in data_transposed]
        

        self.current_trajectory_point = 0
        self.trajectory_wait_counter = 0


        # Create a timer to publish control commands
        self.timer = self.create_timer(0.02, self.timer_callback)

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
                         # Takeoff
                        self.takeoff_sent = True
                elif not self.joystick_inputs.is_takeoff_pressed():
                    self.takeoff_sent = False

            elif self.current_state == "TAKEOFF":
                self.publish_goto_setpoint(self.start_x, self.start_y, self.start_z, self.current_yaw)
                self.get_logger().info("Taking off to 2 meters above the ground")
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
                self.get_logger().info("Holding at this position until stop trajectory command is received")
                self.publish_goto_setpoint(self.current_position[0], self.current_position[1], - self.current_position[2], self.current_yaw)
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
        # If we've completed all points, hold the position
        if self.current_trajectory_point >= len(self.trajectory_points):
            self.current_state = "HOLD"
            self.get_logger().info("Trajectory complete, switching to HOLD")
        
        else:
            # Send the next point
            point = self.trajectory_points[self.current_trajectory_point]
            self.publish_goto_setpoint(point[0], point[1], point[2], self.current_yaw)
            self.get_logger().info(f"Moving to point {point}")
            self.current_trajectory_point += 1

    def publish_goto_setpoint(self, position_x: float, position_y: float, position_z: float, heading: float):
        """Publish the position setpoint as a GotoSetpoint."""
        msg = GotoSetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [float(position_x), float(position_y), - float(position_z)]
        msg.flag_control_heading = False
        msg.heading = heading
        msg.flag_set_max_horizontal_speed = True
        msg.max_horizontal_speed = 20.0
        msg.flag_set_max_vertical_speed = True
        msg.max_vertical_speed = 20.0
        msg.flag_set_max_heading_rate = True
        msg.max_heading_rate = 20.0
        self.goto_setpoint_publisher.publish(msg)


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
