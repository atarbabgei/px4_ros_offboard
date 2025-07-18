#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from px4_ros_offboard.joy_inputs import JoystickInputs  # Ensure this is correctly imported
from geometry_msgs.msg import PoseStamped
import numpy as np

class TrajectoryPlanner(Node):
    """Node for controlling a vehicle in position control mode using joystick inputs."""

    def __init__(self) -> None:
        super().__init__('position_control')

        # Declare the use_world_frame parameter (default is False)
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
        self.mocap_subscriber = self.create_subscription(PoseStamped, '/mocap/Kh4/Kh4', self.mocap_callback, qos_profile)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_status = VehicleStatus()
        self.current_yaw = 0.0
        self.current_position = [0.0, 0.0, 0.0]
        self.current_state = "IDLE"
        self.last_state = self.current_state

        self.takeoff_position = [0.0, 0.0, 0.0]  # Will be set when takeoff starts
        self.takeoff_altitude = 1.5  # 2 meters above takeoff position
        self.trajectory_position = [0.0, 0.0, 0.0]  # Will be set when trajectory starts
        self.hold_position = [0.0, 0.0, 0.0]  # Current hold position, updated after trajectory completion
        
        # Mocap tracking variables
        self.mocap_position = [0.0, 0.0, 0.0]  # Current mocap position
        self.mocap_available = False  # Flag to check if mocap data is available
        self.last_mocap_time = 0.0  # Time of last mocap message

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.01, self.timer_callback)  # 10ms timer
        
        # Debug: Log initial setup
        self.get_logger().info("TrajectoryPlanner initialized")
        self.get_logger().info("=== CONTROL KEYS ===")
        self.get_logger().info("  1 - ARM")
        self.get_logger().info("  2 - KILL")
        self.get_logger().info("  3 - TAKEOFF")
        self.get_logger().info("  4 - START TRAJECTORY TRACKING")
        self.get_logger().info("  5 - STOP TRAJECTORY / LAND")
        self.get_logger().info("===================")
        self.get_logger().info("Waiting for vehicle status messages...")
        self.get_logger().info("Publishers created for:")
        self.get_logger().info("  - /fmu/in/offboard_control_mode")
        self.get_logger().info("  - /fmu/in/vehicle_command") 
        self.get_logger().info("  - /fmu/in/trajectory_setpoint")
        self.get_logger().info("Subscribers created for:")
        self.get_logger().info("  - /fmu/out/vehicle_status")
        self.get_logger().info("  - /fmu/out/vehicle_local_position")
        self.get_logger().info("  - /mocap/Kh4/Kh4")

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status
        # Debug logging for vehicle status changes
        if not hasattr(self, 'last_nav_state') or self.last_nav_state != vehicle_status.nav_state:
            self.get_logger().info(f"Nav state changed to: {vehicle_status.nav_state}")
            self.last_nav_state = vehicle_status.nav_state
        
        if not hasattr(self, 'last_arming_state') or self.last_arming_state != vehicle_status.arming_state:
            self.get_logger().info(f"Arming state changed to: {vehicle_status.arming_state}")
            self.last_arming_state = vehicle_status.arming_state

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.current_position = [
            vehicle_local_position.x,
            vehicle_local_position.y,
            vehicle_local_position.z
        ]
        self.current_yaw = vehicle_local_position.heading
        
        # Debug logging for position updates (only occasionally)
        if not hasattr(self, 'position_counter'):
            self.position_counter = 0
        self.position_counter += 1
        
        if self.position_counter % 100 == 0:  # Every 100 position updates
            self.get_logger().info(f"Position update: X={self.current_position[0]:.3f}, Y={self.current_position[1]:.3f}, Z={self.current_position[2]:.3f}")

    def mocap_callback(self, pose_stamped):
        """Callback function for mocap topic subscriber."""
        self.mocap_position = [
            pose_stamped.pose.position.x,
            pose_stamped.pose.position.y,
            pose_stamped.pose.position.z
        ]
        self.mocap_available = True
        self.last_mocap_time = self.get_clock().now().nanoseconds / 1e9  # Convert to seconds

    def is_mocap_data_valid(self):
        """Check if mocap data is available and not too old."""
        if not self.mocap_available:
            return False
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        time_since_last_mocap = current_time - self.last_mocap_time
        
        # Consider mocap data valid if it's less than 1 second old
        return time_since_last_mocap < 1.0

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
        
        # Debug logging for offboard heartbeat (only first few times)
        if self.offboard_setpoint_counter <= 5:
            self.get_logger().info(f"Publishing offboard heartbeat #{self.offboard_setpoint_counter}")

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
        
        # Debug logging for vehicle commands
        command_names = {
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM: "ARM/DISARM",
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE: "SET_MODE",
            VehicleCommand.VEHICLE_CMD_NAV_LAND: "LAND",
            VehicleCommand.VEHICLE_CMD_DO_FLIGHTTERMINATION: "KILL"
        }
        command_name = command_names.get(command, f"UNKNOWN({command})")
        self.get_logger().info(f"Sending vehicle command: {command_name} with params: {params}")
        
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        # Debug logging for offboard mode transition
        if self.offboard_setpoint_counter <= 12:
            self.get_logger().info(f"Offboard counter: {self.offboard_setpoint_counter}, Current nav_state: {self.vehicle_status.nav_state}")

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.get_logger().info("Attempting to engage offboard mode...")

        # More detailed vehicle status logging
        if self.offboard_setpoint_counter % 50 == 0:  # Every 0.5 seconds
            self.get_logger().info(f"Vehicle Status - Nav State: {self.vehicle_status.nav_state}, Arming State: {self.vehicle_status.arming_state}")
            self.get_logger().info(f"Expected Offboard Nav State: {VehicleStatus.NAVIGATION_STATE_OFFBOARD}")
            self.get_logger().info(f"Expected Armed State: {VehicleStatus.ARMING_STATE_ARMED}")
            self.get_logger().info(f"Current State Machine State: {self.current_state}")
            self.get_logger().info(f"Is offboard mode? {self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD}")
            
            # Debug joystick inputs
            self.get_logger().info(f"Joystick - Arm: {self.joystick_inputs.is_arm_pressed()}, Takeoff: {self.joystick_inputs.is_takeoff_pressed()}")
            self.get_logger().info(f"Joystick - Start Trajectory: {self.joystick_inputs.is_start_trajectory_pressed()}, Stop: {self.joystick_inputs.is_stop_trajectory_pressed()}")
            self.get_logger().info(f"Joystick - Kill: {self.joystick_inputs.is_kill_pressed()}")

        # Handle arming and kill commands using joystick buttons
        if self.joystick_inputs.is_kill_pressed() and not getattr(self, 'kill_sent', False):
            self.kill_vehicle()
            self.kill_sent = True
            self.current_state = "IDLE"
            self.get_logger().info("State changed to IDLE")
        elif not self.joystick_inputs.is_kill_pressed():
            self.kill_sent = False  # Reset kill flag

        # Direct trajectory control - bypass all state checks
        if self.joystick_inputs.is_start_trajectory_pressed() and not getattr(self, 'direct_trajectory_sent', False):
            self.current_state = "TRAJECTORY"
            self.get_logger().info("DIRECT TRAJECTORY MODE ACTIVATED!")
            self.direct_trajectory_sent = True
        elif not self.joystick_inputs.is_start_trajectory_pressed():
            self.direct_trajectory_sent = False

        # Run the state machine
        # Debug: Always show if we're checking the state machine
        if self.offboard_setpoint_counter % 100 == 0:
            self.get_logger().info(f"Checking state machine: nav_state={self.vehicle_status.nav_state}, OFFBOARD={VehicleStatus.NAVIGATION_STATE_OFFBOARD}")
            self.get_logger().info(f"Is in offboard mode? {self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD}")
            self.get_logger().info(f"=== CURRENT STATE: {self.current_state} ===")
        
        # Handle TRAJECTORY state regardless of offboard mode
        if self.current_state == "TRAJECTORY":
            # Debug logging for trajectory state
            if self.offboard_setpoint_counter % 50 == 0:  # Every 0.5 seconds
                self.get_logger().info(f"TRAJECTORY State - Mocap valid: {self.is_mocap_data_valid()}")
                if self.mocap_available:
                    self.get_logger().info(f"TRAJECTORY State - Mocap position: {self.mocap_position}")
            
            # Track mocap position if available, otherwise hold current position
            if self.is_mocap_data_valid():
                # Use mocap X,Y position but keep takeoff Z position
                target_x = self.mocap_position[0]
                target_y = - self.mocap_position[1]
                target_z = self.takeoff_position[2] - self.takeoff_altitude if hasattr(self, 'takeoff_position') else -1.5  # Default altitude
                
                # Update trajectory position for tracking
                self.trajectory_position = [target_x, target_y, target_z]
                
                self.get_logger().info(f"Tracking mocap position: X={target_x:.3f}, Y={target_y:.3f}, Z={target_z:.3f}")
            else:
                # No valid mocap data, hold current position or default
                target_x = self.hold_position[0] if hasattr(self, 'hold_position') else 0.0
                target_y = self.hold_position[1] if hasattr(self, 'hold_position') else 0.0
                target_z = self.hold_position[2] if hasattr(self, 'hold_position') else -1.5
                
                if not self.mocap_available:
                    self.get_logger().warn("No mocap data available, holding current position")
                else:
                    self.get_logger().warn("Mocap data too old, holding current position")

            # Publish trajectory setpoint
            self.publish_trajectory_setpoint(target_x, target_y, target_z, self.current_yaw)

            # Stop trajectory tracking when button is pressed again
            if self.joystick_inputs.is_start_trajectory_pressed() and not getattr(self, 'trajectory_stop_sent', False):
                self.current_state = "HOLD"
                # Update hold position to current trajectory position
                if self.is_mocap_data_valid():
                    self.hold_position = self.trajectory_position.copy()
                    self.get_logger().info(f"Trajectory tracking stopped, holding at mocap position: {self.hold_position}")
                else:
                    # Keep current hold position if no valid mocap data
                    self.get_logger().info("Trajectory tracking stopped, holding at current position")
                self.trajectory_stop_sent = True
            elif not self.joystick_inputs.is_start_trajectory_pressed():
                self.trajectory_stop_sent = False

            # Land when stop/land button is pressed
            if self.joystick_inputs.is_stop_trajectory_pressed() and not getattr(self, 'land_sent', False):
                self.current_state = "LANDING"
                self.get_logger().info("Starting landing sequence")
                self.land()
                self.land_sent = True
            elif not self.joystick_inputs.is_stop_trajectory_pressed():
                self.land_sent = False
            
        elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            # Debug logging every 100 cycles (1 second) - show current state always
            if self.offboard_setpoint_counter % 100 == 0:
                self.get_logger().info(f"=== CURRENT STATE: {self.current_state} ===")
                self.get_logger().info(f"Vehicle armed: {self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED}")
            
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
                # Reset takeoff position flag when in ARMED state
                if hasattr(self, 'takeoff_position_set'):
                    delattr(self, 'takeoff_position_set')
                    
                if self.joystick_inputs.is_takeoff_pressed() and not getattr(self, 'takeoff_sent', False):
                    if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                        self.current_state = "TAKEOFF"
                        self.get_logger().info("State changed to TAKEOFF")
                        self.takeoff_sent = True
                elif not self.joystick_inputs.is_takeoff_pressed():
                    self.takeoff_sent = False

            elif self.current_state == "TAKEOFF":
                # Record takeoff position when entering TAKEOFF state
                if not hasattr(self, 'takeoff_position_set'):
                    self.takeoff_position = self.current_position.copy()
                    self.takeoff_position_set = True
                    self.get_logger().info(f"Takeoff position set to: {self.takeoff_position}")
                    self.get_logger().info(f"Target takeoff altitude: {self.takeoff_altitude}m above ground")
                
                # Move to takeoff altitude (1.5m above current position)
                target_x = self.takeoff_position[0]
                target_y = self.takeoff_position[1]
                target_z = self.takeoff_position[2] - self.takeoff_altitude  # 1.5m up in NED frame
                
                self.publish_trajectory_setpoint(target_x, target_y, target_z, self.current_yaw)
                
                # Calculate altitude error for state transition
                current_altitude = abs(self.current_position[2])
                target_altitude = abs(target_z)
                altitude_error = abs(current_altitude - target_altitude)
                
                # Automatically transition to HOLD after takeoff
                # Check if we're close to the target altitude
                if altitude_error < 0.3:  # Within 30cm of target
                    self.current_state = "HOLD"
                    # Initialize hold position to takeoff altitude position
                    self.hold_position = [
                        self.takeoff_position[0],
                        self.takeoff_position[1],
                        self.takeoff_position[2] - self.takeoff_altitude
                    ]
                    self.get_logger().info("Takeoff complete, switching to HOLD")

            elif self.current_state == "HOLD":
                # Hold position at current hold position
                self.publish_trajectory_setpoint(
                    self.hold_position[0], 
                    self.hold_position[1], 
                    self.hold_position[2], 
                    self.current_yaw
                )

                # Debug trajectory button checking
                if self.offboard_setpoint_counter % 100 == 0:  # Every second
                    self.get_logger().info(f"HOLD State - Trajectory button pressed: {self.joystick_inputs.is_start_trajectory_pressed()}")
                    self.get_logger().info(f"HOLD State - Trajectory start sent flag: {getattr(self, 'trajectory_start_sent', False)}")

                # Start trajectory when start trajectory button is pressed
                if self.joystick_inputs.is_start_trajectory_pressed() and not getattr(self, 'trajectory_start_sent', False):
                    self.current_state = "TRAJECTORY"
                    self.get_logger().info("Starting trajectory - moving forward")
                    self.trajectory_start_sent = True
                elif not self.joystick_inputs.is_start_trajectory_pressed():
                    self.trajectory_start_sent = False

                # Land when stop/land button is pressed
                if self.joystick_inputs.is_stop_trajectory_pressed() and not getattr(self, 'land_sent', False):
                    self.current_state = "LANDING"
                    self.get_logger().info("Starting landing sequence")
                    self.land()
                    self.land_sent = True
                elif not self.joystick_inputs.is_stop_trajectory_pressed():
                    self.land_sent = False

            elif self.current_state == "LANDING":
                # Continue holding position during landing sequence at current hold position
                self.publish_trajectory_setpoint(
                    self.hold_position[0], 
                    self.hold_position[1], 
                    self.hold_position[2], 
                    self.current_yaw
                )
                
                # Check if landed (very close to ground)
                if abs(self.current_position[2]) < 0.2:  # Within 20cm of ground
                    self.current_state = "IDLE"
                    self.get_logger().info("Landing complete, switching to IDLE")
        else:
            # Debug logging when not in offboard mode
            if self.offboard_setpoint_counter % 100 == 0:
                self.get_logger().warn(f"Vehicle not in offboard mode. Current nav_state: {self.vehicle_status.nav_state}")
                self.get_logger().info("Make sure to switch to offboard mode before using this controller")

        # Increment offboard counter for enabling offboard mode
        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

        # Log state transition
        if self.last_state != self.current_state:
            self.last_state = self.current_state
            self.get_logger().info(f"State changed to {self.current_state}")

    def publish_trajectory_setpoint(self, position_x: float, position_y: float, position_z: float, yaw: float):
        """Publish the position setpoint as a TrajectorySetpoint."""
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        # Set the position (NED frame)
        msg.position = [float(position_x), float(position_y), float(position_z)]

        # Set velocity, acceleration, and jerk to NaN for position control
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.jerk = [float('nan'), float('nan'), float('nan')]
        msg.yaw = yaw
        msg.yawspeed = float('nan')

        self.trajectory_setpoint_publisher.publish(msg)


def main(args=None) -> None:
    print('Starting simple takeoff and hold controller...')
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
