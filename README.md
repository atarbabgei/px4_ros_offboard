
# `px4_ros_offboard`

This package provides ROS2 nodes for controlling drones via PX4's Offboard control. It supports two modes: **velocity control** and **altitude hold control**. Users can operate the drone using either a **keyboard** or a **joystick**. For keyboard input, the `keyboard_joy` package is required, while for joystick input, the standard ROS `joy` node is used.

## Dependencies

Ensure the following dependencies are installed before running the launch files:

- **keyboard_joy** (for keyboard-based control)
  - Repository: [keyboard_joy](https://github.com/atarbabgei/keyboard_joy)
  
- **joy** (for joystick-based control)
  - Install the `joy` package for joystick input:

    ```bash
    sudo apt-get install ros-<your-ros-distro>-joy
    ```

## Installation

Follow these steps to install the required packages:

```bash
# Create a new workspace
mkdir -p your_workspace/src
# Navigate to the new src folder
cd ~/your_workspace/src
# Clone the keyboard_joy repository
git clone https://github.com/atarbabgei/keyboard_joy.git
# Clone the px4_ros_offboard repository
git clone https://github.com/atarbabgei/px4_ros_offboard.git
# Build the workspace
cd ~/your_workspace
colcon build
```

## Launch Files and Usage

### 1. **`velocity_control.launch.py`**

This launch file starts the `velocity_control` node.

#### Example Commands

- To launch with **keyboard** input:

  ```bash
  ros2 launch px4_ros_offboard velocity_control.launch.py input:=keyboard use_world_frame:=false
  ```

- To launch with **joystick** input:

  ```bash
  ros2 launch px4_ros_offboard velocity_control.launch.py input:=joystick use_world_frame:=true
  ```

### 2. **`manual_control.launch.py`**

This launch file starts the `manual_control` node, which mimics the manual control mode in PX4.

#### Example Commands

- To launch with **keyboard** input:

  ```bash
  ros2 launch px4_ros_offboard manual_control.launch.py input:=keyboard
  ```

### 3. **`altitude_hold_control.launch.py`**

This launch file starts the `altitude_hold_control` node, which implements a PID loop for altitude hold.

#### Example Commands

- To launch with **keyboard** input:

  ```bash
  ros2 launch px4_ros_offboard altitude_hold_control.launch.py input:=keyboard
  ```

### 4. **`position_control.launch.py`**

This launch file starts the `position_control` node, which updates the position setpoint in PX4's world frame (NED) based on `/joy` input.

#### Example Commands

- To launch with **keyboard** input:

  ```bash
  ros2 launch px4_ros_offboard position_control.launch.py input:=keyboard
  ```

## Parameters

- **`input`**: Selects the input method, either `'keyboard'` or `'joystick'`. Default is `'keyboard'`.
  
- **`use_world_frame`** (used only in `velocity_control.launch.py`): Determines whether to use the world frame (NED) or the vehicle-relative frame (FLU). The default is `'false'` (FLU frame).

## Configuration Files

The package includes configuration files for both keyboard and joystick input located in the `config` directory:

- `joy_config.yaml` (for joystick input)
- `keyboard_joy_config.yaml` (for keyboard input)

These configuration files are automatically selected based on the `input` argument.