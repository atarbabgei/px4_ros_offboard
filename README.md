
# px4_ros_offboard

This package contains ROS2 nodes for controlling drones using PX4 Offboard control. It supports two modes of control: **velocity control** and **altitude hold control**. Users can control the drone using either a **keyboard** or a **joystick**. If using a keyboard, the `keyboard_joy` package is required. For joystick input, the standard ROS `joy` node is used.

## Dependencies

Before running the launch files, ensure the following dependencies are installed:

- **keyboard_joy** (for keyboard-based control)
  - Repository: [keyboard_joy](https://github.com/atarbabgei/keyboard_joy)
  
- **joy** (for joystick-based control)
  - Install the `joy` package for joystick input:

    ```bash
    sudo apt-get install ros-<your-ros-distro>-joy
    ```

## Installation

To install the required packages, clone the repositories into your ROS2 workspace and build them:

```bash
# Make a new workspace
mkdir -p your_workspace/src
# Go to the new created src folder
cd ~/your_workspace/src
# Clone the keyboard joy repository
git clone https://github.com/atarbabgei/keyboard_joy.git
# Clone the px4_ros_offboard repository
git clone https://github.com/atarbabgei/px4_ros_offboard.git
# Build the workspace
cd ~/your_workspace
colcon build
```

## Launch Files and Usage

### 1. **velocity_control.launch.py**

This launch file runs the `velocity_control` node. 

#### Example Commands

- To launch with a **keyboard**:

  ```bash
  ros2 launch px4_ros_offboard velocity_control.launch.py input:=keyboard use_world_frame:=false
  ```

- To launch with a **joystick**:

  ```bash
  ros2 launch px4_ros_offboard velocity_control.launch.py input:=joystick use_world_frame:=false
  ```
#### Parameters:
- **input**: Selects the input method. Accepts `'keyboard'` or `'joystick'`. The default is `'keyboard'`.
  
- **use_world_frame** (applicable only for `velocity_control.launch.py`): Defines whether to use the world frame (NED) or the vehicle-relative frame (FLU). The default is `'false'` (FLU frame).


### 2. **manual_control.launch.py**

This launch file runs the `manual_control` node. Similar to the velocity control node, it can use either a **keyboard** or a **joystick** for control.

#### Example Commands

- To launch with a **keyboard**:

  ```bash
  ros2 launch px4_ros_offboard manual_control.launch.py input:=keyboard
  ```

- To launch with a **joystick**:

  ```bash
  ros2 launch px4_ros_offboard manual_control.launch.py input:=joystick
  ```

#### Parameters:
- **input**: Selects the input method. Accepts `'keyboard'` or `'joystick'`. The default is `'keyboard'`.

### 3. **altitude_hold_control.launch.py**

This launch file runs the `altitude_hold_control` node. Similar to the velocity control node, it can use either a **keyboard** or a **joystick** for control.

#### Example Commands

- To launch with a **keyboard**:

  ```bash
  ros2 launch px4_ros_offboard altitude_hold_control.launch.py input:=keyboard
  ```

- To launch with a **joystick**:

  ```bash
  ros2 launch px4_ros_offboard altitude_hold_control.launch.py input:=joystick
  ```

#### Parameters:
- **input**: Selects the input method. Accepts `'keyboard'` or `'joystick'`. The default is `'keyboard'`.

## Configuration Files

The package includes configuration files for both keyboard and joystick input in the `config` directory:

- `joy_config.yaml` (for joystick input)
- `keyboard_joy_config.yaml` (for keyboard input)

These configuration files are automatically selected based on the `input` argument.