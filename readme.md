# Dynamixel ROS2 Multi-Motor Control

![Dynamixel Motors](https://via.placeholder.com/800x200.png?text=Dynamixel+ROS2+Multi-Motor+Control)

## Overview

This package provides comprehensive ROS2-based control for multiple Dynamixel motors, enabling seamless configuration and real-time feedback. It supports various operating modes including current control, velocity control, and position control, with highly customizable parameters via YAML configuration files.

## Features

- **Real-time feedback** using Group Bulk Read functionality
- **Multiple control modes** for versatile application needs:
  - Current-based control
  - Velocity control
  - Position control
- **Flexible command interfaces**:
  - Individual motor control
  - Synchronized group control
- **Customizable parameters**:
  - PID control settings
  - Motor configuration
  - Communication parameters
- **ROS2 integration** with standard message types

## Installation

### Prerequisites

- ROS2 Humble (or later)
- Dynamixel SDK
- `dynamixel_sdk` ROS2 package

### Building from Source

Clone this repository into your ROS2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/kittinook/dynamixel_ros2_multi_motor_control.git
git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b humble https://github.com/ROBOTIS-GIT/dynamixel_interfaces.git
```

Build the package:

```bash
cd ..
colcon build
```

Source your workspace:
```bash
source install/setup.bash
```

## Configuration

### Motor Configuration

Modify motor parameters in the YAML configuration file:

```bash
nano ~/ros2_ws/src/dynamixel_ros2_multi_motor_control/config/motor_params.yaml
```

### Example Configuration

```yaml
multi_motor_control_node:
  ros__parameters:
    motor_ids: [12, 41]                         # List of Dynamixel motor IDs you want to control (e.g., [12, 41]).
    default_current_factor: 2.69                # Factor to convert the motor’s raw current reading into milliamps (mA).
    default_velocity_factor: 0.229              # Factor to convert the motor’s raw velocity value into revolutions per minute (RPM).
    default_position_full_scale: 4095.0         # Maximum position value representing a full rotation of the motor (e.g., 4095 for 360 degrees).
    default_operating_mode: 0                   # The default operating mode for the motor:
                                                #  0: Current Control Mode
                                                #  1: Velocity Control Mode
                                                #  3: Position Control Mode
    default_temperature_limit: 80               # Maximum operating temperature (°C) before the motor shuts down for protection.
    default_max_voltage_limit: 160              # Maximum voltage limit (in 0.1V units, e.g., 160 = 16.0V).
    default_min_voltage_limit: 95               # Minimum operational voltage (in 0.1V units, e.g., 95 = 9.5V).
    default_pwm_limit: 885                      # Maximum PWM (Pulse Width Modulation) value allowed.
    default_current_limit: 1193                 # Maximum allowable current (in mA) that the motor can draw.
    default_velocity_limit: 200                 # Maximum allowable velocity of the motor (in RPM).
    default_max_position_limit: 4095            # Maximum allowed position value for the motor (e.g., 4095 = 360 degrees).
    default_min_position_limit: 0               # Minimum allowed position value for the motor (e.g., 0 = 0 degrees).
    
    # PID controller parameters (Integral and Proportional gains) used in Velocity Control mode.
    default_velocity_i_gain: 1920               
    default_velocity_p_gain: 100
    
    # PID controller parameters (Derivative, Integral, and Proportional gains) used in Position Control mode.                
    default_position_d_gain: 0
    default_position_i_gain: 0
    default_position_p_gain: 800
    
    # Feedforward gains used to improve positional control performance (usually set to 0 if unused).
    default_feedforward_2nd_gain: 0             
    default_feedforward_1st_gain: 0

    port_name: "/dev/ttyUSB0"                   # Serial port name for connecting to Dynamixel motors (e.g., /dev/ttyUSB0).
    baudrate: 1000000                           # Communication speed with Dynamixel motors (e.g., 1000000 = 1 Mbps).
    
    motor_12:
      current_factor: 2.7
      velocity_factor: 0.225
      position_full_scale: 4095.0
      operating_mode: 1
      temperature_limit: 80
      max_voltage_limit: 160
      min_voltage_limit: 95
      pwm_limit: 885
      current_limit: 1193
      velocity_limit: 200
      max_position_limit: 4095
      min_position_limit: 0
      velocity_i_gain: 1920
      velocity_p_gain: 100
      position_d_gain: 0
      position_i_gain: 0
      position_p_gain: 800
      feedforward_2nd_gain: 0
      feedforward_1st_gain: 0

    motor_41:
      current_factor: 2.69
      velocity_factor: 0.229
      position_full_scale: 4095.0
      operating_mode: 1
      temperature_limit: 80
      max_voltage_limit: 160
      min_voltage_limit: 95
      pwm_limit: 885
      current_limit: 1193
      velocity_limit: 200
      max_position_limit: 4095
      min_position_limit: 0
      velocity_i_gain: 1920
      velocity_p_gain: 100
      position_d_gain: 0
      position_i_gain: 0
      position_p_gain: 800
      feedforward_2nd_gain: 0
      feedforward_1st_gain: 0
```

## Usage

### Starting the Node

Launch the node using the provided launch file:

```bash
ros2 launch dynamixel_ros2_multi_motor_control dynamixel_hardware_interface.launch.py
```

Custom configurations can be specified:

```bash
ros2 launch dynamixel_ros2_multi_motor_control dynamixel_hardware_interface.launch.py config_file:=my_config.yaml
```

### Operating Modes

| Mode | Value | Description |
|------|-------|-------------|
| Current | 0 | Control motors by current |
| Velocity | 1 | Control motor velocity |
| Position | 3 | Control motor position |

### Control Topics

#### Individual Motor Control

- Current: `/motor_<id>/goal_current` (Float32)
- Velocity: `/motor_<id>/goal_velocity` (Float32)
- Position: `/motor_<id>/goal_position` (Float32)

#### Group Control

- Current: `/group_goal_current` (Float32MultiArray)
- Velocity: `/group_goal_velocity` (Float32MultiArray)
- Position: `/group_goal_position` (Float32MultiArray)

### Feedback Topics

- `/motor_<id>/present_current` (Float32)
- `/motor_<id>/present_velocity` (Float32)
- `/motor_<id>/present_position` (Float32)
- `/joint_states` (sensor_msgs/JointState)

## Example Usage

### Position Control Example

```bash
# Control motor 1 position
ros2 topic pub /motor_1/goal_position std_msgs/msg/Float32 "data: 512.0" -1

# Control multiple motors at once
ros2 topic pub /group_goal_position std_msgs/msg/Float32MultiArray "data: [512.0, 512.0]" -1
```

### Reading Feedback

```bash
# Monitor position feedback
ros2 topic echo /motor_1/present_position

# Monitor all joint states
ros2 topic echo /joint_states
```

## Troubleshooting

### Common Issues

- **Communication Errors**: Verify correct USB port and permissions
- **Motor Not Responding**: Check ID, baudrate, and power supply
- **Unexpected Behavior**: Ensure correct operating mode configuration

### Setting USB Permissions

```bash
sudo chmod 666 /dev/ttyUSB0
```

---

*This package is not affiliated with ROBOTIS, the manufacturer of Dynamixel motors.*
