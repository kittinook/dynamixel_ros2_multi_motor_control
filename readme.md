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
git clone <repository_url>
cd ..
colcon build --symlink-install
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
motors:
  - id: 1
    operating_mode: 3  # Position mode
    position_p_gain: 800
    position_i_gain: 0
    position_d_gain: 0
  - id: 2
    operating_mode: 1  # Velocity mode
    velocity_p_gain: 100
    velocity_i_gain: 1920
    velocity_d_gain: 0
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

## Contributing

Pull requests and issues are welcome. Please follow these guidelines:

1. Clearly document changes and rationale
2. Test thoroughly before submission
3. Follow ROS2 coding standards
4. Update documentation as needed

## License

Distributed under the MIT License. See `LICENSE` for more details.

## Contact

For questions, suggestions, or support:
- Open an issue on the repository
- Contact the repository maintainer at: [maintainer@example.com](mailto:maintainer@example.com)

---

*This package is not affiliated with ROBOTIS, the manufacturer of Dynamixel motors.*