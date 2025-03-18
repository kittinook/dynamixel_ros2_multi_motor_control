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
- `dynamixel_hardware_interface` ROS2 package

### Installing Dependencies

```bash
# Install Dynamixel SDK
sudo apt install ros-humble-dynamixel-sdk

# Install hardware interface dependencies
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers

# Install dynamixel_hardware_interface from source
cd ~/ros2_ws/src
git clone https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface.git
```

The `dynamixel_hardware_interface` package provides the necessary hardware abstraction to integrate Dynamixel motors with ROS2 Control. It handles the lower-level communication and provides a standardized interface for controlling the motors through ROS2 Control.

### Building from Source

Clone and build all the required packages:

```bash
cd ~/ros2_ws/src
git clone <repository_url>
git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b humble https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface.git
git clone -b humble https://github.com/ROBOTIS-GIT/dynamixel_interfaces.git
cd ..
colcon build --symlink-install
source install/setup.bash
```

## Hardware Interface Configuration

When using the `dynamixel_hardware_interface`, you'll need to configure your YAML file to properly set up the hardware interface:

```yaml
# Example controller configuration
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    position_controllers:
      type: position_controllers/JointGroupPositionController

# Dynamixel hardware interface configuration
dynamixel_hardware:
  ros__parameters:
    loop_rate: 100  # Control loop rate in Hz
    use_dummy: false  # Set to true for simulation mode

    # Serial communication settings
    usb_port:
      port_name: "/dev/ttyUSB0"
      baudrate: 57600
      timeout: 0.5

    # Motor configuration
    motors:
      - id: 1
        model: "XM430-W210"
        mode: "position"
        enable_torque: true
        joint_name: "joint_1"
      - id: 2
        model: "XM430-W210"
        mode: "position"
        enable_torque: true
        joint_name: "joint_2"
```

### Integration with ROS2 Control

The `dynamixel_hardware_interface` works with the ROS2 Control framework. You'll need to configure both your hardware interface and controllers:

```yaml
# ROS2 Control configuration
controller_manager:
  ros__parameters:
    update_rate: 100
    
    # List of controllers
    dynamixel_controller:
      type: position_controllers/JointGroupPositionController

dynamixel_controller:
  ros__parameters:
    joints:
      - joint_1
      - joint_2
    interface_name: position
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
```

## Usage

### Starting with Hardware Interface

When using the hardware interface, launch your node with:

```bash
ros2 launch dynamixel_ros2_multi_motor_control multi_motor_hardware_launch.py
```

### Controlling Motors with ROS2 Control

```bash
# Send commands through the controller
ros2 topic pub /dynamixel_controller/commands std_msgs/msg/Float64MultiArray "data: [1.57, 0.0]" -1
```

## Hardware Interface vs. Direct SDK Control

This package supports two methods of controlling Dynamixel motors:

1. **Direct SDK Control** (original implementation): Directly uses Dynamixel SDK to communicate with motors
2. **Hardware Interface Control**: Uses `dynamixel_hardware_interface` to integrate with ROS2 Control

The hardware interface approach is recommended for:
- Integration with other ROS2 Control components
- Using standard controller interfaces
- More complex control systems with multiple different hardware types

## Troubleshooting Hardware Interface

### Common Issues

- **Controller Not Spawning**: Verify controller configuration in your YAML file
- **Hardware Interface Not Found**: Check that `dynamixel_hardware_interface` is built and sourced
- **Position Control Not Working**: Ensure torque is enabled and operating mode is set correctly

## Additional Resources

- [dynamixel_hardware_interface Documentation](https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface)
- [ROS2 Control Documentation](https://control.ros.org/master/index.html)
- [Dynamixel SDK Manual](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/)

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