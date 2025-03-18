from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('dynamixel_control'),
        'config',
        'motor_params.yaml'
    )

    motor_control_node = Node(
        package='dynamixel_control',
        executable='current_node_ex3',
        output='screen',
        parameters=[config]
    )

    return LaunchDescription([motor_control_node])
