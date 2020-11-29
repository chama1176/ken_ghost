import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    joy_node = Node(
        package='joy',
        node_executable='joy_node',
        output='screen'
    )

    config = os.path.join(
        get_package_share_directory('ken_teleop'),
        'config',
        'ps4_config.yaml')

    teleop_node = Node(
        package='ken_teleop',
        node_executable='ken_teleop_joy',
        output='screen',
        parameters=[config])

    return LaunchDescription([joy_node, teleop_node])
