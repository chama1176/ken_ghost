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
        get_package_share_directory('ken_mission_manager'),
        'config',
        'ps4_auto_config.yaml')

    mission_manager_node = Node(
        package='ken_mission_manager',
        node_executable='ken_mission_manager',
        output='screen',
        parameters=[config])

    return LaunchDescription([joy_node, mission_manager_node])
