import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('ken_path_planner'),
        'config',
        'path_planning_config.yaml')

    path_planner_node = Node(
        package='ken_path_planner',
        node_executable='ken_path_planner',
        output='screen',
        parameters=[config])

    return LaunchDescription([path_planner_node])
