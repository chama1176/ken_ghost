import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
        
    node=Node(
        package = 'joy',
        node_executable = 'joy_node',
        output='screen'
    )
    ld.add_action(node)
    return ld