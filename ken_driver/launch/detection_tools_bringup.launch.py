import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro

ken_driver_dir = os.path.join(
    get_package_share_directory('ken_driver'), 'launch')
ken_target_detect_dir = os.path.join(
    get_package_share_directory('ken_target_detect'), 'launch')


def generate_launch_description():

    rs_include_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [ken_driver_dir, '/rs.launch.py']),)

    detect_include_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [ken_target_detect_dir, '/detect.launch.py']),)

    return LaunchDescription([rs_include_launch, detect_include_launch])
