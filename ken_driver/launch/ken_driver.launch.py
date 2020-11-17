import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    share_dir_path = os.path.join(
        get_package_share_directory('ken_description'))
    xacro_path = os.path.join(share_dir_path, 'urdf', 'ken.urdf.xacro')
    urdf_path = os.path.join(share_dir_path, 'urdf', 'ken.urdf')
    doc = xacro.process_file(xacro_path)
    robot_desc = doc.toprettyxml(indent='  ')
    params = {'robot_description': robot_desc}
    f = open(urdf_path, 'w')
    f.write(robot_desc)
    f.close()

    config = os.path.join(
        get_package_share_directory('ken_driver'),
        'config',
        'ken_controllers.yaml')

    ken_driver_node = Node(package='ken_driver',
                           node_executable='ken_controller',
                           output='both',
                           #        parameters=[{'controller_name': 'ken_joint_trajectory_controller'}])
                           parameters=[config])

    return LaunchDescription([ken_driver_node])
