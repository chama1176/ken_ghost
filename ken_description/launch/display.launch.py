import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

import xacro

def generate_launch_description():
    share_dir_path = os.path.join(get_package_share_directory('ken_description'))
    xacro_path = os.path.join(share_dir_path, 'urdf', 'ken.urdf.xacro')
    urdf_path = os.path.join(share_dir_path, 'urdf', 'ken.urdf')
    doc = xacro.process_file(xacro_path)
    robot_desc = doc.toprettyxml(indent='  ')
    params = {'robot_description': robot_desc}
    f = open(urdf_path, 'w')
    f.write(robot_desc)
    f.close()

    rsp_node = Node(package='robot_state_publisher',
        node_executable='robot_state_publisher',
        output='both',
        arguments=[urdf_path])
    jspg_node = Node(package='joint_state_publisher_gui',
        node_executable='joint_state_publisher_gui',
        output='screen')

    rviz_config_dir_path = os.path.join(get_package_share_directory('ken_description'))
    rviz_config_path = os.path.join(rviz_config_dir_path, 'config', 'display.rviz')
    rviz_node = Node(package='rviz2',
        node_executable='rviz2',
        output='log',
        arguments=['-d', rviz_config_path])

    return LaunchDescription([rsp_node, jspg_node, rviz_node])
