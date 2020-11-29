import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
        
    config = os.path.join(
        get_package_share_directory('ken_target_detect'),
        'config',
        'detection_params.yaml'
        )

    node=Node(
        package = 'ken_target_detect',
        node_executable = 'detect',
        parameters = [config],
#            {'red_h_range_min': 20},
#            {'red_h_range_max': 50}
#        ],
        output='screen'
    )
    ld.add_action(node)
    return ld