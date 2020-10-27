from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        # Realsense
        launch_ros.actions.Node(
            package='realsense_ros2_camera', 
            node_executable='realsense_ros2_camera',
            output='screen'),
    ])
