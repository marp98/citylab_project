from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_patrol',
            executable='direction_service',
            name='direction_service_node',
            output='screen'
        ),
        Node(
            package='robot_patrol',
            executable='patrol_with_service',
            name='robot_patrol_node',
            output='screen'
        )
    ])