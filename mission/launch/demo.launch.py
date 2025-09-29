from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mavros',
            executable='mavros_node',
            output='screen',
            parameters=[{'fcu_url': 'udp://:14540@localhost:14557'}]
        ),
        Node(
            package='drone_demo',
            executable='mission_node',
            output='screen'
        )
    ])
