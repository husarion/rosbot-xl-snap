import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Node configuration for foxglove_bridge
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'port': 8765,
            'address': '127.0.0.1',
            'capabilities': ['clientPublish', 'parameters', 'parametersSubscribe', 'services', 'connectionGraph', 'assets']
            # 'capabilities': ['clientPublish', 'connectionGraph', 'assets']
        }],
        output='screen'
    )

    # Return the launch description which includes both nodes and the launch argument
    return LaunchDescription([
        foxglove_bridge,
    ])
