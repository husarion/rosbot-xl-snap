import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Retrieve the SNAP_COMMON environment variable
    snap_common = os.environ.get('SNAP_COMMON', '/var/snap/rosbot-xl/common/')  # Provide a default path in case it's not set

    # Declare the parameters file launch argument
    params_file = LaunchConfiguration("params_file")
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(snap_common, 'teleop_twist_joy_params.yaml')
    )

    namespace = LaunchConfiguration("namespace")
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for all topics",
    )

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

    # Node configuration for teleop_twist_joy
    teleop_twist_joy = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[params_file],
        output='screen',
        namespace=namespace,
    )

    # Return the launch description which includes both nodes and the launch argument
    return LaunchDescription([
        declare_namespace_arg,
        params_file_arg,
        foxglove_bridge,
        teleop_twist_joy
    ])
