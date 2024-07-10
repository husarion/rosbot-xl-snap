import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    # Retrieve the SNAP_COMMON environment variable
    snap_common = os.environ.get('SNAP_COMMON', '/var/snap/rosbot-xl/common/')  # Provide a default path in case it's not set

    namespace = LaunchConfiguration("namespace")
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for all topics",
    )

    # Declare the parameters file launch argument
    params_file = LaunchConfiguration("params_file")
    declare_params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(snap_common, 'teleop_twist_joy_params.yaml')
    )

    # Node configuration for teleop_twist_joy
    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[params_file],
        output='screen',
        namespace=namespace,
    )

    return LaunchDescription(
        [
            declare_namespace_arg,
            declare_params_file_arg,
            teleop_twist_joy_node,
        ]
    )
