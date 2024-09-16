import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    joy_dev = LaunchConfiguration('joy_dev')
    config_filepath = LaunchConfiguration('config_filepath')
    publish_stamped_twist = LaunchConfiguration('publish_stamped_twist')
    namespace = LaunchConfiguration('namespace')  # Capture the namespace

    return LaunchDescription([
        DeclareLaunchArgument(
            'joy_vel',
            default_value='cmd_vel',
            description='The topic name to which velocity commands will be published.'
        ),
        DeclareLaunchArgument(
            'joy_dev',
            default_value='/dev/input/js0',
            description='The device file for the joystick (e.g., /dev/input/js0).'
        ),
        DeclareLaunchArgument(
            'config_filepath',
            default_value=[
                os.path.join(os.environ.get("SNAP_COMMON"), 'joy_teleop.config.yaml')
            ],
            description='The file path to the configuration YAML file for the teleop_twist_joy node.'
        ),
        DeclareLaunchArgument(
            'publish_stamped_twist',
            default_value='false',  # Default set to false
            description='Flag to control whether to publish geometry_msgs/TwistStamped messages instead of geometry_msgs/Twist.'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Optional namespace to apply to the nodes and remapped topics. If left empty, no namespace is applied.'
        ),

        Node(
            package='joy', 
            executable='joy_node', 
            name='joy_node',
            namespace=namespace,  # Apply the namespace here
            parameters=[{
                'dev': joy_dev,
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
            }]
        ),
        Node(
            package='teleop_twist_joy', 
            executable='teleop_node',
            name='teleop_twist_joy_node', 
            namespace=namespace,  # Apply the namespace here
            parameters=[config_filepath, {'publish_stamped_twist': publish_stamped_twist}],
            remappings={('/cmd_vel', LaunchConfiguration('joy_vel'))},  # Remap the topic under the namespace
        ),
    ])
