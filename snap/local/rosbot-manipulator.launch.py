# Copyright 2024 Husarion sp. z o.o.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    IncludeLaunchDescription
)
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    ThisLaunchFileDir,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_microros_agent_node(context, *args, **kwargs):
    env_setup_actions = []

    ros_domain_id = os.environ.get("ROS_DOMAIN_ID")
    if ros_domain_id:
        env_setup_actions.append(
            SetEnvironmentVariable(name="XRCE_DOMAIN_ID_OVERRIDE", value=ros_domain_id)
        )

    port = LaunchConfiguration("port").perform(context)

    localhost_only_fastrtps_profiles_file = LaunchConfiguration(
        "localhost_only_fastrtps_profiles_file"
    ).perform(context)

    if os.environ.get("ROS_LOCALHOST_ONLY") == "1":
        env_setup_actions.extend(
            [
                LogInfo(
                    msg=[
                        "ROS_LOCALHOST_ONLY set to 1. Using FASTRTPS_DEFAULT_PROFILES_FILE=",
                        localhost_only_fastrtps_profiles_file,
                        ".",
                    ]
                ),
                SetEnvironmentVariable(name="RMW_IMPLEMENTATION", value="rmw_fastrtps_cpp"),
                SetEnvironmentVariable(
                    name="FASTRTPS_DEFAULT_PROFILES_FILE",
                    value=localhost_only_fastrtps_profiles_file,
                ),
            ]
        )

    microros_agent_node = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        arguments=["udp4", "--port", port],
        output="screen",
    )

    return env_setup_actions + [microros_agent_node]

def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for all topics and tfs",
    )

    declare_port_arg = DeclareLaunchArgument(
        "port",
        default_value="8888",
        description="UDP4 port for micro-ROS agent",
    )

    # Locate the rosbot_bringup package
    package_dir = FindPackageShare("rosbot_xl_bringup").find("rosbot_xl_bringup")

    # Construct the path to the XML file within the package
    fastrtps_profiles_file = PathJoinSubstitution(
        [package_dir, "config", "microros_localhost_only.xml"]
    )

    declare_localhost_only_fastrtps_profiles_file_arg = DeclareLaunchArgument(
        "localhost_only_fastrtps_profiles_file",
        default_value=fastrtps_profiles_file,
        description=(
            "Path to the Fast RTPS default profiles file for Micro-ROS agent for localhost only"
            " setup"
        ),
    )

    mecanum = LaunchConfiguration("mecanum")
    declare_mecanum_arg = DeclareLaunchArgument(
        "mecanum",
        default_value="False",
        description="Whether to use mecanum drive controller, otherwise use diff drive",
    )

    camera_model = LaunchConfiguration("camera_model")
    declare_camera_model_arg = DeclareLaunchArgument(
        "camera_model",
        default_value="None",
        description="Add camera model to the robot URDF",
        choices=[
            "None",
            "intel_realsense_d435",
            "orbbec_astra",
            "stereolabs_zed",
            "stereolabs_zedm",
            "stereolabs_zed2",
            "stereolabs_zed2i",
            "stereolabs_zedx",
            "stereolabs_zedxm",
        ],
    )

    lidar_model = LaunchConfiguration("lidar_model")
    declare_lidar_model_arg = DeclareLaunchArgument(
        "lidar_model",
        default_value="slamtec_rplidar_s1",
        description="Add LiDAR model to the robot URDF",
        choices=[
            "None",
            "slamtec_rplidar_a2",
            "slamtec_rplidar_a3",
            "slamtec_rplidar_s1",
            "slamtec_rplidar_s2",
            "slamtec_rplidar_s3",
            "velodyne_puck",
        ],
    )

    include_camera_mount = LaunchConfiguration("include_camera_mount")
    declare_include_camera_mount_arg = DeclareLaunchArgument(
        "include_camera_mount",
        default_value="False",
        description="Whether to include camera mount to the robot URDF",
    )

    use_sim = LaunchConfiguration("use_sim")
    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="False",
        description="Whether simulation is used",
    )

    simulation_engine = LaunchConfiguration("simulation_engine")
    declare_simulation_engine_arg = DeclareLaunchArgument(
        "simulation_engine",
        default_value="ignition-gazebo",
        description="Which simulation engine will be used",
    )

    combined_launch_deprecated = LaunchConfiguration("combined_launch_deprecated")
    declare_combined_launch_deprecated_arg = DeclareLaunchArgument(
        "combined_launch_deprecated",
        default_value="False",
    )

    rosbot_xl_controller = FindPackageShare("rosbot_xl_controller")
    rosbot_xl_bringup = FindPackageShare("rosbot_xl_bringup")
    rosbot_xl_manipulation_controller = FindPackageShare("rosbot_xl_manipulation_controller")
    rosbot_xl_manipulation_moveit = FindPackageShare("rosbot_xl_manipulation_moveit")

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    rosbot_xl_controller,
                    "launch",
                    "controller.launch.py",
                ]
            )
        ),
        launch_arguments={
            "mecanum": mecanum,
            "lidar_model": lidar_model,
            "camera_model": camera_model,
            "include_camera_mount": include_camera_mount,
            "use_sim": use_sim,
            "simulation_engine": simulation_engine,
            "namespace": namespace,
        }.items(),
    )

    ekf_config = PathJoinSubstitution([rosbot_xl_bringup, "config", "ekf.yaml"])

    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
        namespace=namespace,
    )

    laser_filter_config = PathJoinSubstitution(
        [
            rosbot_xl_bringup,
            "config",
            "laser_filter.yaml",
        ]
    )

    laser_filter_node = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[
            laser_filter_config,
        ],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
        namespace=namespace,
    )

    launch_joy_node = LaunchConfiguration("launch_joy_node")
    declare_launch_joy_node_arg = DeclareLaunchArgument(
        "launch_joy_node",
        default_value="False",
    )

    manipulator_usb_port = LaunchConfiguration("manipulator_usb_port")
    declare_manipulator_usb_port_arg = DeclareLaunchArgument(
        "manipulator_usb_port",
        default_value="/dev/ttyMANIPULATOR",
    )

    manipulator_baud_rate = LaunchConfiguration("manipulator_baud_rate")
    declare_manipulator_baud_rate_arg = DeclareLaunchArgument(
        "manipulator_baud_rate",
        default_value="1000000",
    )

    joy_servo_config = LaunchConfiguration("joy_servo_params_file")
    declare_joy_servo_config_arg = DeclareLaunchArgument(
        "joy_servo_params_file",
        default_value=PathJoinSubstitution(
            [
                rosbot_xl_manipulation_moveit,
                "config",
                "joy_servo.yaml",
            ]
        ),
        description="ROS2 parameters file to use with joy_servo node",
    )

    joint1_limit_min = LaunchConfiguration("joint1_limit_min")
    declare_joint1_limit_min_arg = DeclareLaunchArgument(
        "joint1_limit_min",
        default_value="-2.356",
        description="Min angle (in radians) that can be achieved by rotating joint1 of the manipulator",
    )
    joint1_limit_max = LaunchConfiguration("joint1_limit_max")
    declare_joint1_limit_max_arg = DeclareLaunchArgument(
        "joint1_limit_max",
        default_value="5.934",
        description="Max angle (in radians) that can be achieved by rotating joint1 of the manipulator",
    )

    manipulator_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    rosbot_xl_manipulation_controller,
                    "launch",
                    "controller.launch.py",
                ]
            )
        ),
        launch_arguments={
            "manipulator_usb_port": manipulator_usb_port,
            "manipulator_baud_rate": manipulator_baud_rate,
            "joint1_limit_min": joint1_limit_min,
            "joint1_limit_max": joint1_limit_max,
            "mecanum": mecanum,
            "use_sim": use_sim,
        }.items(),
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    rosbot_xl_manipulation_moveit,
                    "launch",
                    "move_group.launch.py",
                ]
            )
        ),
        launch_arguments={
            "joint1_limit_min": joint1_limit_min,
            "joint1_limit_max": joint1_limit_max,
            "mecanum": mecanum,
            "use_sim": use_sim,
        }.items(),
    )

    servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    rosbot_xl_manipulation_moveit,
                    "launch",
                    "servo.launch.py",
                ]
            )
        ),
        launch_arguments={
            "launch_joy_node": launch_joy_node,
            "joy_servo_params_file": joy_servo_config,
            "joint1_limit_min": joint1_limit_min,
            "joint1_limit_max": joint1_limit_max,
            "mecanum": mecanum,
            "use_sim": use_sim,
        }.items(),
    )

    # Retrieve the SNAP_COMMON environment variable
    snap_common = os.environ.get('SNAP_COMMON', '/var/snap/rosbot-xl/common/')  # Provide a default path in case it's not set

    # Declare the parameters file launch argument
    joy_params_file = LaunchConfiguration("params_file")
    joy_params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(snap_common, 'teleop_twist_joy_params.yaml')
    )

    # Node configuration for teleop_twist_joy
    teleop_twist_joy = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[joy_params_file],
        output='screen',
        namespace=namespace,
    )

    return LaunchDescription(
        [
            declare_port_arg,
            declare_localhost_only_fastrtps_profiles_file_arg,
            declare_namespace_arg,
            declare_mecanum_arg,
            declare_lidar_model_arg,
            declare_camera_model_arg,
            declare_include_camera_mount_arg,
            declare_use_sim_arg,
            declare_simulation_engine_arg,
            declare_combined_launch_deprecated_arg,
            declare_launch_joy_node_arg,
            declare_manipulator_usb_port_arg,
            declare_manipulator_baud_rate_arg,
            declare_joy_servo_config_arg,
            declare_joint1_limit_min_arg,
            declare_joint1_limit_max_arg,
            SetParameter(name="use_sim_time", value=use_sim),
            controller_launch,
            robot_localization_node,
            laser_filter_node,
            OpaqueFunction(function=generate_microros_agent_node),
            manipulator_controller_launch,
            moveit_launch,
            servo_launch,
            joy_params_file_arg,
            teleop_twist_joy
        ]
    )
