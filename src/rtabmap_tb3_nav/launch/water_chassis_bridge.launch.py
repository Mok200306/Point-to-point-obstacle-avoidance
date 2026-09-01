#!/usr/bin/env python3
"""Start only the WATER SDK ROS bridge for H2/H3 interface tests.

This launch intentionally starts no camera, RTAB-Map or Nav2 node.  It is the
safe way to validate the chassis connection, /odom, odom->base_link and the
/cmd_vel_safe subscriber before starting the complete navigation launch.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("rtabmap_tb3_nav")
    sdk_root = os.path.join(package_share, "water_chassis_sdk_cn_v5_1")

    return LaunchDescription([
        DeclareLaunchArgument(
            "water_sdk_root", default_value=sdk_root,
            description="Installed/source root containing water_chassis_sdk/."),
        DeclareLaunchArgument(
            "water_config_path", default_value="",
            description="Optional WATER SDK config.json override."),
        DeclareLaunchArgument(
            "water_robot_host", default_value="192.168.10.10",
            description="WATER chassis IP; confirm on site."),
        DeclareLaunchArgument(
            "water_robot_port", default_value="31001",
            description="WATER vendor TCP port; confirm on site."),
        DeclareLaunchArgument(
            "water_gateway_port", default_value="8080",
            description="Local SDK Gateway HTTP port."),
        DeclareLaunchArgument(
            "water_connect_timeout_s", default_value="12.0"),
        DeclareLaunchArgument(
            "water_command_rate_hz", default_value="10.0"),
        DeclareLaunchArgument(
            "water_state_rate_hz", default_value="5.0"),
        DeclareLaunchArgument(
            "water_odom_rate_hz", default_value="20.0"),
        DeclareLaunchArgument(
            "water_command_timeout_s", default_value="0.25"),
        DeclareLaunchArgument(
            "water_state_timeout_s", default_value="1.2"),
        DeclareLaunchArgument(
            "water_max_linear_velocity", default_value="0.12",
            description="Bridge-side first-boot linear speed cap in m/s."),
        DeclareLaunchArgument(
            "water_max_angular_velocity", default_value="0.35",
            description="Bridge-side first-boot angular speed cap in rad/s."),
        DeclareLaunchArgument(
            "water_auto_start_gateway", default_value="true"),
        DeclareLaunchArgument(
            "water_enable_motion", default_value="false",
            description=(
                "Explicitly allow /cmd_vel_safe forwarding to WATER. Keep "
                "false for diagnostics-only startup.")),
        DeclareLaunchArgument(
            "water_allow_provisional_odom", default_value="false",
            description=(
                "Explicit software-integration mode. It publishes only a "
                "provisional velocity-integrated /odom; real encoder odom "
                "is required for formal navigation.")),
        Node(
            package="rtabmap_tb3_nav",
            executable="water_chassis_ros_bridge.py",
            name="water_chassis_ros_bridge",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "sdk_root": LaunchConfiguration("water_sdk_root"),
                "config_path": LaunchConfiguration("water_config_path"),
                "robot_host": LaunchConfiguration("water_robot_host"),
                "robot_port": LaunchConfiguration("water_robot_port"),
                "gateway_port": LaunchConfiguration("water_gateway_port"),
                "connect_timeout_s": LaunchConfiguration("water_connect_timeout_s"),
                "auto_start_gateway": LaunchConfiguration("water_auto_start_gateway"),
                "enable_motion": LaunchConfiguration("water_enable_motion"),
                "cmd_vel_topic": "/cmd_vel_safe",
                "odom_topic": "/odom",
                "odom_frame": "odom",
                "base_frame": "base_link",
                "command_rate_hz": LaunchConfiguration("water_command_rate_hz"),
                "state_rate_hz": LaunchConfiguration("water_state_rate_hz"),
                "odom_rate_hz": LaunchConfiguration("water_odom_rate_hz"),
                "command_timeout_s": LaunchConfiguration("water_command_timeout_s"),
                "state_timeout_s": LaunchConfiguration("water_state_timeout_s"),
                "max_linear_velocity": LaunchConfiguration("water_max_linear_velocity"),
                "max_angular_velocity": LaunchConfiguration("water_max_angular_velocity"),
                "publish_tf": True,
                "allow_provisional_odom": LaunchConfiguration(
                    "water_allow_provisional_odom"),
                "status_topic": "/water_chassis/status",
            }],
        ),
    ])
