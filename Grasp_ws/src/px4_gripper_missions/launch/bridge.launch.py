"""Starts ros_gz_bridge's parameter_bridge from config/bridge.yaml.

Assumes PX4 SITL + Gazebo are already running against
x500_gripper_armbot.sdf (per 02_run_sitl_baseline.md's flow, extended
per wayfinder/MAP.md's "Implemented" section) — this launch file only
adds the Armbot-joint / world-pose bridge on top, it does not spawn or
relaunch the sim itself.

Usage: ros2 launch px4_gripper_missions bridge.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bridge_config = os.path.join(
        get_package_share_directory('px4_gripper_missions'), 'config', 'bridge.yaml'
    )
    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='px4_gripper_missions_bridge',
            output='screen',
            parameters=[{'config_file': bridge_config}],
        ),
    ])
