"""Start the mission node with our PX4 topic names."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Launch the mission node with px4_topics.yaml and adjustable timeouts."""
    topics = os.path.join(
        get_package_share_directory('drone_bringup'), 'config', 'px4_topics.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('climb_timeout', default_value='20.0'),
        DeclareLaunchArgument('odometry_timeout', default_value='0.5'),
        Node(
            package='drone_mission',
            executable='mission',
            output='screen',
            parameters=[topics, {
                'climb_timeout': ParameterValue(
                    LaunchConfiguration('climb_timeout'), value_type=float),
                'odometry_timeout': ParameterValue(
                    LaunchConfiguration('odometry_timeout'), value_type=float),
            }],
        ),
    ])
