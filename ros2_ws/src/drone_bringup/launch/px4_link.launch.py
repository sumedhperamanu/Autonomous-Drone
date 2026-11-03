import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Start PX4 Agent, Gazebo clock bridge, and the odometry reader."""
    bridge_config = os.path.join(
        get_package_share_directory('drone_bringup'), 'config', 'gz_bridge.yaml')
    odom_topic = LaunchConfiguration('odom_topic')

    return LaunchDescription([
        DeclareLaunchArgument('odom_topic', default_value='/fmu/out/vehicle_odometry'),
        ExecuteProcess(cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'], output='screen'),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': bridge_config}],
            output='screen'
        ),
        Node(
            package='drone_px4_interface',
            executable='odom_reader',
            parameters=[{'use_sim_time': True, 'topic': odom_topic}],
            output='screen'
        )
    ])
