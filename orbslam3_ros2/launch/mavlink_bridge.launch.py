import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config_path = PathJoinSubstitution([
        FindPackageShare('orbslam3_ros2'),
        'config',
        'mavlink_bridge.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace of the bridge node'
        ),
        DeclareLaunchArgument(
            'connection_url',
            default_value='udpout:127.0.0.1:14550',
            description='MAVLink target connection URL (e.g. udpout:127.0.0.1:14550 or udpout:192.168.2.2:14550)'
        ),
        DeclareLaunchArgument(
            'bridge_mode',
            default_value='both',
            description='Bridge mode: "estimate", "delta", or "both"'
        ),
        DeclareLaunchArgument(
            'pose_topic',
            default_value='pose_cov',
            description='Topic name for geometry_msgs/PoseWithCovarianceStamped'
        ),
        DeclareLaunchArgument(
            'status_topic',
            default_value='slam_status',
            description='Topic name for orbslam3_msgs/SlamStatus'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config_path,
            description='Path to YAML configuration file'
        ),

        Node(
            package='orbslam3_ros2',
            executable='mavlink_bridge_node.py',
            name='orbslam3_mavlink_bridge',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                {
                    'connection_url': LaunchConfiguration('connection_url'),
                    'bridge_mode': LaunchConfiguration('bridge_mode'),
                    'pose_topic': LaunchConfiguration('pose_topic'),
                    'status_topic': LaunchConfiguration('status_topic'),
                }
            ]
        )
    ])
