import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    description_dir = get_package_share_directory('orbslam3_ros2')
    urdf_file = os.path.join(description_dir, 'urdf', 'tbuggy.urdf')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_file).read()}],
        ),

        ExecuteProcess(
            cmd=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
                 '--frame-id', 'tbuggy/map',
                 '--child-frame-id', 'tbuggy/base_footprint',],
            output='screen',
        ),
        ExecuteProcess(
            cmd=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
                 '--roll', '-1.5707',
                 '--yaw', '-1.5707',
                 '--frame-id', 'tbuggy/map',
                 '--child-frame-id', 'orbslam3'],
            output='screen',
        ),


    ])
