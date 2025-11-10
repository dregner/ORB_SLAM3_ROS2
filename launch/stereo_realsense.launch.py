from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'vocabulary',
            default_value=PathJoinSubstitution([
                FindPackageShare('orbslam3_ros2'),
                'vocabulary',
                'ORBvoc.txt'
            ]),
            description='Path to the ORB_SLAM3 vocabulary file'
        ),
        DeclareLaunchArgument(
            'pangolin',
            default_value="False",
            description='Use the viewer'
        ),
        DeclareLaunchArgument(
            'yaml_file',
            default_value='stereo_realsense.yaml',
            description='YAML config file for RealSense D455'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='VORIS/SM2',
            description='Namespace of the RealSense camera'
        ),
        DeclareLaunchArgument(
            'rescale',
            default_value='True',
            description='Rescale Image'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='orbslam3',
            description='frame_id'
        ),
        DeclareLaunchArgument(
            'parent_frame_id',
            default_value='VORIS/SM2/base_link',
            description='parent_frame_id'
        ),
        DeclareLaunchArgument(
            'child_frame_id',
            default_value='VORIS/SM2/left_camera_link',
            description='child_frame_id'
        ),

        Node(
            package='orbslam3_ros2',
            executable='stereo',
            name='stereo_orbslam3',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            arguments=[
                LaunchConfiguration('vocabulary'),
                PathJoinSubstitution([
                    FindPackageShare('orbslam3_ros2'),
                    'config',
                    'stereo',
                    LaunchConfiguration('yaml_file')
                ]),
                'True',
                LaunchConfiguration('pangolin')
            ],
            parameters=[
                {'rescale': LaunchConfiguration('rescale')},
                {'frame_id': LaunchConfiguration('frame_id')},
                {'parent_frame_id': LaunchConfiguration('parent_frame_id')},
                {'child_frame_id': LaunchConfiguration('child_frame_id')}
            ],
            remappings=[
                # Ajustado aos tópicos exatos da RealSense sob /VORIS/SM2
                ('camera/left/image_raw', '/VORIS/SM2/infra1/image_rect_raw'),
                ('camera/right/image_raw', '/VORIS/SM2/infra2/image_rect_raw'),
                ('camera/left/camera_info', '/VORIS/SM2/infra1/camera_info'),
                ('camera/right/camera_info', '/VORIS/SM2/infra2/camera_info'),
            ]
        )
    ])

