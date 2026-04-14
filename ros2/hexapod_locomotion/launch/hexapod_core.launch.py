from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    config_dir = Path(get_package_share_directory('hexapod_locomotion')) / 'config'
    servo_dry_run = LaunchConfiguration('servo_dry_run')
    apply_offsets = LaunchConfiguration('apply_offsets')
    yaw_correction_gain = LaunchConfiguration('yaw_correction_gain')
    odom_topic = LaunchConfiguration('odom_topic')
    odom_frame_id = LaunchConfiguration('odom_frame_id')
    base_frame_id = LaunchConfiguration('base_frame_id')
    publish_odom_tf = LaunchConfiguration('publish_odom_tf')
    imu_frame = LaunchConfiguration('imu_frame')
    imu_x = LaunchConfiguration('imu_x')
    imu_y = LaunchConfiguration('imu_y')
    imu_z = LaunchConfiguration('imu_z')
    imu_roll = LaunchConfiguration('imu_roll')
    imu_pitch = LaunchConfiguration('imu_pitch')
    imu_yaw = LaunchConfiguration('imu_yaw')

    return LaunchDescription([
        DeclareLaunchArgument(
            'servo_dry_run',
            default_value='false',
            description='When true, servo_driver logs commands instead of driving hardware.',
        ),
        DeclareLaunchArgument(
            'apply_offsets',
            default_value='true',
            description='Apply saved calibration offsets in servo_driver.',
        ),
        DeclareLaunchArgument(
            'yaw_correction_gain',
            default_value='0.0',
            description='IMU-based yaw damping gain used by locomotion when no yaw is commanded.',
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='odom',
            description='Odometry topic published by locomotion.',
        ),
        DeclareLaunchArgument(
            'odom_frame_id',
            default_value='odom',
            description='Frame id used for the odometry parent frame.',
        ),
        DeclareLaunchArgument(
            'base_frame_id',
            default_value='base_link',
            description='Frame id used for the robot base.',
        ),
        DeclareLaunchArgument(
            'publish_odom_tf',
            default_value='true',
            description='Publish the odom to base_link TF from locomotion.',
        ),
        DeclareLaunchArgument(
            'imu_frame',
            default_value='imu_link',
            description='Frame id stamped onto IMU data and used for the base->imu static TF.',
        ),
        DeclareLaunchArgument(
            'imu_x',
            default_value='0.0',
            description='X offset of the IMU from base_link in metres (forward +).',
        ),
        DeclareLaunchArgument(
            'imu_y',
            default_value='0.0',
            description='Y offset of the IMU from base_link in metres (left +).',
        ),
        DeclareLaunchArgument(
            'imu_z',
            default_value='0.0',
            description='Z offset of the IMU from base_link in metres (up +).',
        ),
        DeclareLaunchArgument(
            'imu_roll',
            default_value='0.0',
            description='Roll offset of the IMU from base_link in radians.',
        ),
        DeclareLaunchArgument(
            'imu_pitch',
            default_value='0.0',
            description='Pitch offset of the IMU from base_link in radians.',
        ),
        DeclareLaunchArgument(
            'imu_yaw',
            default_value='0.0',
            description='Yaw offset of the IMU from base_link in radians.',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu',
            output='screen',
            arguments=[
                '--x', imu_x, '--y', imu_y, '--z', imu_z,
                '--roll', imu_roll, '--pitch', imu_pitch, '--yaw', imu_yaw,
                '--frame-id', base_frame_id, '--child-frame-id', imu_frame,
            ],
        ),
        Node(
            package='hexapod_locomotion',
            executable='imu_publisher',
            name='imu_publisher',
            output='screen',
            parameters=[{
                'frame_id': imu_frame,
            }]
        ),
        Node(
            package='hexapod_locomotion',
            executable='locomotion',
            name='locomotion',
            output='screen',
            parameters=[
                str(config_dir / 'locomotion.yaml'),
                {
                    'yaw_correction_gain': yaw_correction_gain,
                    'odom_topic': odom_topic,
                    'odom_frame_id': odom_frame_id,
                    'base_frame_id': base_frame_id,
                    'publish_odom_tf': publish_odom_tf,
                },
            ]
        ),
        Node(
            package='hexapod_locomotion',
            executable='servo_driver',
            name='servo_driver',
            output='screen',
            parameters=[{
                'dry_run': servo_dry_run,
                'apply_offsets': apply_offsets
            }]
        ),
    ])
