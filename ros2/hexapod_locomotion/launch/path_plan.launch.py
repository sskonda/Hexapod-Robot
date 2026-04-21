from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = Path(get_package_share_directory('hexapod_locomotion'))
    core_launch = share_dir / 'launch' / 'hexapod_core.launch.py'

    servo_dry_run = LaunchConfiguration('servo_dry_run')
    apply_offsets = LaunchConfiguration('apply_offsets')
    yaw_correction_gain = LaunchConfiguration('yaw_correction_gain')
    yaw_deadband_deg = LaunchConfiguration('yaw_deadband_deg')
    imu_use_external_crystal = LaunchConfiguration('imu_use_external_crystal')
    imu_read_retry_count = LaunchConfiguration('imu_read_retry_count')
    imu_retry_backoff_sec = LaunchConfiguration('imu_retry_backoff_sec')
    imu_yaw_filter_time_constant_sec = LaunchConfiguration('imu_yaw_filter_time_constant_sec')
    imu_startup_still_time_sec = LaunchConfiguration('imu_startup_still_time_sec')
    imu_startup_motion_grace_sec = LaunchConfiguration('imu_startup_motion_grace_sec')
    wait_for_imu_yaw = LaunchConfiguration('wait_for_imu_yaw')
    imu_topic = LaunchConfiguration('imu_topic')
    publish_rate_hz = LaunchConfiguration('publish_rate_hz')
    startup_delay_sec = LaunchConfiguration('startup_delay_sec')
    forward_distance_m = LaunchConfiguration('forward_distance_m')
    backward_distance_m = LaunchConfiguration('backward_distance_m')
    linear_speed_mps = LaunchConfiguration('linear_speed_mps')
    path_type = LaunchConfiguration('path_type')
    square_side_m = LaunchConfiguration('square_side_m')

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
            default_value='0.1',
            description='IMU heading-hold gain passed through to hexapod_core.launch.py.',
        ),
        DeclareLaunchArgument(
            'yaw_deadband_deg',
            default_value='5.0',
            description='Yaw-error deadband passed through to hexapod_core.launch.py.',
        ),
        DeclareLaunchArgument(
            'imu_use_external_crystal',
            default_value='false',
            description='Pass through to hexapod_core.launch.py for the BNO055 UART bring-up.',
        ),
        DeclareLaunchArgument(
            'imu_read_retry_count',
            default_value='3',
            description='Pass through to hexapod_core.launch.py for transient BNO055 UART read retries.',
        ),
        DeclareLaunchArgument(
            'imu_retry_backoff_sec',
            default_value='0.01',
            description='Pass through to hexapod_core.launch.py for BNO055 UART retry backoff.',
        ),
        DeclareLaunchArgument(
            'imu_yaw_filter_time_constant_sec',
            default_value='0.5',
            description='Pass through to hexapod_core.launch.py for the BNO055 yaw filter.',
        ),
        DeclareLaunchArgument(
            'imu_startup_still_time_sec',
            default_value='15.0',
            description='Pass through to hexapod_core.launch.py for the IMU startup settle period.',
        ),
        DeclareLaunchArgument(
            'imu_startup_motion_grace_sec',
            default_value='0.5',
            description='Pass through to hexapod_core.launch.py for startup stillness motion grace.',
        ),
        DeclareLaunchArgument(
            'wait_for_imu_yaw',
            default_value='true',
            description='Wait for valid IMU yaw before starting the predefined path.',
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/imu/data_raw',
            description='IMU topic monitored by path_plan when wait_for_imu_yaw is true.',
        ),
        DeclareLaunchArgument(
            'publish_rate_hz',
            default_value='20.0',
            description='How often the path planner publishes cmd_vel commands.',
        ),
        DeclareLaunchArgument(
            'startup_delay_sec',
            default_value='20.0',
            description='Delay before the predefined path starts. Keep this longer than the IMU startup settle time.',
        ),
        DeclareLaunchArgument(
            'forward_distance_m',
            default_value='0.5',
            description='Forward distance for the predefined path in meters.',
        ),
        DeclareLaunchArgument(
            'backward_distance_m',
            default_value='0.5',
            description='Backward distance for the predefined path in meters.',
        ),
        DeclareLaunchArgument(
            'linear_speed_mps',
            default_value='0.05',
            description='Linear speed for the predefined path in meters per second.',
        ),
        DeclareLaunchArgument(
            'path_type',
            default_value='square',
            description='Path type: "linear" (forward/backward) or "square" (crab-style square).',
        ),
        DeclareLaunchArgument(
            'square_side_m',
            default_value='0.5',
            description='Side length of the square path in meters (used when path_type=square).',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(core_launch)),
            launch_arguments={
                'servo_dry_run': servo_dry_run,
                'apply_offsets': apply_offsets,
                'yaw_correction_gain': yaw_correction_gain,
                'yaw_deadband_deg': yaw_deadband_deg,
                'imu_use_external_crystal': imu_use_external_crystal,
                'imu_read_retry_count': imu_read_retry_count,
                'imu_retry_backoff_sec': imu_retry_backoff_sec,
                'imu_yaw_filter_time_constant_sec': imu_yaw_filter_time_constant_sec,
                'imu_startup_still_time_sec': imu_startup_still_time_sec,
                'imu_startup_motion_grace_sec': imu_startup_motion_grace_sec,
            }.items(),
        ),
        Node(
            package='hexapod_locomotion',
            executable='path_plan',
            name='path_plan',
            output='screen',
            parameters=[{
                'publish_rate_hz': publish_rate_hz,
                'startup_delay_sec': startup_delay_sec,
                'forward_distance_m': forward_distance_m,
                'backward_distance_m': backward_distance_m,
                'linear_speed_mps': linear_speed_mps,
                'path_type': path_type,
                'square_side_m': square_side_m,
                'wait_for_imu_yaw': wait_for_imu_yaw,
                'imu_topic': imu_topic,
            }],
        ),
    ])
