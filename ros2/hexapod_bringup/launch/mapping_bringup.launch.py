from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include_lidar_launch(context, *args, **kwargs):
    if LaunchConfiguration('launch_lidar').perform(context).lower() != 'true':
        return []

    lidar_package = LaunchConfiguration('lidar_launch_package').perform(context).strip()
    lidar_launch_file = LaunchConfiguration('lidar_launch_file').perform(context).strip()

    if not lidar_package or not lidar_launch_file:
        return [
            LogInfo(
                msg=(
                    'launch_lidar is true, but lidar_launch_package or lidar_launch_file '
                    'was not provided. Skipping lidar bringup.'
                )
            )
        ]

    try:
        lidar_share_dir = Path(get_package_share_directory(lidar_package))
    except PackageNotFoundError:
        return [
            LogInfo(
                msg=f'Lidar package "{lidar_package}" was not found. Skipping lidar bringup.'
            )
        ]

    lidar_launch_path = lidar_share_dir / 'launch' / lidar_launch_file
    if not lidar_launch_path.exists():
        return [
            LogInfo(
                msg=f'Lidar launch file "{lidar_launch_path}" was not found. Skipping lidar bringup.'
            )
        ]

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(lidar_launch_path)),
        )
    ]


def generate_launch_description():
    locomotion_share_dir = Path(get_package_share_directory('hexapod_locomotion'))
    slam_share_dir = Path(get_package_share_directory('hexapod_slam'))

    core_launch = locomotion_share_dir / 'launch' / 'hexapod_core.launch.py'
    slam_launch = slam_share_dir / 'launch' / 'slam.launch.py'
    default_slam_params = slam_share_dir / 'config' / 'slam_toolbox.yaml'

    use_locomotion = LaunchConfiguration('use_locomotion')
    servo_dry_run = LaunchConfiguration('servo_dry_run')
    apply_offsets = LaunchConfiguration('apply_offsets')
    use_sim_time = LaunchConfiguration('use_sim_time')
    scan_topic = LaunchConfiguration('scan_topic')
    slam_params_file = LaunchConfiguration('slam_params_file')
    map_frame = LaunchConfiguration('map_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_locomotion',
            default_value='true',
            description='Launch the hexapod core stack so odometry is available to SLAM.',
        ),
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
            'launch_lidar',
            default_value='false',
            description='When true, include the external lidar launch file.',
        ),
        DeclareLaunchArgument(
            'lidar_launch_package',
            default_value='',
            description='Package name that contains the lidar launch file.',
        ),
        DeclareLaunchArgument(
            'lidar_launch_file',
            default_value='',
            description='Launch file inside the lidar package launch directory.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use the simulation clock if available.',
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='scan',
            description='LaserScan topic consumed by SLAM.',
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=str(default_slam_params),
            description='Full path to the slam_toolbox parameter file.',
        ),
        DeclareLaunchArgument(
            'map_frame',
            default_value='map',
            description='TF frame published by SLAM for the map.',
        ),
        DeclareLaunchArgument(
            'odom_frame',
            default_value='odom',
            description='TF frame used for local odometry.',
        ),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_link',
            description='Robot base TF frame.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(core_launch)),
            condition=IfCondition(use_locomotion),
            launch_arguments={
                'servo_dry_run': servo_dry_run,
                'apply_offsets': apply_offsets,
            }.items(),
        ),
        OpaqueFunction(function=include_lidar_launch),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(slam_launch)),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'scan_topic': scan_topic,
                'slam_params_file': slam_params_file,
                'map_frame': map_frame,
                'odom_frame': odom_frame,
                'base_frame': base_frame,
            }.items(),
        ),
    ])
