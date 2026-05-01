from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share_dir = Path(get_package_share_directory('hexapod_bringup'))
    pose_stack_launch = bringup_share_dir / 'launch' / 'pose_stack.launch.py'

    servo_dry_run = LaunchConfiguration('servo_dry_run')
    launch_lidar = LaunchConfiguration('launch_lidar')
    enable_slam_toolbox = LaunchConfiguration('enable_slam_toolbox')
    scan_topic = LaunchConfiguration('scan_topic')
    use_sim_time = LaunchConfiguration('use_sim_time')
    lidar_serial_port = LaunchConfiguration('lidar_serial_port')
    locomotion_use_imu_for_odom = LaunchConfiguration('locomotion_use_imu_for_odom')

    explorer_enabled = LaunchConfiguration('explorer_enabled')
    explorer_max_speed_mps = LaunchConfiguration('explorer_max_speed_mps')
    explorer_min_speed_mps = LaunchConfiguration('explorer_min_speed_mps')
    explorer_obstacle_stop_distance_m = LaunchConfiguration(
        'explorer_obstacle_stop_distance_m'
    )
    explorer_obstacle_slow_distance_m = LaunchConfiguration(
        'explorer_obstacle_slow_distance_m'
    )
    explorer_desired_clearance_m = LaunchConfiguration('explorer_desired_clearance_m')
    explorer_crab_motion = LaunchConfiguration('explorer_crab_motion')
    explorer_reverse_allowed = LaunchConfiguration('explorer_reverse_allowed')

    return LaunchDescription([
        DeclareLaunchArgument(
            'servo_dry_run',
            default_value='true',
            description='Default true so the first exploration launch is a no-motion test.',
        ),
        DeclareLaunchArgument(
            'launch_lidar',
            default_value='true',
            description='Launch the RPLIDAR driver.',
        ),
        DeclareLaunchArgument(
            'enable_slam_toolbox',
            default_value='true',
            description='Launch slam_toolbox for 2D mapping.',
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
            description='LaserScan topic used by the explorer and slam_toolbox.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if a simulator provides /clock.',
        ),
        DeclareLaunchArgument(
            'lidar_serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial device for the RPLIDAR A1 USB adapter.',
        ),
        DeclareLaunchArgument(
            'locomotion_use_imu_for_odom',
            default_value='false',
            description='Pass through to pose_stack.launch.py.',
        ),
        DeclareLaunchArgument(
            'explorer_enabled',
            default_value='true',
            description='When false, explorer publishes stop commands only.',
        ),
        DeclareLaunchArgument(
            'explorer_max_speed_mps',
            default_value='0.035',
            description='Maximum exploration translation speed.',
        ),
        DeclareLaunchArgument(
            'explorer_min_speed_mps',
            default_value='0.012',
            description='Minimum speed when moving through a narrowing opening.',
        ),
        DeclareLaunchArgument(
            'explorer_obstacle_stop_distance_m',
            default_value='0.30',
            description='Stop/turn if the selected direction is closer than this.',
        ),
        DeclareLaunchArgument(
            'explorer_obstacle_slow_distance_m',
            default_value='0.70',
            description='Start slowing below this selected-direction clearance.',
        ),
        DeclareLaunchArgument(
            'explorer_desired_clearance_m',
            default_value='0.55',
            description='Range threshold used to compute angular clearance.',
        ),
        DeclareLaunchArgument(
            'explorer_crab_motion',
            default_value='true',
            description='Use x/y crab motion toward the chosen LiDAR direction.',
        ),
        DeclareLaunchArgument(
            'explorer_reverse_allowed',
            default_value='false',
            description='Allow the explorer to command negative linear.x.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(pose_stack_launch)),
            launch_arguments={
                'servo_dry_run': servo_dry_run,
                'launch_lidar': launch_lidar,
                'enable_slam_toolbox': enable_slam_toolbox,
                'scan_topic': scan_topic,
                'use_sim_time': use_sim_time,
                'lidar_serial_port': lidar_serial_port,
                'locomotion_use_imu_for_odom': locomotion_use_imu_for_odom,
            }.items(),
        ),
        Node(
            package='hexapod_locomotion',
            executable='lidar_open_space_explorer',
            name='lidar_open_space_explorer',
            output='screen',
            parameters=[{
                'scan_topic': scan_topic,
                'cmd_vel_topic': 'cmd_vel',
                'enabled': explorer_enabled,
                'max_speed_mps': explorer_max_speed_mps,
                'min_speed_mps': explorer_min_speed_mps,
                'obstacle_stop_distance_m': explorer_obstacle_stop_distance_m,
                'obstacle_slow_distance_m': explorer_obstacle_slow_distance_m,
                'desired_clearance_m': explorer_desired_clearance_m,
                'crab_motion': explorer_crab_motion,
                'reverse_allowed': explorer_reverse_allowed,
            }],
        ),
    ])
