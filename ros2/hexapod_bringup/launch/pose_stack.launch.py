from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share_dir = Path(get_package_share_directory('hexapod_bringup'))
    mapping_bringup_launch = bringup_share_dir / 'launch' / 'mapping_bringup.launch.py'

    launch_lidar = LaunchConfiguration('launch_lidar')
    lidar_launch_package = LaunchConfiguration('lidar_launch_package')
    lidar_launch_file = LaunchConfiguration('lidar_launch_file')
    lidar_channel_type = LaunchConfiguration('lidar_channel_type')
    lidar_serial_port = LaunchConfiguration('lidar_serial_port')
    lidar_serial_baudrate = LaunchConfiguration('lidar_serial_baudrate')
    lidar_inverted = LaunchConfiguration('lidar_inverted')
    lidar_angle_compensate = LaunchConfiguration('lidar_angle_compensate')
    lidar_scan_mode = LaunchConfiguration('lidar_scan_mode')
    scan_topic = LaunchConfiguration('scan_topic')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_slam_toolbox = LaunchConfiguration('enable_slam_toolbox')
    servo_dry_run = LaunchConfiguration('servo_dry_run')
    apply_offsets = LaunchConfiguration('apply_offsets')
    locomotion_use_imu_for_odom = LaunchConfiguration('locomotion_use_imu_for_odom')
    laser_frame = LaunchConfiguration('laser_frame')
    laser_x = LaunchConfiguration('laser_x')
    laser_y = LaunchConfiguration('laser_y')
    laser_z = LaunchConfiguration('laser_z')
    laser_roll = LaunchConfiguration('laser_roll')
    laser_pitch = LaunchConfiguration('laser_pitch')
    laser_yaw = LaunchConfiguration('laser_yaw')
    imu_frame = LaunchConfiguration('imu_frame')
    imu_x = LaunchConfiguration('imu_x')
    imu_y = LaunchConfiguration('imu_y')
    imu_z = LaunchConfiguration('imu_z')
    imu_roll = LaunchConfiguration('imu_roll')
    imu_pitch = LaunchConfiguration('imu_pitch')
    imu_yaw = LaunchConfiguration('imu_yaw')
    imu_startup_still_time_sec = LaunchConfiguration('imu_startup_still_time_sec')

    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_lidar',
            default_value='true',
            description=(
                'When true, launch the RPLIDAR A1 driver before starting SLAM. '
                'Set false only if scan is already being published elsewhere.'
            ),
        ),
        DeclareLaunchArgument(
            'lidar_launch_package',
            default_value='rplidar_ros',
            description='Package containing the RPLIDAR A1 launch file.',
        ),
        DeclareLaunchArgument(
            'lidar_launch_file',
            default_value='rplidar_a1_launch.py',
            description='RPLIDAR A1 launch file inside the lidar package launch directory.',
        ),
        DeclareLaunchArgument(
            'lidar_channel_type',
            default_value='serial',
            description='RPLIDAR transport channel type.',
        ),
        DeclareLaunchArgument(
            'lidar_serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial device for the RPLIDAR A1 USB adapter.',
        ),
        DeclareLaunchArgument(
            'lidar_serial_baudrate',
            default_value='115200',
            description='RPLIDAR A1 serial baud rate.',
        ),
        DeclareLaunchArgument(
            'lidar_inverted',
            default_value='false',
            description='Invert RPLIDAR scan direction.',
        ),
        DeclareLaunchArgument(
            'lidar_angle_compensate',
            default_value='true',
            description='Enable RPLIDAR angle compensation.',
        ),
        DeclareLaunchArgument(
            'lidar_scan_mode',
            default_value='Sensitivity',
            description='RPLIDAR scan mode passed to rplidar_ros.',
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
            description='LaserScan topic used by slam_toolbox.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if a simulator is providing /clock.',
        ),
        DeclareLaunchArgument(
            'enable_slam_toolbox',
            default_value='true',
            description='Launch slam_toolbox for lidar mapping.',
        ),
        DeclareLaunchArgument(
            'servo_dry_run',
            default_value='false',
            description='When true, log servo targets without driving hardware.',
        ),
        DeclareLaunchArgument(
            'apply_offsets',
            default_value='true',
            description='Apply saved servo calibration offsets.',
        ),
        DeclareLaunchArgument(
            'locomotion_use_imu_for_odom',
            default_value='false',
            description=(
                'Use BNO055 yaw in locomotion odometry only when the IMU is trusted. '
                'The default maps with gait odometry plus lidar scan matching.'
            ),
        ),
        DeclareLaunchArgument(
            'laser_frame',
            default_value='laser',
            description='LiDAR TF frame.',
        ),
        DeclareLaunchArgument(
            'laser_x',
            default_value='0.0',
            description='LiDAR x offset from base_link in metres.',
        ),
        DeclareLaunchArgument(
            'laser_y',
            default_value='0.0',
            description='LiDAR y offset from base_link in metres.',
        ),
        DeclareLaunchArgument(
            'laser_z',
            default_value='0.0',
            description='LiDAR z offset from base_link in metres.',
        ),
        DeclareLaunchArgument(
            'laser_roll',
            default_value='0.0',
            description='LiDAR roll offset from base_link in radians.',
        ),
        DeclareLaunchArgument(
            'laser_pitch',
            default_value='0.0',
            description='LiDAR pitch offset from base_link in radians.',
        ),
        DeclareLaunchArgument(
            'laser_yaw',
            default_value='-1.5708',
            description='LiDAR yaw offset from base_link in radians. Default is -90 deg for the current sideways mount.',
        ),
        DeclareLaunchArgument(
            'imu_frame',
            default_value='imu_link',
            description='IMU TF frame.',
        ),
        DeclareLaunchArgument(
            'imu_x',
            default_value='0.0',
            description='IMU x offset from base_link in metres.',
        ),
        DeclareLaunchArgument(
            'imu_y',
            default_value='0.0',
            description='IMU y offset from base_link in metres.',
        ),
        DeclareLaunchArgument(
            'imu_z',
            default_value='0.0',
            description='IMU z offset from base_link in metres.',
        ),
        DeclareLaunchArgument(
            'imu_roll',
            default_value='0.0',
            description='IMU roll offset from base_link in radians.',
        ),
        DeclareLaunchArgument(
            'imu_pitch',
            default_value='0.0',
            description='IMU pitch offset from base_link in radians.',
        ),
        DeclareLaunchArgument(
            'imu_yaw',
            default_value='0.0',
            description='IMU yaw offset from base_link in radians.',
        ),
        DeclareLaunchArgument(
            'imu_startup_still_time_sec',
            default_value='15.0',
            description='Still time required before BNO055 yaw is trusted.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(mapping_bringup_launch)),
            launch_arguments={
                'use_locomotion': 'true',
                'enable_slam_toolbox': enable_slam_toolbox,
                'locomotion_odom_topic': 'odom',
                'locomotion_publish_odom_tf': 'true',
                'locomotion_use_imu_for_odom': locomotion_use_imu_for_odom,
                'launch_lidar': launch_lidar,
                'lidar_launch_package': lidar_launch_package,
                'lidar_launch_file': lidar_launch_file,
                'lidar_channel_type': lidar_channel_type,
                'lidar_serial_port': lidar_serial_port,
                'lidar_serial_baudrate': lidar_serial_baudrate,
                'lidar_inverted': lidar_inverted,
                'lidar_angle_compensate': lidar_angle_compensate,
                'lidar_scan_mode': lidar_scan_mode,
                'scan_topic': scan_topic,
                'use_sim_time': use_sim_time,
                'servo_dry_run': servo_dry_run,
                'apply_offsets': apply_offsets,
                'laser_frame': laser_frame,
                'laser_x': laser_x,
                'laser_y': laser_y,
                'laser_z': laser_z,
                'laser_roll': laser_roll,
                'laser_pitch': laser_pitch,
                'laser_yaw': laser_yaw,
                'imu_frame': imu_frame,
                'imu_x': imu_x,
                'imu_y': imu_y,
                'imu_z': imu_z,
                'imu_roll': imu_roll,
                'imu_pitch': imu_pitch,
                'imu_yaw': imu_yaw,
                'imu_startup_still_time_sec': imu_startup_still_time_sec,
            }.items(),
        ),
    ])
