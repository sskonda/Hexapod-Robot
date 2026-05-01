from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def include_lidar_launch(context, *args, **kwargs):
    if LaunchConfiguration('launch_lidar').perform(context).lower() != 'true':
        return []

    lidar_package = LaunchConfiguration('lidar_launch_package').perform(context).strip()
    lidar_launch_file = LaunchConfiguration('lidar_launch_file').perform(context).strip()

    if not lidar_package or not lidar_launch_file:
        return [
            LogInfo(
                msg=(
                    'launch_lidar is true, but lidar_launch_package or '
                    'lidar_launch_file was not provided. Skipping lidar bringup.'
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
            launch_arguments={
                'channel_type': LaunchConfiguration('lidar_channel_type'),
                'serial_port': LaunchConfiguration('lidar_serial_port'),
                'serial_baudrate': LaunchConfiguration('lidar_serial_baudrate'),
                'frame_id': LaunchConfiguration('laser_frame'),
                'inverted': LaunchConfiguration('lidar_inverted'),
                'angle_compensate': LaunchConfiguration('lidar_angle_compensate'),
                'scan_mode': LaunchConfiguration('lidar_scan_mode'),
            }.items(),
        )
    ]


def generate_launch_description():
    bringup_share_dir = Path(get_package_share_directory('hexapod_bringup'))
    locomotion_share_dir = Path(get_package_share_directory('hexapod_locomotion'))

    core_launch = locomotion_share_dir / 'launch' / 'hexapod_core.launch.py'
    default_slam_params = bringup_share_dir / 'config' / 'slam_toolbox.yaml'

    use_locomotion = LaunchConfiguration('use_locomotion')
    servo_dry_run = LaunchConfiguration('servo_dry_run')
    apply_offsets = LaunchConfiguration('apply_offsets')
    use_sim_time = LaunchConfiguration('use_sim_time')
    scan_topic = LaunchConfiguration('scan_topic')
    slam_params_file = LaunchConfiguration('slam_params_file')
    enable_slam_toolbox = LaunchConfiguration('enable_slam_toolbox')
    autostart_slam_toolbox = LaunchConfiguration('autostart_slam_toolbox')
    map_frame = LaunchConfiguration('map_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')
    laser_frame = LaunchConfiguration('laser_frame')
    laser_x = LaunchConfiguration('laser_x')
    laser_y = LaunchConfiguration('laser_y')
    laser_z = LaunchConfiguration('laser_z')
    laser_roll = LaunchConfiguration('laser_roll')
    laser_pitch = LaunchConfiguration('laser_pitch')
    laser_yaw = LaunchConfiguration('laser_yaw')
    locomotion_odom_topic = LaunchConfiguration('locomotion_odom_topic')
    locomotion_publish_odom_tf = LaunchConfiguration('locomotion_publish_odom_tf')
    locomotion_use_imu_for_odom = LaunchConfiguration('locomotion_use_imu_for_odom')
    imu_frame = LaunchConfiguration('imu_frame')
    imu_use_external_crystal = LaunchConfiguration('imu_use_external_crystal')
    imu_read_retry_count = LaunchConfiguration('imu_read_retry_count')
    imu_retry_backoff_sec = LaunchConfiguration('imu_retry_backoff_sec')
    imu_yaw_filter_time_constant_sec = LaunchConfiguration('imu_yaw_filter_time_constant_sec')
    imu_max_trusted_yaw_covariance_rad2 = LaunchConfiguration(
        'imu_max_trusted_yaw_covariance_rad2'
    )
    imu_startup_still_time_sec = LaunchConfiguration('imu_startup_still_time_sec')
    show_imu_data = LaunchConfiguration('show_imu_data')
    imu_x = LaunchConfiguration('imu_x')
    imu_y = LaunchConfiguration('imu_y')
    imu_z = LaunchConfiguration('imu_z')
    imu_roll = LaunchConfiguration('imu_roll')
    imu_pitch = LaunchConfiguration('imu_pitch')
    imu_yaw = LaunchConfiguration('imu_yaw')

    slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        condition=IfCondition(enable_slam_toolbox),
        parameters=[
            slam_params_file,
            {
                'use_sim_time': use_sim_time,
                'scan_topic': scan_topic,
                'map_frame': map_frame,
                'odom_frame': odom_frame,
                'base_frame': base_frame,
            },
        ],
        remappings=[
            ('scan', scan_topic),
        ],
    )

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
            default_value='true',
            description='When true, include the external 2D lidar launch file.',
        ),
        DeclareLaunchArgument(
            'lidar_launch_package',
            default_value='rplidar_ros',
            description='Package name that contains the lidar launch file.',
        ),
        DeclareLaunchArgument(
            'lidar_launch_file',
            default_value='rplidar_a1_launch.py',
            description='Launch file inside the lidar package launch directory.',
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
            'use_sim_time',
            default_value='false',
            description='Use the simulation clock if available.',
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
            description='LaserScan topic consumed by slam_toolbox.',
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=str(default_slam_params),
            description='Full path to the slam_toolbox parameter file.',
        ),
        DeclareLaunchArgument(
            'enable_slam_toolbox',
            default_value='true',
            description='Launch slam_toolbox for lidar-based 2D mapping.',
        ),
        DeclareLaunchArgument(
            'autostart_slam_toolbox',
            default_value='true',
            description='Automatically configure and activate the slam_toolbox lifecycle node.',
        ),
        DeclareLaunchArgument(
            'map_frame',
            default_value='map',
            description='TF frame published by slam_toolbox for the map.',
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
        DeclareLaunchArgument(
            'laser_frame',
            default_value='laser',
            description='TF frame stamped on lidar scans.',
        ),
        DeclareLaunchArgument(
            'laser_x',
            default_value='0.0',
            description='X offset of the lidar from base_link in metres.',
        ),
        DeclareLaunchArgument(
            'laser_y',
            default_value='0.0',
            description='Y offset of the lidar from base_link in metres.',
        ),
        DeclareLaunchArgument(
            'laser_z',
            default_value='0.0',
            description='Z offset of the lidar from base_link in metres.',
        ),
        DeclareLaunchArgument(
            'laser_roll',
            default_value='0.0',
            description='Roll offset of the lidar from base_link in radians.',
        ),
        DeclareLaunchArgument(
            'laser_pitch',
            default_value='0.0',
            description='Pitch offset of the lidar from base_link in radians.',
        ),
        DeclareLaunchArgument(
            'laser_yaw',
            default_value='-1.5708',
            description='Yaw offset of the lidar from base_link in radians. Default is -90 deg for the current sideways mount.',
        ),
        DeclareLaunchArgument(
            'locomotion_odom_topic',
            default_value='odom',
            description='Odometry topic published by locomotion for slam_toolbox.',
        ),
        DeclareLaunchArgument(
            'locomotion_publish_odom_tf',
            default_value='true',
            description='Whether locomotion publishes odom->base_link TF directly.',
        ),
        DeclareLaunchArgument(
            'locomotion_use_imu_for_odom',
            default_value='false',
            description=(
                'Use trusted BNO055 yaw inside locomotion odometry. Default false '
                'keeps mapping odometry independent of an intermittently trusted IMU.'
            ),
        ),
        DeclareLaunchArgument(
            'imu_frame',
            default_value='imu_link',
            description='Frame id used by the IMU publisher and base_link->imu static TF.',
        ),
        DeclareLaunchArgument(
            'imu_use_external_crystal',
            default_value='false',
            description='Use the BNO055 external crystal.',
        ),
        DeclareLaunchArgument(
            'imu_read_retry_count',
            default_value='3',
            description='How many transient BNO055 UART read retries to allow.',
        ),
        DeclareLaunchArgument(
            'imu_retry_backoff_sec',
            default_value='0.01',
            description='Base retry backoff for transient BNO055 UART read errors.',
        ),
        DeclareLaunchArgument(
            'imu_yaw_filter_time_constant_sec',
            default_value='0.5',
            description='Complementary-filter time constant for the BNO055 yaw estimate.',
        ),
        DeclareLaunchArgument(
            'imu_max_trusted_yaw_covariance_rad2',
            default_value='1.0',
            description='Maximum yaw covariance treated as trusted by locomotion.',
        ),
        DeclareLaunchArgument(
            'imu_startup_still_time_sec',
            default_value='15.0',
            description='Continuous still time required before IMU yaw heading hold activates.',
        ),
        DeclareLaunchArgument(
            'show_imu_data',
            default_value='true',
            description='When false, suppress routine BNO055 IMU status logs while still publishing IMU topics.',
        ),
        DeclareLaunchArgument(
            'imu_x',
            default_value='0.0',
            description='X offset of the IMU from base_link in metres.',
        ),
        DeclareLaunchArgument(
            'imu_y',
            default_value='0.0',
            description='Y offset of the IMU from base_link in metres.',
        ),
        DeclareLaunchArgument(
            'imu_z',
            default_value='0.0',
            description='Z offset of the IMU from base_link in metres.',
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(core_launch)),
            condition=IfCondition(use_locomotion),
            launch_arguments={
                'servo_dry_run': servo_dry_run,
                'apply_offsets': apply_offsets,
                'odom_topic': locomotion_odom_topic,
                'odom_frame_id': odom_frame,
                'base_frame_id': base_frame,
                'publish_odom_tf': locomotion_publish_odom_tf,
                'use_imu_for_odom': locomotion_use_imu_for_odom,
                'imu_frame': imu_frame,
                'imu_use_external_crystal': imu_use_external_crystal,
                'imu_read_retry_count': imu_read_retry_count,
                'imu_retry_backoff_sec': imu_retry_backoff_sec,
                'imu_yaw_filter_time_constant_sec': imu_yaw_filter_time_constant_sec,
                'imu_max_trusted_yaw_covariance_rad2': imu_max_trusted_yaw_covariance_rad2,
                'imu_startup_still_time_sec': imu_startup_still_time_sec,
                'show_imu_data': show_imu_data,
                'imu_x': imu_x,
                'imu_y': imu_y,
                'imu_z': imu_z,
                'imu_roll': imu_roll,
                'imu_pitch': imu_pitch,
                'imu_yaw': imu_yaw,
            }.items(),
        ),
        OpaqueFunction(function=include_lidar_launch),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            output='screen',
            arguments=[
                '--x', laser_x, '--y', laser_y, '--z', laser_z,
                '--roll', laser_roll, '--pitch', laser_pitch, '--yaw', laser_yaw,
                '--frame-id', base_frame, '--child-frame-id', laser_frame,
            ],
        ),
        slam_toolbox_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action=slam_toolbox_node,
                on_start=[
                    EmitEvent(
                        event=ChangeState(
                            lifecycle_node_matcher=matches_action(slam_toolbox_node),
                            transition_id=Transition.TRANSITION_CONFIGURE,
                        ),
                        condition=IfCondition(autostart_slam_toolbox),
                    ),
                ],
            ),
            condition=IfCondition(enable_slam_toolbox),
        ),
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=slam_toolbox_node,
                goal_state='inactive',
                entities=[
                    EmitEvent(
                        event=ChangeState(
                            lifecycle_node_matcher=matches_action(slam_toolbox_node),
                            transition_id=Transition.TRANSITION_ACTIVATE,
                        ),
                        condition=IfCondition(autostart_slam_toolbox),
                    ),
                ],
            ),
            condition=IfCondition(enable_slam_toolbox),
        ),
    ])
