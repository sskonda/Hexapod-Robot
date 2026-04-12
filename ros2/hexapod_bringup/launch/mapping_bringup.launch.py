from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


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
    default_ekf_params = slam_share_dir / 'config' / 'ekf.yaml'

    use_locomotion = LaunchConfiguration('use_locomotion')
    servo_dry_run = LaunchConfiguration('servo_dry_run')
    apply_offsets = LaunchConfiguration('apply_offsets')
    use_sim_time = LaunchConfiguration('use_sim_time')
    scan_topic = LaunchConfiguration('scan_topic')
    slam_params_file = LaunchConfiguration('slam_params_file')
    map_frame = LaunchConfiguration('map_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')
    enable_explorer = LaunchConfiguration('enable_explorer')
    path_topic = LaunchConfiguration('path_topic')
    stop_point_topic = LaunchConfiguration('stop_point_topic')
    explorer_control_rate_hz = LaunchConfiguration('explorer_control_rate_hz')
    explorer_path_publish_period_sec = LaunchConfiguration('explorer_path_publish_period_sec')
    explorer_decision_pause_sec = LaunchConfiguration('explorer_decision_pause_sec')
    explorer_stop_distance_m = LaunchConfiguration('explorer_stop_distance_m')
    explorer_open_distance_m = LaunchConfiguration('explorer_open_distance_m')
    explorer_goal_backoff_m = LaunchConfiguration('explorer_goal_backoff_m')
    explorer_max_goal_distance_m = LaunchConfiguration('explorer_max_goal_distance_m')
    explorer_min_goal_distance_m = LaunchConfiguration('explorer_min_goal_distance_m')
    explorer_footprint_radius_m = LaunchConfiguration('explorer_footprint_radius_m')
    explorer_wall_clearance_margin_m = LaunchConfiguration('explorer_wall_clearance_margin_m')
    explorer_clearance_window_deg = LaunchConfiguration('explorer_clearance_window_deg')
    explorer_min_gap_width_deg = LaunchConfiguration('explorer_min_gap_width_deg')
    explorer_reverse_avoidance_deg = LaunchConfiguration('explorer_reverse_avoidance_deg')
    explorer_max_replan_attempts = LaunchConfiguration('explorer_max_replan_attempts')
    explorer_forward_bias_weight = LaunchConfiguration('explorer_forward_bias_weight')
    explorer_max_yaw_drift_deg = LaunchConfiguration('explorer_max_yaw_drift_deg')
    explorer_min_progress_m = LaunchConfiguration('explorer_min_progress_m')
    explorer_recovery_backup_m = LaunchConfiguration('explorer_recovery_backup_m')
    crab_follower_speed_mps = LaunchConfiguration('crab_follower_speed_mps')
    crab_follower_goal_tolerance_m = LaunchConfiguration('crab_follower_goal_tolerance_m')
    crab_follower_yaw_correction_gain = LaunchConfiguration('crab_follower_yaw_correction_gain')
    crab_follower_max_angular_speed_rad_s = LaunchConfiguration(
        'crab_follower_max_angular_speed_rad_s'
    )
    crab_follower_yaw_deadband_deg = LaunchConfiguration('crab_follower_yaw_deadband_deg')
    enable_robot_localization = LaunchConfiguration('enable_robot_localization')
    robot_localization_params_file = LaunchConfiguration('robot_localization_params_file')
    raw_odom_topic = LaunchConfiguration('raw_odom_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    laser_frame = LaunchConfiguration('laser_frame')
    laser_x = LaunchConfiguration('laser_x')
    laser_y = LaunchConfiguration('laser_y')
    laser_z = LaunchConfiguration('laser_z')
    laser_roll = LaunchConfiguration('laser_roll')
    laser_pitch = LaunchConfiguration('laser_pitch')
    laser_yaw = LaunchConfiguration('laser_yaw')
    safety_stop_distance_m = LaunchConfiguration('safety_stop_distance_m')
    safety_slowdown_distance_m = LaunchConfiguration('safety_slowdown_distance_m')
    safety_clearance_window_deg = LaunchConfiguration('safety_clearance_window_deg')
    locomotion_odom_topic = LaunchConfiguration('locomotion_odom_topic')
    locomotion_publish_odom_tf = LaunchConfiguration('locomotion_publish_odom_tf')
    imu_frame = LaunchConfiguration('imu_frame')
    imu_x = LaunchConfiguration('imu_x')
    imu_y = LaunchConfiguration('imu_y')
    imu_z = LaunchConfiguration('imu_z')
    imu_roll = LaunchConfiguration('imu_roll')
    imu_pitch = LaunchConfiguration('imu_pitch')
    imu_yaw = LaunchConfiguration('imu_yaw')
    odom_topic = LaunchConfiguration('odom_topic')

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
        DeclareLaunchArgument(
            'enable_explorer',
            default_value='true',
            description='Launch the lidar gap-following exploration node.',
        ),
        DeclareLaunchArgument(
            'path_topic',
            default_value='path',
            description='Path topic published by the exploration node.',
        ),
        DeclareLaunchArgument(
            'stop_point_topic',
            default_value='decision_point',
            description='PointStamped topic published when the robot stops to choose a new direction.',
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='odom',
            description='Odometry topic consumed by the exploration node.',
        ),
        DeclareLaunchArgument(
            'raw_odom_topic',
            default_value='odom/raw',
            description='Raw locomotion odom topic used by robot_localization when enabled.',
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/imu/data_raw',
            description='IMU topic fused by robot_localization when enabled.',
        ),
        DeclareLaunchArgument(
            'enable_robot_localization',
            default_value='false',
            description='Launch robot_localization to fuse raw odom + IMU into odom.',
        ),
        DeclareLaunchArgument(
            'robot_localization_params_file',
            default_value=str(default_ekf_params),
            description='Full path to the robot_localization EKF parameter file.',
        ),
        DeclareLaunchArgument(
            'laser_frame',
            default_value='laser',
            description='TF frame of the LiDAR sensor.',
        ),
        DeclareLaunchArgument(
            'laser_x',
            default_value='0.0',
            description='X offset of the LiDAR from base_link in metres.',
        ),
        DeclareLaunchArgument(
            'laser_y',
            default_value='0.0',
            description='Y offset of the LiDAR from base_link in metres.',
        ),
        DeclareLaunchArgument(
            'laser_z',
            default_value='0.0',
            description='Z offset of the LiDAR from base_link in metres.',
        ),
        DeclareLaunchArgument(
            'laser_roll',
            default_value='0.0',
            description='Roll offset of the LiDAR from base_link in radians.',
        ),
        DeclareLaunchArgument(
            'laser_pitch',
            default_value='0.0',
            description='Pitch offset of the LiDAR from base_link in radians.',
        ),
        DeclareLaunchArgument(
            'laser_yaw',
            default_value='0.0',
            description='Yaw offset of the LiDAR from base_link in radians.',
        ),
        DeclareLaunchArgument(
            'locomotion_odom_topic',
            default_value='odom',
            description='Odometry topic published by locomotion. Use odom/raw when enabling robot_localization.',
        ),
        DeclareLaunchArgument(
            'locomotion_publish_odom_tf',
            default_value='true',
            description='Whether locomotion publishes odom->base_link TF directly.',
        ),
        DeclareLaunchArgument(
            'imu_frame',
            default_value='imu_link',
            description='Frame id used by the IMU publisher and base_link->imu static TF.',
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
        DeclareLaunchArgument(
            'explorer_control_rate_hz',
            default_value='5.0',
            description='Explorer control loop rate in Hz.',
        ),
        DeclareLaunchArgument(
            'explorer_path_publish_period_sec',
            default_value='0.5',
            description='How often the rolling path is refreshed while the robot is moving.',
        ),
        DeclareLaunchArgument(
            'explorer_decision_pause_sec',
            default_value='0.75',
            description='Pause after reaching a wall before picking a new heading.',
        ),
        DeclareLaunchArgument(
            'explorer_stop_distance_m',
            default_value='0.55',
            description='Minimum allowed lidar clearance before stopping and replanning.',
        ),
        DeclareLaunchArgument(
            'explorer_open_distance_m',
            default_value='0.90',
            description='Preferred clearance used to score open gaps.',
        ),
        DeclareLaunchArgument(
            'explorer_goal_backoff_m',
            default_value='0.35',
            description='Distance kept between the rolling goal point and the detected obstacle.',
        ),
        DeclareLaunchArgument(
            'explorer_max_goal_distance_m',
            default_value='1.25',
            description='Maximum distance for the rolling path target.',
        ),
        DeclareLaunchArgument(
            'explorer_min_goal_distance_m',
            default_value='0.20',
            description='Minimum distance for the rolling path target when space is tight.',
        ),
        DeclareLaunchArgument(
            'explorer_footprint_radius_m',
            default_value='0.4572',
            description='Robot circular footprint radius used by the explorer.',
        ),
        DeclareLaunchArgument(
            'explorer_wall_clearance_margin_m',
            default_value='0.10',
            description='Extra wall-clearance buffer used by the explorer.',
        ),
        DeclareLaunchArgument(
            'explorer_clearance_window_deg',
            default_value='12.0',
            description='Half-width of the lidar sector used to evaluate a candidate heading.',
        ),
        DeclareLaunchArgument(
            'explorer_min_gap_width_deg',
            default_value='18.0',
            description='Minimum angular width required for a gap to be considered traversable.',
        ),
        DeclareLaunchArgument(
            'explorer_reverse_avoidance_deg',
            default_value='70.0',
            description='How aggressively the explorer avoids selecting the direction it just came from.',
        ),
        DeclareLaunchArgument(
            'explorer_max_replan_attempts',
            default_value='5',
            description='Number of consecutive obstacle stops before declaring a dead end and halting.',
        ),
        DeclareLaunchArgument(
            'explorer_forward_bias_weight',
            default_value='1.5',
            description='Bias toward forward-facing gaps when selecting a heading.',
        ),
        DeclareLaunchArgument(
            'explorer_max_yaw_drift_deg',
            default_value='8.0',
            description='Maximum odom yaw drift allowed before the explorer forces a replan.',
        ),
        DeclareLaunchArgument(
            'explorer_min_progress_m',
            default_value='0.15',
            description='Minimum travel that counts as progress before a stop.',
        ),
        DeclareLaunchArgument(
            'explorer_recovery_backup_m',
            default_value='0.30',
            description='Recovery backup distance used when repeated low-progress stops occur.',
        ),
        DeclareLaunchArgument(
            'crab_follower_speed_mps',
            default_value='0.04',
            description='Constant travel speed for the crab path follower in m/s.',
        ),
        DeclareLaunchArgument(
            'crab_follower_goal_tolerance_m',
            default_value='0.08',
            description='Distance from the rolling goal at which the follower considers itself arrived.',
        ),
        DeclareLaunchArgument(
            'crab_follower_yaw_correction_gain',
            default_value='1.5',
            description='Proportional yaw correction gain used by the crab path follower.',
        ),
        DeclareLaunchArgument(
            'crab_follower_max_angular_speed_rad_s',
            default_value='0.35',
            description='Maximum yaw-rate correction commanded by the crab path follower.',
        ),
        DeclareLaunchArgument(
            'crab_follower_yaw_deadband_deg',
            default_value='3.0',
            description='Yaw-error deadband for the crab path follower.',
        ),
        DeclareLaunchArgument(
            'safety_stop_distance_m',
            default_value='0.72',
            description='Emergency-stop clearance used by the scan cmd_vel safety filter.',
        ),
        DeclareLaunchArgument(
            'safety_slowdown_distance_m',
            default_value='0.90',
            description='Slowdown clearance used by the scan cmd_vel safety filter.',
        ),
        DeclareLaunchArgument(
            'safety_clearance_window_deg',
            default_value='20.0',
            description='LiDAR sector half-width used by the scan cmd_vel safety filter.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(core_launch)),
            condition=IfCondition(use_locomotion),
            launch_arguments={
                'servo_dry_run': servo_dry_run,
                'apply_offsets': apply_offsets,
                'odom_topic': PythonExpression([
                    "'", raw_odom_topic, "' if '", enable_robot_localization,
                    "' == 'true' else '", locomotion_odom_topic, "'",
                ]),
                'odom_frame_id': odom_frame,
                'base_frame_id': base_frame,
                'publish_odom_tf': PythonExpression([
                    "'false' if '", enable_robot_localization,
                    "' == 'true' else '", locomotion_publish_odom_tf, "'",
                ]),
                'imu_frame': imu_frame,
                'imu_x': imu_x,
                'imu_y': imu_y,
                'imu_z': imu_z,
                'imu_roll': imu_roll,
                'imu_pitch': imu_pitch,
                'imu_yaw': imu_yaw,
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
                'enable_explorer': enable_explorer,
                'enable_robot_localization': enable_robot_localization,
                'robot_localization_params_file': robot_localization_params_file,
                'laser_frame': laser_frame,
                'laser_x': laser_x,
                'laser_y': laser_y,
                'laser_z': laser_z,
                'laser_roll': laser_roll,
                'laser_pitch': laser_pitch,
                'laser_yaw': laser_yaw,
                'path_topic': path_topic,
                'stop_point_topic': stop_point_topic,
                'odom_topic': odom_topic,
                'raw_odom_topic': raw_odom_topic,
                'imu_topic': imu_topic,
                'explorer_control_rate_hz': explorer_control_rate_hz,
                'explorer_path_publish_period_sec': explorer_path_publish_period_sec,
                'explorer_decision_pause_sec': explorer_decision_pause_sec,
                'explorer_stop_distance_m': explorer_stop_distance_m,
                'explorer_open_distance_m': explorer_open_distance_m,
                'explorer_goal_backoff_m': explorer_goal_backoff_m,
                'explorer_max_goal_distance_m': explorer_max_goal_distance_m,
                'explorer_min_goal_distance_m': explorer_min_goal_distance_m,
                'explorer_footprint_radius_m': explorer_footprint_radius_m,
                'explorer_wall_clearance_margin_m': explorer_wall_clearance_margin_m,
                'explorer_clearance_window_deg': explorer_clearance_window_deg,
                'explorer_min_gap_width_deg': explorer_min_gap_width_deg,
                'explorer_reverse_avoidance_deg': explorer_reverse_avoidance_deg,
                'explorer_max_replan_attempts': explorer_max_replan_attempts,
                'explorer_forward_bias_weight': explorer_forward_bias_weight,
                'explorer_max_yaw_drift_deg': explorer_max_yaw_drift_deg,
                'explorer_min_progress_m': explorer_min_progress_m,
                'explorer_recovery_backup_m': explorer_recovery_backup_m,
                'crab_follower_speed_mps': crab_follower_speed_mps,
                'crab_follower_goal_tolerance_m': crab_follower_goal_tolerance_m,
                'crab_follower_yaw_correction_gain': crab_follower_yaw_correction_gain,
                'crab_follower_max_angular_speed_rad_s': crab_follower_max_angular_speed_rad_s,
                'crab_follower_yaw_deadband_deg': crab_follower_yaw_deadband_deg,
                'safety_stop_distance_m': safety_stop_distance_m,
                'safety_slowdown_distance_m': safety_slowdown_distance_m,
                'safety_clearance_window_deg': safety_clearance_window_deg,
            }.items(),
        ),
    ])
