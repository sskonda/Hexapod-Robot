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
    yaw_kp = LaunchConfiguration('yaw_kp')
    yaw_correction_gain = LaunchConfiguration('yaw_correction_gain')
    yaw_ki = LaunchConfiguration('yaw_ki')
    yaw_kd = LaunchConfiguration('yaw_kd')
    yaw_deadband_deg = LaunchConfiguration('yaw_deadband_deg')
    yaw_integrator_limit = LaunchConfiguration('yaw_integrator_limit')
    tripod_planar_travel_scale = LaunchConfiguration('tripod_planar_travel_scale')
    odom_topic = LaunchConfiguration('odom_topic')
    odom_frame_id = LaunchConfiguration('odom_frame_id')
    base_frame_id = LaunchConfiguration('base_frame_id')
    publish_odom_tf = LaunchConfiguration('publish_odom_tf')
    imu_frame = LaunchConfiguration('imu_frame')
    imu_publish_rate_hz = LaunchConfiguration('imu_publish_rate_hz')
    imu_mag_topic = LaunchConfiguration('imu_mag_topic')
    imu_uart_port = LaunchConfiguration('imu_uart_port')
    imu_baud_rate = LaunchConfiguration('imu_baud_rate')
    imu_mode = LaunchConfiguration('imu_mode')
    imu_use_external_crystal = LaunchConfiguration('imu_use_external_crystal')
    imu_read_retry_count = LaunchConfiguration('imu_read_retry_count')
    imu_retry_backoff_sec = LaunchConfiguration('imu_retry_backoff_sec')
    imu_yaw_filter_time_constant_sec = LaunchConfiguration('imu_yaw_filter_time_constant_sec')
    imu_min_mag_calibration_for_yaw = LaunchConfiguration('imu_min_mag_calibration_for_yaw')
    imu_startup_still_time_sec = LaunchConfiguration('imu_startup_still_time_sec')
    imu_startup_motion_grace_sec = LaunchConfiguration('imu_startup_motion_grace_sec')
    imu_allow_mag_baseline_trust_without_calibration = LaunchConfiguration(
        'imu_allow_mag_baseline_trust_without_calibration'
    )
    imu_idle_baseline_mag_axis_tolerance_ut = LaunchConfiguration(
        'imu_idle_baseline_mag_axis_tolerance_ut'
    )
    imu_idle_baseline_min_still_samples = LaunchConfiguration(
        'imu_idle_baseline_min_still_samples'
    )
    bno055_init_retry_period_sec = LaunchConfiguration('bno055_init_retry_period_sec')
    imu_zero_yaw_to_startup_heading = LaunchConfiguration('imu_zero_yaw_to_startup_heading')
    imu_publish_orientation_during_startup = LaunchConfiguration('imu_publish_orientation_during_startup')
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
            'yaw_kp',
            default_value='0.45',
            description='Heading-hold proportional gain used by locomotion when no yaw is commanded.',
        ),
        DeclareLaunchArgument(
            'yaw_correction_gain',
            default_value='0.0',
            description='Legacy alias for yaw_kp. Leave at 0.0 unless an older script sets it.',
        ),
        DeclareLaunchArgument(
            'yaw_ki',
            default_value='0.12',
            description='Heading-hold integral gain used by locomotion when no yaw is commanded.',
        ),
        DeclareLaunchArgument(
            'yaw_kd',
            default_value='0.10',
            description='Heading-hold derivative gain used by locomotion when no yaw is commanded.',
        ),
        DeclareLaunchArgument(
            'yaw_deadband_deg',
            default_value='2.5',
            description='Yaw-error deadband used by locomotion heading hold.',
        ),
        DeclareLaunchArgument(
            'yaw_integrator_limit',
            default_value='1.2',
            description='Absolute limit for the locomotion heading-hold integrator state.',
        ),
        DeclareLaunchArgument(
            'tripod_planar_travel_scale',
            default_value='2.0',
            description='Planar foot-travel multiplier for the tripod gait. Increase slightly if the robot still under-travels.',
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
            'imu_publish_rate_hz',
            default_value='50.0',
            description='BNO055 publish rate in Hz.',
        ),
        DeclareLaunchArgument(
            'imu_mag_topic',
            default_value='/imu/mag',
            description='Magnetometer topic published by the BNO055 driver.',
        ),
        DeclareLaunchArgument(
            'imu_uart_port',
            default_value='/dev/ttyAMA5',
            description='UART device used by the BNO055.',
        ),
        DeclareLaunchArgument(
            'imu_baud_rate',
            default_value='115200',
            description='UART baud rate used by the BNO055.',
        ),
        DeclareLaunchArgument(
            'imu_mode',
            default_value='NDOF_MODE',
            description='BNO055 operating mode. NDOF_MODE publishes the BNO055 fused orientation quaternion.',
        ),
        DeclareLaunchArgument(
            'imu_use_external_crystal',
            default_value='false',
            description=(
                'Enable the BNO055 external crystal. The current robot wiring is '
                'validated with the internal crystal on UART.'
            ),
        ),
        DeclareLaunchArgument(
            'imu_read_retry_count',
            default_value='3',
            description='How many times the BNO055 UART driver retries transient read overruns/timeouts.',
        ),
        DeclareLaunchArgument(
            'imu_retry_backoff_sec',
            default_value='0.01',
            description='Base retry backoff for transient BNO055 UART read errors.',
        ),
        DeclareLaunchArgument(
            'imu_yaw_filter_time_constant_sec',
            default_value='0.5',
            description='Complementary-filter time constant for gyro-smoothed magnetometer yaw.',
        ),
        DeclareLaunchArgument(
            'imu_min_mag_calibration_for_yaw',
            default_value='3',
            description='Minimum BNO055 magnetometer calibration level required for magnetic yaw correction.',
        ),
        DeclareLaunchArgument(
            'imu_startup_still_time_sec',
            default_value='15.0',
            description='Continuous still time required before IMU yaw heading hold becomes active.',
        ),
        DeclareLaunchArgument(
            'imu_startup_motion_grace_sec',
            default_value='0.5',
            description='Continuous motion time that resets the IMU startup stillness timer.',
        ),
        DeclareLaunchArgument(
            'imu_allow_mag_baseline_trust_without_calibration',
            default_value='true',
            description='Allow stable startup magnetic baseline checks to enable yaw correction even when BNO055 mag calibration stays low.',
        ),
        DeclareLaunchArgument(
            'imu_idle_baseline_mag_axis_tolerance_ut',
            default_value='2.0',
            description='Per-axis startup magnetometer tolerance in microtesla used by the low-calibration yaw fallback.',
        ),
        DeclareLaunchArgument(
            'imu_idle_baseline_min_still_samples',
            default_value='25',
            description='Minimum still startup samples required before the low-calibration magnetometer baseline fallback may be accepted.',
        ),
        DeclareLaunchArgument(
            'bno055_init_retry_period_sec',
            default_value='1.0',
            description='How often the BNO055 node retries sensor bring-up after a UART/protocol failure.',
        ),
        DeclareLaunchArgument(
            'imu_zero_yaw_to_startup_heading',
            default_value='true',
            description='Zero IMU yaw to the startup heading so locomotion sees relative rotation from launch.',
        ),
        DeclareLaunchArgument(
            'imu_publish_orientation_during_startup',
            default_value='true',
            description='Publish startup-relative IMU orientation before settle completes so heading hold can react immediately.',
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
            executable='bno055_publisher',
            name='imu_publisher',
            output='screen',
            parameters=[{
                'frame_id': imu_frame,
                'topic': '/imu/data_raw',
                'mag_topic': imu_mag_topic,
                'publish_rate_hz': imu_publish_rate_hz,
                'uart_port': imu_uart_port,
                'baud_rate': imu_baud_rate,
                'mode': imu_mode,
                'use_external_crystal': imu_use_external_crystal,
                'read_retry_count': imu_read_retry_count,
                'retry_backoff_sec': imu_retry_backoff_sec,
                'imu_yaw_filter_time_constant_sec': imu_yaw_filter_time_constant_sec,
                'min_mag_calibration_for_yaw': imu_min_mag_calibration_for_yaw,
                'imu_startup_still_time_sec': imu_startup_still_time_sec,
                'imu_startup_motion_grace_sec': imu_startup_motion_grace_sec,
                'allow_mag_baseline_trust_without_calibration': imu_allow_mag_baseline_trust_without_calibration,
                'idle_baseline_mag_axis_tolerance_ut': imu_idle_baseline_mag_axis_tolerance_ut,
                'idle_baseline_min_still_samples': imu_idle_baseline_min_still_samples,
                'init_retry_period_sec': bno055_init_retry_period_sec,
                'zero_yaw_to_startup_heading': imu_zero_yaw_to_startup_heading,
                'publish_orientation_during_startup': imu_publish_orientation_during_startup,
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
                    'yaw_kp': yaw_kp,
                    'yaw_correction_gain': yaw_correction_gain,
                    'yaw_ki': yaw_ki,
                    'yaw_kd': yaw_kd,
                    'yaw_deadband_deg': yaw_deadband_deg,
                    'yaw_integrator_limit': yaw_integrator_limit,
                    'tripod_planar_travel_scale': tripod_planar_travel_scale,
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
