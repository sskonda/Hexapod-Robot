from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    start_camera = LaunchConfiguration('start_camera')
    show_rqt = LaunchConfiguration('show_rqt')
    video_device = LaunchConfiguration('video_device')
    pixel_format = LaunchConfiguration('pixel_format')
    image_width = LaunchConfiguration('image_width')
    image_height = LaunchConfiguration('image_height')
    framerate = LaunchConfiguration('framerate')
    image_topic = LaunchConfiguration('image_topic')
    text_topic = LaunchConfiguration('text_topic')
    output_image_topic = LaunchConfiguration('output_image_topic')
    republish_same_text = LaunchConfiguration('republish_same_text')

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_camera',
            default_value='true',
            description='Start usb_cam along with the QR detector.',
        ),
        DeclareLaunchArgument(
            'show_rqt',
            default_value='false',
            description='Start rqt_image_view on the annotated QR image topic.',
        ),
        DeclareLaunchArgument(
            'video_device',
            default_value='/dev/video0',
            description='Camera device passed to usb_cam.',
        ),
        DeclareLaunchArgument(
            'pixel_format',
            default_value='mjpeg2rgb',
            description='Pixel format passed to usb_cam.',
        ),
        DeclareLaunchArgument(
            'image_width',
            default_value='640',
            description='Camera image width.',
        ),
        DeclareLaunchArgument(
            'image_height',
            default_value='480',
            description='Camera image height.',
        ),
        DeclareLaunchArgument(
            'framerate',
            default_value='30.0',
            description='Camera frame rate.',
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/image_raw',
            description='Input image topic for QR detection.',
        ),
        DeclareLaunchArgument(
            'text_topic',
            default_value='/qr_code/text',
            description='String topic where decoded QR text is published.',
        ),
        DeclareLaunchArgument(
            'output_image_topic',
            default_value='/qr_code/image',
            description='Annotated image topic for QR visualization.',
        ),
        DeclareLaunchArgument(
            'republish_same_text',
            default_value='false',
            description='When true, publish repeated detections of the same QR text.',
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            condition=IfCondition(start_camera),
            parameters=[{
                'video_device': video_device,
                'pixel_format': pixel_format,
                'image_width': ParameterValue(image_width, value_type=int),
                'image_height': ParameterValue(image_height, value_type=int),
                'framerate': ParameterValue(framerate, value_type=float),
            }],
        ),
        Node(
            package='hexapod_locomotion',
            executable='qr_code_detector',
            name='qr_code_detector',
            output='screen',
            parameters=[{
                'image_topic': image_topic,
                'text_topic': text_topic,
                'annotated_image_topic': output_image_topic,
                'publish_annotated_image': True,
                'republish_same_text': ParameterValue(
                    republish_same_text,
                    value_type=bool,
                ),
            }],
        ),
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='rqt_image_view',
            output='screen',
            condition=IfCondition(show_rqt),
            arguments=[output_image_topic],
        ),
    ])
