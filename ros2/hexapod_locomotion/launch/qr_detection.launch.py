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
    seen_text_topic = LaunchConfiguration('seen_text_topic')
    output_image_topic = LaunchConfiguration('output_image_topic')
    republish_same_text = LaunchConfiguration('republish_same_text')
    publish_annotated_image = LaunchConfiguration('publish_annotated_image')
    processing_fps = LaunchConfiguration('processing_fps')
    output_image_width = LaunchConfiguration('output_image_width')
    output_image_height = LaunchConfiguration('output_image_height')
    output_image_grayscale = LaunchConfiguration('output_image_grayscale')

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
            default_value='yuyv2rgb',
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
            default_value='10.0',
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
            'seen_text_topic',
            default_value='',
            description='Optional topic where every visible QR text frame is published.',
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
        DeclareLaunchArgument(
            'publish_annotated_image',
            default_value='false',
            description='Publish annotated QR debug images.',
        ),
        DeclareLaunchArgument(
            'processing_fps',
            default_value='10.0',
            description='Maximum QR processing rate.',
        ),
        DeclareLaunchArgument(
            'output_image_width',
            default_value='320',
            description='Width of the published QR debug image.',
        ),
        DeclareLaunchArgument(
            'output_image_height',
            default_value='240',
            description='Height of the published QR debug image.',
        ),
        DeclareLaunchArgument(
            'output_image_grayscale',
            default_value='true',
            description='Publish the QR debug image as grayscale to reduce bandwidth.',
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
                'seen_text_topic': seen_text_topic,
                'annotated_image_topic': output_image_topic,
                'publish_annotated_image': ParameterValue(
                    publish_annotated_image,
                    value_type=bool,
                ),
                'republish_same_text': ParameterValue(
                    republish_same_text,
                    value_type=bool,
                ),
                'processing_fps': ParameterValue(
                    processing_fps,
                    value_type=float,
                ),
                'output_image_width': ParameterValue(
                    output_image_width,
                    value_type=int,
                ),
                'output_image_height': ParameterValue(
                    output_image_height,
                    value_type=int,
                ),
                'output_image_grayscale': ParameterValue(
                    output_image_grayscale,
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
