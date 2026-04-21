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
    output_image_topic = LaunchConfiguration('output_image_topic')
    bounding_box_topic = LaunchConfiguration('bounding_box_topic')
    scale_factor = LaunchConfiguration('scale_factor')
    min_neighbors = LaunchConfiguration('min_neighbors')
    min_size_pixels = LaunchConfiguration('min_size_pixels')

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_camera',
            default_value='true',
            description='Start usb_cam along with the face detector.',
        ),
        DeclareLaunchArgument(
            'show_rqt',
            default_value='false',
            description='Start rqt_image_view on the annotated image topic.',
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
            description='Input image topic for face detection.',
        ),
        DeclareLaunchArgument(
            'output_image_topic',
            default_value='/face_detection/image',
            description='Annotated image topic for rqt_image_view.',
        ),
        DeclareLaunchArgument(
            'bounding_box_topic',
            default_value='/face_detection/bounding_box',
            description='RegionOfInterest topic for the largest detected face.',
        ),
        DeclareLaunchArgument(
            'scale_factor',
            default_value='1.1',
            description='OpenCV Haar cascade scale factor.',
        ),
        DeclareLaunchArgument(
            'min_neighbors',
            default_value='5',
            description='OpenCV Haar cascade minimum neighbors.',
        ),
        DeclareLaunchArgument(
            'min_size_pixels',
            default_value='30',
            description='Minimum face width and height in pixels.',
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
            executable='face_detector',
            name='face_detector',
            output='screen',
            parameters=[{
                'image_topic': image_topic,
                'output_image_topic': output_image_topic,
                'bounding_box_topic': bounding_box_topic,
                'scale_factor': ParameterValue(scale_factor, value_type=float),
                'min_neighbors': ParameterValue(min_neighbors, value_type=int),
                'min_size_pixels': ParameterValue(min_size_pixels, value_type=int),
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
