from glob import glob

from setuptools import find_packages, setup

package_name = 'hexapod_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sanat Konda',
    maintainer_email='sskonda04@gmail.com',
    description='SLAM launch files and configuration for the hexapod robot',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gap_following_explorer = hexapod_slam.gap_following_explorer:main',
            'maze_graph_builder = hexapod_slam.maze_graph_builder:main',
            'maze_graph_planner = hexapod_slam.maze_graph_planner:main',
            'maze_graph_visualizer = hexapod_slam.maze_graph_visualizer:main',
            'path_to_local_goal = hexapod_slam.path_to_local_goal:main',
            'maze_mission = hexapod_slam.maze_mission:main',
        ],
    },
)
