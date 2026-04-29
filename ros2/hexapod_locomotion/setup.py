from setuptools import find_packages, setup
from glob import glob

package_name = 'hexapod_locomotion'

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
    description='Hexapod locomotion, IMU, calibration, and servo driver nodes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_publisher = hexapod_locomotion.imu_publisher:main',
            'bno055_publisher = hexapod_locomotion.bno055_publisher:main',
            'mpu6050_publisher = hexapod_locomotion.mpu6050_publisher:main',
            'locomotion = hexapod_locomotion.locomotion:main',
            'crab_path_follower = hexapod_locomotion.crab_path_follower:main',
            'path_plan = hexapod_locomotion.path_plan:main',
            'servo_driver = hexapod_locomotion.servo_driver:main',
            'calibration = hexapod_locomotion.calibration:main',
            'calibration_matrix = hexapod_locomotion.calibration_matrix:main',
            'calibration_points = hexapod_locomotion.calibration_points:main',
        ],
    },
)
