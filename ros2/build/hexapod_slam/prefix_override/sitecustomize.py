import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/snail/ros2_ws/src/Hexapod-Robot/ros2/install/hexapod_slam'
