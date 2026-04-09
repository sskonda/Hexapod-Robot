import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sanat/Hexapod-Robot/ros2/install/hexapod_bringup'
