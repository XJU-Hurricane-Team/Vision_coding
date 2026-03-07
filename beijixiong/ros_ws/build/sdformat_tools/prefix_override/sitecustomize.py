import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/chairman/working/ros2/beijixiong/ros_ws/install/sdformat_tools'
