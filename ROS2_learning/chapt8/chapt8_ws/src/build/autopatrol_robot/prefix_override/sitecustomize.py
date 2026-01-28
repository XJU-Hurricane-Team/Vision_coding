import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/chairman/working/ros2/learning/chapt8/chapt8_ws/src/install/autopatrol_robot'
