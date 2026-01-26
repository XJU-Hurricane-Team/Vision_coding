import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/chairman/working/ros2/ROS2_learning/chapt7/chapt7_ws/install/autopatrol_robot'
