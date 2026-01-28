import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/chairman/working/ros2/learning/chapt4/chapt4_ws/src/install/demo_python_service'
