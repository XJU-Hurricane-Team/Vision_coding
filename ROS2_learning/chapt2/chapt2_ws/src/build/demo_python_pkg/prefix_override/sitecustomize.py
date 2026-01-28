import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/chairman/working/ros2/learning/chapt2/chapt2_ws/src/install/demo_python_pkg'
