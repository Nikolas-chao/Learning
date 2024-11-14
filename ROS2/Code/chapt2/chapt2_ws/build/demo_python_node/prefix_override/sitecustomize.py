import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/cat/WorkSpace/Code/ROS2/chapt2/chapt2_ws/install/demo_python_node'
