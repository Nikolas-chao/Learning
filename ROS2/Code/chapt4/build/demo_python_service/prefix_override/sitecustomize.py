import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/cat/WorkSpace/Learning/ROS2/Code/chapt4/install/demo_python_service'
