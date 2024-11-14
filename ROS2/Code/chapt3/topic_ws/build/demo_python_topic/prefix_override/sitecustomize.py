import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/cat/WorkSpace/Learning/ROS2/Code/chapt3/topic_ws/install/demo_python_topic'
