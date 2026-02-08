import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rajath/ROS_Workshop/ros2_keyboar_led_control/install/keyboard_node'
