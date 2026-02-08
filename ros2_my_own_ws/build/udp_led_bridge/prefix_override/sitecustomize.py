import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rajath/ROS_Workshop/ros2_my_own_ws/install/udp_led_bridge'
