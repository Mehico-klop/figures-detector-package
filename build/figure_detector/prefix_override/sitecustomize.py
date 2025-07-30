import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/faust/ros2_ws/src/figure_detector/install/figure_detector'
