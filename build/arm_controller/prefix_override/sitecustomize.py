import sys
if sys.prefix == '/home/antonia/miniforge3/envs/ros_env':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/antonia/robot_arm_ws/install/arm_controller'
