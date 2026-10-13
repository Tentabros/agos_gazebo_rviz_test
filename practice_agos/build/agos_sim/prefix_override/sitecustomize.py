import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/run/media/sazoverse/3228b30c-6f8e-4590-8359-973ebc8d68b0/School/Others/AGOS/Testing/ROS2/agos_gazebo_rviz_test/practice_agos/install/agos_sim'
