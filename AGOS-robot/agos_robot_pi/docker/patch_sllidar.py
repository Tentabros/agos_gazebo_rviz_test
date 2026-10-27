"""Patch sllidar_ros2's CMakeLists.txt for ROS 2 Lyrical (see the Dockerfile)."""
import pathlib
import re
import sys

p = pathlib.Path(sys.argv[1])
s = p.read_text().replace('set(CMAKE_CXX_STANDARD 14)', 'set(CMAKE_CXX_STANDARD 17)')
s = re.sub(r'ament_target_dependencies\((\w+)\s+rclcpp\s+std_srvs\s+sensor_msgs\s*\)',
           r'target_link_libraries(\1 rclcpp::rclcpp ${std_srvs_TARGETS} ${sensor_msgs_TARGETS})',
           s)
if 'ament_target_dependencies' in s:
    sys.exit('sllidar_ros2 patch did not apply: CMakeLists.txt has changed upstream')
p.write_text(s)
print('sllidar_ros2 patched for Lyrical')
