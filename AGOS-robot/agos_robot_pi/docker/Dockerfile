# AGOS test robot on a Raspberry Pi 4: the sensor drivers, LiDAR odometry,
# RTAB-Map and Zenoh. Nav2, the explorer and RViz stay on the laptop. The
# micro-ROS agent for the ESP32 runs in its own image
# (microros/micro-ros-agent, started by ./agos_robot).
#
# You normally never build this yourself: ./agos_robot start builds it the
# first time it needs it, and ./agos_robot image rebuilds it. By hand, in the
# agos_robot_pi folder on the Pi:
#   docker build -t agos-robot:lyrical docker
#
# The ROS 2 version must be the laptop's (Lyrical in agos_robot_laptop): Zenoh
# only connects topics whose message types match exactly.
#
# ros:<distro>-ros-base is an official image for 64-bit ARM, so this builds
# on the Pi itself; it takes 20-40 minutes on a Pi 4.
ARG ROS_DISTRO=lyrical
FROM ros:${ROS_DISTRO}-ros-base
ARG ROS_DISTRO

ENV DEBIAN_FRONTEND=noninteractive
# dist-upgrade first: the base image's core ROS libraries can be older than
# the newest RTAB-Map, which then crashes with "undefined symbol".
#   orbbec-camera          Gemini 335 driver (Orbbec's ROS 2 wrapper, released for Lyrical)
#   rtabmap-*              SLAM, LiDAR odometry, RGB-D sync
#   imu-filter-madgwick    orientation for the camera's IMU
#   rmw-zenoh-cpp          the middleware AGOS uses between computers
#   rmw-fastrtps-cpp       the middleware the micro-ROS agent speaks (uros_relay.py)
#   image-transport-plugins  JPEG images for RViz on the laptop
RUN apt-get update && apt-get dist-upgrade -y && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-orbbec-camera \
        ros-${ROS_DISTRO}-rtabmap-slam \
        ros-${ROS_DISTRO}-rtabmap-odom \
        ros-${ROS_DISTRO}-rtabmap-sync \
        ros-${ROS_DISTRO}-imu-filter-madgwick \
        ros-${ROS_DISTRO}-rmw-zenoh-cpp \
        ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
        ros-${ROS_DISTRO}-robot-state-publisher \
        ros-${ROS_DISTRO}-image-transport-plugins \
        python3-colcon-common-extensions python3-yaml git usbutils procps \
    && rm -rf /var/lib/apt/lists/*

# Slamtec's C1 driver, sllidar_ros2 (Lyrical's rplidar-ros is another fork,
# without the C1). It calls ament_target_dependencies, which Lyrical's
# ament_cmake removed: patch its CMakeLists.txt, then build.
COPY patch_sllidar.py /tmp/patch_sllidar.py
RUN mkdir -p /opt/agos_ws/src && cd /opt/agos_ws/src \
    && git clone --depth 1 https://github.com/Slamtec/sllidar_ros2.git \
    && python3 /tmp/patch_sllidar.py sllidar_ros2/CMakeLists.txt
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && cd /opt/agos_ws \
    && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log

# Every shell in the container gets ROS and the C1 driver, talking through Zenoh
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
RUN printf '%s\n' '' '# AGOS test robot' "source /opt/ros/${ROS_DISTRO}/setup.bash" \
        'source /opt/agos_ws/install/setup.bash' >> /etc/bash.bashrc
