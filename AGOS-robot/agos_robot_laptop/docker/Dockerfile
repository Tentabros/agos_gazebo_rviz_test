# AGOS test robot, laptop side: ROS 2 Lyrical with Nav2, RViz and Zenoh, set
# up to show its windows on the Windows desktop (WSLg) or a Linux desktop.
# No Gazebo and no RTAB-Map: those are practice_agos's (the simulation) and
# the robot's Pi's.
#
# You normally never build this yourself: ./agos_laptop builds it the first
# time it needs it, and ./agos_laptop image rebuilds it after this file
# changes. By hand, from the agos_robot_laptop folder:
#   docker build -t agos-laptop:lyrical docker
#
# The workspace is not copied into the image. ./agos_laptop mounts the
# agos_robot_laptop folder at /ws, so the files you edit are the files the
# container builds and runs.
#
# The ROS 2 version must be the Pi's (agos_robot_pi/docker/Dockerfile):
# Zenoh only connects topics whose message types match exactly.
FROM osrf/ros:lyrical-desktop

ENV DEBIAN_FRONTEND=noninteractive
# dist-upgrade first: the base image's core ROS libraries can be older than
# the newest Nav2, which then crashes with "undefined symbol".
#   navigation2, nav2-bringup  Nav2 and its stock parameters
#   rmw-zenoh-cpp              the link to the Pi
#   image-transport-plugins    the Pi's JPEG camera images in RViz
#   teleop-twist-keyboard      ./agos_laptop teleop
RUN apt-get update && apt-get dist-upgrade -y && apt-get install -y --no-install-recommends \
        ros-lyrical-navigation2 \
        ros-lyrical-nav2-bringup \
        ros-lyrical-rmw-zenoh-cpp \
        ros-lyrical-image-transport-plugins \
        ros-lyrical-teleop-twist-keyboard \
        python3-colcon-common-extensions \
        python3-numpy \
        python3-yaml \
        mesa-utils procps \
    && rm -rf /var/lib/apt/lists/*

# Windows display (WSLg): draw through X11, and use the Windows GPU through
# Direct3D 12 when ./agos_laptop passes it in (/usr/lib/wsl, /dev/dxg).
ENV QT_QPA_PLATFORM=xcb \
    LD_LIBRARY_PATH=/usr/lib/wsl/lib \
    XDG_RUNTIME_DIR=/tmp/runtime-agos

# Every shell in the container gets ROS, the workspace once it has been
# built, and "agos_laptop" as a command (the same as ./agos_laptop outside).
RUN printf '%s\n' \
        '' \
        '# AGOS test robot, laptop side' \
        'mkdir -p -m 700 "$XDG_RUNTIME_DIR"' \
        'source /opt/ros/lyrical/setup.bash' \
        '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' \
        'alias agos_laptop="bash /ws/agos_laptop"' \
        >> /etc/bash.bashrc
