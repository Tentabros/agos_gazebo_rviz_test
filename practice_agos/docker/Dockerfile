# AGOS simulation image: ROS 2 Lyrical, Gazebo Jetty, Nav2, RTAB-Map and
# robot_localization, set up to show its windows on the Windows desktop (WSLg).
#
# Build it from the practice_agos folder:
#   docker build -t agos-sim:lyrical docker
#
# The workspace is not copied into the image. docker/agos.sh mounts the
# practice_agos folder at /ws, so files you edit on Windows or in WSL are the
# files the container builds and runs.
FROM osrf/ros:lyrical-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-lyrical-navigation2 \
        ros-lyrical-nav2-bringup \
        ros-lyrical-rtabmap-ros \
        ros-lyrical-robot-localization \
        ros-lyrical-ros-gz \
        ros-lyrical-xacro \
        ros-lyrical-joint-state-publisher-gui \
        ros-lyrical-teleop-twist-keyboard \
        python3-colcon-common-extensions \
        mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Windows display (WSLg): draw through X11, and use the Windows GPU through
# Direct3D 12 when docker/agos.sh passes it in (/usr/lib/wsl, /dev/dxg).
ENV QT_QPA_PLATFORM=xcb \
    LD_LIBRARY_PATH=/usr/lib/wsl/lib \
    XDG_RUNTIME_DIR=/tmp/runtime-agos

# Every shell, including each extra terminal docker/agos.sh opens, gets ROS
# and, once it has been built, the AGOS workspace.
RUN printf '%s\n' \
        '' \
        '# AGOS simulation' \
        'mkdir -p -m 700 "$XDG_RUNTIME_DIR"' \
        'source /opt/ros/lyrical/setup.bash' \
        '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' \
        >> /etc/bash.bashrc
