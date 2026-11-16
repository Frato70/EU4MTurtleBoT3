ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

SHELL ["/bin/bash", "-c"]

ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}
ENV TURTLEBOT3_MODEL=burger
ENV TERM=xterm-256color

# ============================================================
# ROS 2 DESKTOP + DEVELOPMENT
# ============================================================
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-rmw-zenoh-cpp \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    build-essential \
    cmake \
    git \
    nano \
    vim \
    tree \
    bash-completion \
    iputils-ping \
    net-tools \
    iproute2 \
    netcat-openbsd \
    && rm -rf /var/lib/apt/lists/*

# ============================================================
# GAZEBO SIM + ROS <-> GAZEBO
# ============================================================
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-ros-gz \
    && rm -rf /var/lib/apt/lists/*

# ============================================================
# TURTLEBOT3
# ============================================================
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-turtlebot3 \
    ros-${ROS_DISTRO}-turtlebot3-simulations \
    && rm -rf /var/lib/apt/lists/*

# ============================================================
# SLAM
# ============================================================
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-slam-toolbox \
    && rm -rf /var/lib/apt/lists/*

# ============================================================
# NAVIGATION 2
# ============================================================
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*

# ============================================================
# BEHAVIOR TREES
# ============================================================
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-behaviortree-cpp \
    && rm -rf /var/lib/apt/lists/*

# ============================================================
# MOVEIT 2 / MANIPULATION
# ============================================================
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-setup-assistant \
    && rm -rf /var/lib/apt/lists/*

# ============================================================
# ROBOT / TF / LOCALIZATION / TELEOP TOOLS
# ============================================================
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

# ============================================================
# SHELL CONFIGURATION
# ============================================================
RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /root/.bashrc && \
    echo 'if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi' >> /root/.bashrc && \
    echo 'export TURTLEBOT3_MODEL=burger' >> /root/.bashrc && \
    echo 'export TERM=xterm-256color' >> /root/.bashrc && \
    echo "alias ls='ls --color=auto'" >> /root/.bashrc && \
    echo "alias grep='grep --color=auto'" >> /root/.bashrc && \
    echo "export PS1='\[\033[1;32m\][ROS2-Jazzy]\[\033[0m\] \[\033[1;36m\]\w\[\033[0m\] \[\033[1;33m\]#\[\033[0m\] '" >> /root/.bashrc

WORKDIR /ros2_ws

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
