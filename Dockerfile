FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV COLCON_WS=/root/turtlebot3_ws
ARG ROS_DISTRO=humble

WORKDIR ${COLCON_WS}

RUN mkdir -p ${COLCON_WS}/src
COPY ./workspace/src/ ${COLCON_WS}/src/

RUN apt update && apt install -y \
    git \
    python3-pip \
    x11-apps \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-cartographer \
    ros-${ROS_DISTRO}-cartographer-ros \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-servo \
    ros-${ROS_DISTRO}-gazebo-ros2-control \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri

RUN rosdep update && rosdep install -i --from-path src --rosdistro ${ROS_DISTRO} -y

RUN rm -rf /var/lib/apt/lists/*

RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd ${COLCON_WS} && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source ${COLCON_WS}/install/setup.bash" >> ~/.bashrc
RUN echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc

CMD ["bash"]