FROM osrf/ros:humble-desktop-full-jammy


LABEL Maintainer="Jakub Czech <czechjakub@icloud.com>"
LABEL Description="Turtlebot ROS2 Galactic Image"

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV TURTLEBOT3_MODEL=burger
ENV ROS_DOMAIN_ID=33

RUN ./ros_entrypoint.sh
RUN apt-get update && apt-get upgrade -y && apt-get autoremove -y
RUN apt-get install -y apt-utils \
    ros-$ROS_DISTRO-turtlebot3 \
    ros-$ROS_DISTRO-turtlebot3-* \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-rqt* \
    ros-$ROS_DISTRO-slam-toolbox \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-nav2-util \
    ros-$ROS_DISTRO-tf2-eigen \
    ros-dev-tools \
    python3-colcon-common-extensions \
    libgsl-dev

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

WORKDIR /root/workspace
VOLUME /dev/shm /dev/shm
VOLUME ./src:/root/workspace/
