# Use the official ROS Noetic base image
FROM ros:noetic-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=noetic

# Update and upgrade system packages
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
        tmux \
        ros-$ROS_DISTRO-realsense2-camera \
        git \
        ros-$ROS_DISTRO-depthai-ros \
        ros-$ROS_DISTRO-rgbd-launch

RUN mkdir -p ~/catkin_ws/src && \
    cd ~/catkin_ws/src && \
    git clone -b ros1-legacy https://github.com/IntelRealSense/realsense-ros.git

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash \
    && cd ~/catkin_ws \
    && catkin_make"

# Copy launch file and set the entrypoint
COPY my_entrypoint.sh .

RUN chmod +x my_entrypoint.sh

ENTRYPOINT [ "./my_entrypoint.sh" ]
