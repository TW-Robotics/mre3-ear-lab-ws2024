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
    ros-$ROS_DISTRO-rgbd-launch \
    curl \
    zip \
    unzip \
    tar \
    libtool \
    ros-noetic-grpc \
    ros-noetic-eigen-conversions \
    protbuf-compiler \
    libprotobuf-dev \
    python2-catkin-tools \
    python3-osrf-pycommon \
    ros-noetic-rviz \
    qt5-default \
    ros-noetic-pcl-conversions \
    ros-noetic-pcl-ros

RUN python3 -m pip install \
    wheel

RUN mkdir -p ~/catkin_ws/src && \
    cd ~/catkin_ws/src && \
    git clone -b ros1-legacy https://github.com/IntelRealSense/realsense-ros.git

RUN cd ~/catkin_ws/src && \
    git clone https://github.com/VIS4ROB-lab/voxfield.git && \
    wstool merge -t . ./voxfield/voxfield_https.rosinstall && \
    wstool update

RUN cd ~/catkin_ws/src && \
    git clone https://github.com/ethz-asl/catkin_boost_python_buildtool.git && \
    git clone https://github.com/ethz-asl/catkin_simple.git && \
    git clone https://github.com/ethz-asl/cmake-build-debug.git && \
    git clone https://github.com/ethz-asl/eigen_catkin.git && \
    git clone https://github.com/ethz-asl/eigen_checks.git && \
    git clone https://github.com/ethz-asl/gflags_catkin.git && \
    git clone https://github.com/ethz-asl/glog_catkin.git && \
    git clone https://github.com/ethz-asl/minkindr.git && \
    git clone https://github.com/ethz-asl/minkindr-ros.git && \
    git clone https://github.com/ethz-asl/numpy_eigen.git

COPY ./files/rs_rgbd_launch ~/catkin_ws/src/realsense-ros/realsense2_camera/launch/

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash \
    && cd ~/catkin_ws \
    && catkin build voxblox_ros realsense2_camera realsense2_description -j3"

# Copy launch file and set the entrypoint
COPY my_entrypoint.sh .

RUN chmod +x my_entrypoint.sh

ENTRYPOINT [ "./my_entrypoint.sh" ]
