#!/bin/bash

# Check if the folder exists, clone open_vins if not
if [ ! -d "open_vins" ]; then
  git clone https://github.com/rpng/open_vins.git
fi

# Build open_vins base image from original repo
docker build -t open_vins -f open_vins/Dockerfile_ros1_20_04 ./open_vins

# Build custom image that compiles open_vins
docker build -t open_vins_built .

# Forward GUI
xhost +local:docker

# Start container
docker run -it --rm --net=host -v $PWD/open_vins/:/catkin_ws/src/open_vins/ \
    -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY --device=/dev/dri:/dev/dri \
    open_vins_built:latest bash

# Command to launch with realsense d455 in online mode
# roslaunch ov_msckf subscribe.launch --wait config:=rs_d455