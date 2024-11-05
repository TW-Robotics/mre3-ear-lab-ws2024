#!/usr/bin/env bash

bash -c "source /home/workstation/catkin_ws/devel/setup.bash && roslaunch taurob_teleop_twist_joy teleop.launch joy_config:=xbox joy_dev:=/dev/input/js0 &"
bash -c "source /home/workstation/catkin_ws/devel/setup.bash && roslaunch fhtw_launch tracker.launch --wait &"

while [ -L /dev/Xbox360Con ]; do
    sleep 0.1
done
for proc in $(rosnode list); do
    rosnode kill "$proc"
done
kill "$(pgrep roscore)"
kill "$(pgrep rosmaster)"
