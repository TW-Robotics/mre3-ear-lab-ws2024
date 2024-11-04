# Directory Structure

All directories must include a Docker Compose `compose.yaml` file that starts the respective part of the robot control pipeline. Use the `host` network mode to make ROS nodes communicate between Docker containers.

- **hw_bringup**: Packages that communicate with the chassis *hardware-specific*
- **hw_sensing**: Direct interfacing with sensors & Sensor specific data processing (e.g., Laserscan Merging) *hardware-specific*
- **perception**: Odometry estimation and SLAM
- **navigation**: Trajectory planning and control, priority-based multiplexing for semi-autonomy
- **systemd**: SystemD service file and scripts for Compose autostart

`run_in_dirs.py` is a convencience script that simultaneously runs a command in specified directories. The script replaces the SystemD autostart functionality and can be used to:

- build all containers in one go `python run_in_dirs.py docker compose build`
- start all containers `python run_in_dirs.py docker compose up --detatch`
- stop all containers `python run_in_dirs.py docker compose down`
