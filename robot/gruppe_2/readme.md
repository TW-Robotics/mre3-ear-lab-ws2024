# Gruppe_1 Roadmap
## Docker Container Setup
### Interface Docker Setup [Dockerfile]
- install ROS-Noetic
- install pointcloud to occupancy grid
    - [https://github.com/jkk-research/pointcloud_to_grid/tree/ros2]

### Docker Compose
- Interface starts host roscore
- Checks client availability [network init verification] --> publish three "ready"-topics, one for each group

### Interface actions
- get pointlcloud from drone [https://github.com/VIS4ROB-lab/voxfield/tree/main]
    - We will use the msg type tsdf_pointcloud pcl::PointCloud<pcl::PointXYZI>
    - The published msg type (although differently documented) is of type sensor_msgs/PointCloud2 with x,y,z and intensity
    - Apperently only one slice of the 3d pointcloud is published and the hight is defined in foxfield by the drone people
    - We need to check if it is really a 2d slice of the pointcloud and that the following converter can handle that
- use pointcloud_to_grid ROS 2 package [https://github.com/jkk-research/pointcloud_to_grid?tab=readme-ov-file#readme]
    to convert to occupency grid
  - this package subscribes to a sensor_msgs/PointCloud2 with x,y,z and intensity
  - after converting it to a occupency grid the node publishes it

## Next Steps
### General
- create second dockerfile [test_client]
    - publishes pcl::pointcloudRBG to topic
- create docker-compose.yaml

 ### network init 
 - create script to check availability of test_client

 ### Occupancy Grid Map OLD Methode
 - subscribe to topic provided by test_client 
    - get pointcloud
- copmute pointcloud to occupancy grid with [https://github.com/jkk-research/pointcloud_to_grid?tab=readme-ov-file#readme]

## What we dont know
- pcl rosdep skip in dockerfile?
- pcl install at beginning dockerfile


## remember
- DOCKER_BUILDKIT=1 docker build --ssh default --build-arg CACHEBUST=$(date +%s) -f Dockerfile.interface -t interface-docker .
- roslaunch interface_pkg interface.launch || sleep infinity"
- xhost +local:root
- subscribe pcl::PointCloud2 here: <param name="cloud_in_topic" value="/left_os1/os1_cloud_node/points"/>
- publishes ground map here: <param name="mapi_topic_name" value="lidargrid_i"/>
- publishes hight map here: <param name="maph_topic_name" value="lidargrid_h"/>
-- adjustable in init to with setting ros param

# toDo
-- launch pointcloud_to_grid rviz launch in initScript.py
