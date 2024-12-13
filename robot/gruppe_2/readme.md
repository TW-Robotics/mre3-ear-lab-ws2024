# Gruppe_1 Roadmap
## Docker Container Setup
### Interface Docker Setup [Dockerfile]
- install ROS-Noetic
- install Map-Conversion-3D-Voxel-Map-to-2D-Occupancy-Map
    - https://github.com/LTU-RAI/Map-Conversion-3D-Voxel-Map-to-2D-Occupancy-Map.git [branch "ros"]
- install pointcloud to ufomap
    - https://github.com/UnknownFreeOccupied/ufomap.git
- install pointcloud to occupancy grid
    - https://github.com/jkk-research/pointcloud_to_grid.git [branch "ros"]

### Docker Compose
- Interface starts host roscore
- Checks client availability [network init verification] --> publish three "ready"-topics, one for each group

### Interface actions
##NEW
- get pointlcloud from drone. One of these topics
    - surface_pointcloud pcl::PointCloud<pcl::PointXYZRGB>
    - tsdf_pointcloud pcl::PointCloud<pcl::PointXYZI>
    -mesh_pointcloud pcl::PointCloud<pcl::PointXYZRGB>
- convert pcl::PointCloud to sensor_msgs/PointCloud2 with custom node (Shouldnt be to hard say Simon)
- use pointcloud_to_grid ROS 2 package (https://github.com/jkk-research/pointcloud_to_grid?tab=readme-ov-file#readme)
    to convert to occupency grid
- publish topic
##OLD
- gets Voxel Map from drone
- gets aruco marker pictures from drone
- computes Voxel to Occupancy grid Map [respects ground hight]
- provides map & aruco marker picture to AMCL


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
- copmute pointcloud to ufo map with [https://github.com/UnknownFreeOccupied/ufomap.git]
- compute ufomap to occupancy grid with [https://github.com/LTU-RAI/Map-Conversion-3D-Voxel-Map-to-2D-Occupancy-Map.git]

## What we dont know
- making sure cloning the correct branch "ros" Map-Conversion-3D-Voxel-Map-to-2D-Occupancy-Map.git for  & pointcloud_to_grid.git
