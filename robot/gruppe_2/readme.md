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
- Checks client availability [network init verification]

### Interface actions
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

 ### Occupancy Grid Map
 - subscribe to topic provided by test_client 
    - get pointcloud
- copmute pointcloud to ufo map with [https://github.com/UnknownFreeOccupied/ufomap.git]
- compute ufomap to occupancy grid with [https://github.com/LTU-RAI/Map-Conversion-3D-Voxel-Map-to-2D-Occupancy-Map.git]

OR
- compute pointcloud directly to occupancy grid [https://github.com/jkk-research/pointcloud_to_grid.git]

## What we dont know
- making sure cloning the correct branch "ros" Map-Conversion-3D-Voxel-Map-to-2D-Occupancy-Map.git for  & pointcloud_to_grid.git