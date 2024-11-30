# Gruppe_1 Roadmap
## Docker Container Setup
### Interface Docker Setup [Dockerfile]
- install ROS-Noetic
- install Map-Conversion-3D-Voxel-Map-to-2D-Occupancy-Map
    - git@github.com:LTU-RAI/Map-Conversion-3D-Voxel-Map-to-2D-Occupancy-Map.git [branch "ros"]

### Docker Compose
- Interface starts host roscore
- Checks client availability [network init verification]

### Interface actions
- gets Voxel Map from drone
- gets aruco marker pictures from drone
- computes Voxel to Occupancy grid Map [respects ground hight]
- provides map & aruco marker picture to AMCL
