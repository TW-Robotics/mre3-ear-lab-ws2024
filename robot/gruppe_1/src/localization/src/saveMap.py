#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
import subprocess

def save_map_callback(msg):
    rospy.loginfo("Saving map from drone...")
    subprocess.call(["rosrun", "map_server", "map_saver", "-f", "/app/src/localization/map/drone_map"])
    rospy.loginfo("Map saved successfully!")

    # Restart map_server to reload the updated map
    subprocess.call(["rosnode", "kill", "/map_server"])
    subprocess.Popen(["rosrun", "map_server", "map_server", "/app/src/localization/map/drone_map.yaml"])

if __name__ == '__main__':
    rospy.init_node('map_saver_node')
    rospy.Subscriber("/map", OccupancyGrid, save_map_callback)
    rospy.spin()
