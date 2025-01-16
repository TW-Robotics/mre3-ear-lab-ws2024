#!/usr/bin/env python3
import rospy
import rosnode
import subprocess
import time

def check_node(node_name):
    """Check if a specific ROS node is online."""
    try:
        active_nodes = rosnode.get_node_names()
        if node_name in active_nodes:
            rospy.loginfo(f"Node {node_name} is online.")
            return True
        else:
            rospy.logwarn(f"Node {node_name} is offline.")
            return False
    except Exception as e:
        rospy.logerr(f"Error checking nodes: {e}")
        return False

def launch_pointcloud_to_grid():
    """Launch the pointcloud_to_grid demo.launch file."""
    try:
        rospy.loginfo("Launching pointcloud_to_grid demo.launch...")
        subprocess.Popen(["roslaunch", "pointcloud_to_grid", "demo.launch"])
    except Exception as e:
        rospy.logerr(f"Failed to launch demo.launch: {e}")

if __name__ == "__main__":
    rospy.init_node("test_node")
    target_node = "/test_node"  # Name of the node to check
    rate = rospy.Rate(1)  # Check every second

    while not rospy.is_shutdown():
        if check_node(target_node):
            launch_pointcloud_to_grid()
            break  # Exit the loop after launching the file
        rate.sleep()
