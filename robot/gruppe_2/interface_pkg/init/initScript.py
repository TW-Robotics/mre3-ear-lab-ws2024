#!/usr/bin/env python3
import rospy
import rosnode
import subprocess
import time

def check_node(node_name):
    """Check if a specific ROS node is online."""
    try:
        active_nodes = rosnode.get_node_names()
        return node_name in active_nodes
    except Exception as e:
        rospy.logerr(f"Error checking nodes: {e}")
        return False

def launch_ptg_node():
    """Launch the ptg node."""
    try:
        rospy.loginfo("Launching ptg node...")
        subprocess.Popen(["roslaunch", "pointcloud_to_grid", "demo.launch"])
    except Exception as e:
        rospy.logerr(f"Failed to launch ptg node: {e}")

if __name__ == "__main__":
    rospy.init_node("network_check_node")
    target_node = "/test_node"
    rate = rospy.Rate(1)  # Check every second

    while not rospy.is_shutdown():
        if check_node(target_node):
            rospy.loginfo(f"Node {target_node} is online. Launching ptg...")
            launch_ptg_node()
            break  # Exit after launching ptg
        else:
            rospy.logwarn(f"Node {target_node} is offline. Retrying...")
        rate.sleep()
