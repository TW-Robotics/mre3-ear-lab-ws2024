#!/usr/bin/env python3
import rospy
import rosnode
import time

def check_node(node_name):
    """Check if a specific ROS node is running."""
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

if __name__ == "__main__":
    rospy.init_node("network_check_node")
    target_node = "/test_node"  # Replace with your test container's node name
    rate = rospy.Rate(1)  # Check every second

    while not rospy.is_shutdown():
        check_node(target_node)
        rate.sleep()
