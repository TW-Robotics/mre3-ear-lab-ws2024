#!/usr/bin/env python3
import rospy
import rosnode
import subprocess

def check_nodes(target_nodes):
    """Check which specified ROS nodes are online."""
    try:
        active_nodes = rosnode.get_node_names()
        missing_nodes = [node for node in target_nodes if node not in active_nodes]
        return missing_nodes
    except Exception as e:
        rospy.logerr(f"Error checking nodes: {e}")
        return target_nodes  # Assume all nodes are missing in case of error

def launch_ptg_node():
    """Launch the ptg node and return the process handle."""
    try:
        rospy.loginfo("Launching ptg node...")
        process = subprocess.Popen(
            ["roslaunch", "pointcloud_to_grid", "demo.launch"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        return process
    except Exception as e:
        rospy.logerr(f"Failed to launch ptg node: {e}")
        return None

if __name__ == "__main__":
    # Configuration section
    target_nodes = ["/test_node", "/example_node"]  # List of nodes to monitor
    check_rate_hz = 1  # Frequency to check node status in Hz

    # Initialize ROS node
    rospy.init_node("network_check_node")
    rate = rospy.Rate(check_rate_hz)

    ptg_process = None  # Will hold the ptg process handle
    ptg_launched = False  # Track if the ptg node has been launched

    rospy.loginfo(f"Monitoring nodes: {target_nodes}")

    try:
        while not rospy.is_shutdown():
            # Check which nodes are missing
            missing_nodes = check_nodes(target_nodes)
            if missing_nodes:
                rospy.logwarn(f"The following nodes are missing: {missing_nodes}")
            else:
                rospy.loginfo("All nodes are online.")

                # Launch the ptg node if it hasn't been launched yet
                if not ptg_launched:
                    ptg_process = launch_ptg_node()
                    if ptg_process:
                        ptg_launched = True
                        rospy.loginfo("ptg node successfully launched.")
                    else:
                        rospy.logerr("Failed to launch ptg node.")

            # Ensure the script keeps running; do not stop the ptg node
            if ptg_launched and ptg_process and ptg_process.poll() is not None:
                rospy.logwarn("ptg node process stopped unexpectedly, but will not be restarted as per requirements.")

            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down node monitoring.")
    rospy.spin()