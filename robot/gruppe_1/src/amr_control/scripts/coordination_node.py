#!/usr/bin/env python

import rospy
import subprocess
from nav_msgs.msg import OccupancyGrid

def is_topic_available(topic_name, msg_type, timeout=1.0):
    """
    Check if a specific topic is available by waiting for a message.
    """
    try:
        rospy.wait_for_message(topic_name, msg_type, timeout=timeout)
        return True
    except rospy.ROSException:
        rospy.logwarn(f"Topic '{topic_name}' is not available.")
        return False

def launch_file(package, launch_file, args=None):
    """
    Launch a ROS launch file using subprocess.
    """
    cmd = ["roslaunch", package, launch_file]
    if args:
        cmd.extend(args)
    rospy.loginfo(f"Launching launch file: {' '.join(cmd)}")
    return subprocess.Popen(cmd)

def coordination_node():
    rospy.init_node('coordination_node', anonymous=True)
    rate = rospy.Rate(1)  # 1 Hz loop rate

    map_checked = False
    aruco_launched = False
    amcl_launched = False
    local_planner_launched = False

    while not rospy.is_shutdown():
        rospy.loginfo("|----------------------------------|")
        # Check the '/map' topic only once
        if not map_checked:
            if not is_topic_available('/map', OccupancyGrid):
                rospy.loginfo("Waiting for '/map' topic...")
                rate.sleep()
                continue
            else:
                rospy.loginfo("'/map' topic is available.")
                map_checked = True

        # Check for the '/drone_image' topic
        if not is_topic_available('/drone_image', OccupancyGrid):
            rospy.loginfo("Waiting for '/drone_image' topic...")
            rate.sleep()
            continue

        # Launch ArUco detection launch file if not already launched
        if not aruco_launched:
            aruco_process = launch_file("aruco_detect", "aruco_detect.launch")
            aruco_launched = True

        # Check for additional topics required for AMCL
        if (not is_topic_available('/init_pose', OccupancyGrid) or
                not is_topic_available('/scan', OccupancyGrid) or
                not is_topic_available('/odom', OccupancyGrid)):
            rospy.loginfo("Waiting for AMCL prerequisites...")
            rate.sleep()
            continue

        # Launch AMCL launch file if not already launched
        if not amcl_launched:
            amcl_process = launch_file("amcl", "amcl.launch")
            amcl_launched = True

        # Select and launch local planner based on argument
        if not local_planner_launched:
            local_planner = rospy.get_param('~local_planner', 'teb')

            if local_planner == 'nmpc':
                launch_file("amr_control", "move_base_nMPC.launch")
            elif local_planner == 'teb':
                launch_file("amr_control", "move_base_TEB.launch")
            elif local_planner == 'dwa':
                launch_file("amr_control", "move_base_DWA.launch")
            else:
                rospy.logwarn(f"Unknown local planner '{local_planner}', defaulting to TEB.")
                launch_file("amr_control", "teb.launch")
            
            local_planner_launched = True

        rate.sleep()  # Wait before next iteration

if __name__ == '__main__':
    try:
        coordination_node()
    except rospy.ROSInterruptException:
        pass
