#!/usr/bin/env python

import rospy
import subprocess
from nav_msgs.msg import OccupancyGrid
import std_msgs.msg
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class CoordinationNode:
    def __init__(self):
        rospy.init_node('coordination_node', anonymous=True)
        self.rate = rospy.Rate(1.5)  # 1.5 Hz loop rate
        rospy.sleep(2.0)

        # Launch the PCL to scan node
        self.launch_file("localization", "pcl_to_scan.launch")
        rospy.loginfo("PCL to scan node launched.")
        self.rate.sleep()

        # Flags to track if components are launched
        self.map_checked = False
        self.aruco_launched = False
        self.amcl_launched = False
        self.local_planner_launched = False
        self.aruco_process = None
        self.amcl_process = None

    def is_topic_available(self, topic_name, msg_type, timeout=1.0):
        """
        Check if a specific topic is available by waiting for a message.
        """
        try:
            rospy.wait_for_message(topic_name, msg_type, timeout=timeout)
            return True
        except rospy.ROSException:
            rospy.logwarn(f"Topic '{topic_name}' is not available.")
            return False

    def launch_file(self, package, launch_file, quiet=False):
        """
        Launch a ROS launch file using subprocess.

        :param package: Name of the ROS package containing the launch file.
        :param launch_file: Name of the launch file to be executed.
        :param quiet: If True, adds the --quiet argument to suppress terminal output.
        :return: subprocess.Popen object for the launched process.
        """
        cmd = ["roslaunch", package, launch_file]
        if quiet:
            cmd.append("--quiet")  # Add the --quiet flag if requested
        rospy.loginfo(f"Launching launch file: {' '.join(cmd)}")
        return subprocess.Popen(cmd)

    def run_node(self, package, node):
        """
        Starts a ROS Python node using rosrun.
        
        :param package: Name of the ROS package containing the node.
        :param node: Name of the Python node (script name).
        :return: subprocess.Popen object for the launched process, or None if the launch fails.
        """
        cmd = ["rosrun", package, node]  # Base command for rosrun
        
        try:
            # Launch the node using rosrun
            process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            rospy.loginfo(f"Started node: {node} in package: {package}")
            return process
        except Exception as e:
            rospy.loginfo(f"Failed to start node {node} in package {package}: {e}")
            return None


    def check_map_topic(self):
        if not self.map_checked:
            if not self.is_topic_available('/map', OccupancyGrid):
                rospy.loginfo("Waiting for '/map' topic...")
                self.rate.sleep()
                return False
            else:
                rospy.loginfo("'/map' topic is available.")
                self.map_checked = True
        return True

    def check_and_launch_aruco(self):
        if not self.aruco_launched:
            self.aruco_process = self.launch_file("aruco_estimation", "detect_aruco.launch")
            self.aruco_launched = True

        # Check if ArUco detection node is still running
        if self.aruco_launched:
            if self.aruco_process.poll() is not None or not self.is_topic_available('/aruco_detected_markers', std_msgs.msg.String):
                rospy.logwarn("ArUco detection Node is not running correctly. Relaunching...")
                self.aruco_process = self.launch_file("aruco_estimation", "detect_aruco.launch")

    def check_and_launch_amcl(self):
        if (not self.is_topic_available('/initialpose', PoseWithCovarianceStamped) or
                not self.is_topic_available('/alexBestScan', LaserScan) or
                not self.is_topic_available('/odom', Odometry)):
            rospy.loginfo("Waiting for AMCL prerequisites...")
            self.rate.sleep()
            return False

        if not self.amcl_launched:
            self.amcl_process = self.launch_file("localization", "amcl_localization.launch")
            self.amcl_launched = True

        return True

    def check_and_launch_local_planner(self):
        # Select and launch local planner based on argument
        if not self.local_planner_launched:
            # Normalisiere den Parameter auf Kleinschreibung
            local_planner = rospy.get_param('~local_planner', 'teb').lower()

            if local_planner == 'nmpc':
                self.launch_file("amr_control", "move_base_nMPC.launch", quiet=True)
                rospy.loginfo("nMPC local planner launched.")
            elif local_planner == 'teb':
                self.launch_file("amr_control", "move_base_TEB.launch", quiet=True)
                rospy.loginfo("TEB local planner launched.")
            elif local_planner == 'dwa':
                self.launch_file("amr_control", "move_base_DWA.launch", quiet=True)
                rospy.loginfo("DWA local planner launched.")
            else:
                rospy.logwarn(f"Unknown local planner '{local_planner}', defaulting to TEB.")
                self.launch_file("amr_control", "move_base_TEB.launch", quiet=True)
                rospy.loginfo("TEB local planner launched.")
            
            rospy.sleep(2.0)
            self.local_planner_launched = True


    def run(self):
        while not rospy.is_shutdown():
            rospy.loginfo("|----------------------------------|")

            # Check and wait for the map topic
            if not self.check_map_topic():
                continue

            # # Check and wait for the drone image topic
            # if not self.is_topic_available('/drone_image', OccupancyGrid):
            #     rospy.loginfo("Waiting for '/drone_image' topic...")
            #     self.rate.sleep()
            #     # continue

            # Check and launch ArUco detection
            # self.check_and_launch_aruco()

            # # For testing purposes, publish an initial pose. Delete in real deployment.
            # self.run_node("localization", "publishInitialPose.py")

            # Check and launch AMCL
            if not self.check_and_launch_amcl():
                continue

            # Check and launch the local planner
            self.check_and_launch_local_planner()

            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = CoordinationNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
