#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
import threading

pose_received = False  # Global flag to track if AMCL responds


def amcl_pose_callback(msg):
    global pose_received
    rospy.loginfo("AMCL pose received, stopping initial pose publication.")
    pose_received = True


def publish_initial_pose():
    global pose_received
    rospy.init_node('initial_pose_publisher', anonymous=True)
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    
    # Subscribe to AMCL output to detect pose update
    # rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amcl_pose_callback)
    
    # Create and populate the message
    initial_pose = PoseWithCovarianceStamped()
    initial_pose.header.frame_id = "map"
    initial_pose.pose.pose.position.x = 300.0
    initial_pose.pose.pose.position.y = 300.0
    initial_pose.pose.pose.position.z = 0.0
    initial_pose.pose.pose.orientation.x = 0.0
    initial_pose.pose.pose.orientation.y = 0.0
    initial_pose.pose.pose.orientation.z = 0.707
    initial_pose.pose.pose.orientation.w = 0.707

    # Covariance matrix (identity with slight noise)
    initial_pose.pose.covariance = [0.1, 0, 0, 0, 0, 0,
                                    0, 0.1, 0, 0, 0, 0,
                                    0, 0, 0.1, 0, 0, 0,
                                    0, 0, 0, 0.1, 0, 0,
                                    0, 0, 0, 0, 0.1, 0,
                                    0, 0, 0, 0, 0, 0.1]
    
    # Publish initial pose every second for up to 10 seconds or until AMCL acknowledges
    rospy.sleep(10)
    start_time = rospy.Time.now().to_sec()
    
    # while not pose_received and rospy.Time.now().to_sec() - start_time < 10:
    initial_pose.header.stamp = rospy.Time.now()
    rospy.loginfo("Publishing initial pose...")
    pub.publish(initial_pose)


if __name__ == '__main__':
    try:
        publish_initial_pose()
    except rospy.ROSInterruptException:
        pass
