#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from tf.transformations import quaternion_from_euler


class ArucoDetectorROS:
    MARKER_SIZE = 0.0717  # in meters

    def __init__(self, marker_length=MARKER_SIZE):
        rospy.loginfo("Initializing ArUco Detector Node")
        self.bridge = CvBridge()
        self.marker_length = marker_length

        # ArUco setup
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # Camera calibration data
        self.camera_matrix = None
        self.dist_coeffs = None

        # Publishing rate control
        self.last_publish_time = rospy.Time.now()
        self.publish_rate = rospy.Duration(1.0)  # 1 second interval

        # Publisher for marker pose
        self.pose_pub = rospy.Publisher("/initialpose", PoseStamped, queue_size=1)

        # Subscribers
        self.image_sub = rospy.Subscriber("/oak/rgb/image_raw", Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber("/oak/rgb/camera_info", CameraInfo, self.camera_info_callback)

    def camera_info_callback(self, msg):
        """Process camera calibration data."""
        if self.camera_matrix is None:
            rospy.loginfo("Received camera calibration")
            self.camera_matrix = np.array(msg.K).reshape(3, 3)
            self.dist_coeffs = np.array(msg.D)[:5]
            self.camera_info_sub.unregister()
            rospy.loginfo("Camera info subscriber unregistered")

    def should_publish(self):
        """Check if enough time has passed to publish again."""
        current_time = rospy.Time.now()
        if (current_time - self.last_publish_time) >= self.publish_rate:
            self.last_publish_time = current_time
            return True
        return False

    def publish_pose(self, rvec, tvec, timestamp):
        """Publish the marker pose if enough time has passed."""
        if not self.should_publish():
            return

        pose_msg = PoseStamped()
        pose_msg.header.stamp = timestamp
        pose_msg.header.frame_id = "camera_optical_frame"

        # Set position
        pose_msg.pose.position.x = tvec[0]
        pose_msg.pose.position.y = tvec[1]
        pose_msg.pose.position.z = tvec[2]

        # Convert rotation vector to quaternion
        rot_matrix, _ = cv2.Rodrigues(rvec)
        roll = np.arctan2(rot_matrix[2, 1], rot_matrix[2, 2])
        pitch = np.arctan2(-rot_matrix[2, 0], np.sqrt(rot_matrix[2, 1] ** 2 + rot_matrix[2, 2] ** 2))
        yaw = np.arctan2(rot_matrix[1, 0], rot_matrix[0, 0])
        quat = quaternion_from_euler(roll, pitch, yaw)

        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]

        self.pose_pub.publish(pose_msg)
        rospy.loginfo_throttle(1.0, f"Published pose for marker 0: x={tvec[0]:.3f}, y={tvec[1]:.3f}, z={tvec[2]:.3f}")

    def image_callback(self, msg):
        """Process RGB frames."""
        if self.camera_matrix is None:
            rospy.logwarn_throttle(5, "Waiting for camera calibration...")
            return

        try:
            # Convert ROS Image to OpenCV format
            rgb_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Detect ArUco markers
            corners, ids, rejected = cv2.aruco.detectMarkers(rgb_frame, self.aruco_dict, parameters=self.aruco_params)

            if ids is not None and 0 in ids:
                timestamp = rospy.Time.now()
                marker_index = np.where(ids == 0)[0][0]

                try:
                    # Estimate pose using PnP
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corners, self.marker_length, self.camera_matrix, self.dist_coeffs
                    )
                    rvec = rvecs[marker_index][0]
                    tvec = tvecs[marker_index][0]

                    # Visualization
                    cv2.aruco.drawDetectedMarkers(rgb_frame, corners, ids)
                    cv2.drawFrameAxes(
                        rgb_frame,
                        self.camera_matrix,
                        self.dist_coeffs,
                        rvec,
                        tvec,
                        self.marker_length / 2,
                    )

                    # Draw position information
                    position_text = f"Marker 0: X:{tvec[0]:.3f}m Y:{tvec[1]:.3f}m Z:{tvec[2]:.3f}m"
                    cv2.putText(rgb_frame, position_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    self.publish_pose(rvec, tvec, timestamp)

                except Exception as e:
                    rospy.logerr(f"Failed to estimate marker pose: {e}")

            # Display the frame
            cv2.imshow("Frame", rgb_frame)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f"Failed to process RGB frame: {e}")


def main():
    """Main function."""
    rospy.init_node("aruco_detector_node")
    marker_length = rospy.get_param("~marker_length", 0.0717)  # in meters

    try:
        detector = ArucoDetectorROS(marker_length=marker_length)
        rospy.loginfo("ArUco detector node started")
        rospy.spin()
    except Exception as e:
        rospy.logerr(f"Node crashed: {e}")
    finally:
        cv2.destroyAllWindows()
        rospy.loginfo("Node shutting down")


if __name__ == "__main__":
    main()
