#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from tf.transformations import quaternion_from_euler


class ArucoDetectorROS:
    def __init__(self, marker_length=0.0717, use_depth=True):
        rospy.loginfo("Initializing ArUco Detector Node")
        self.bridge = CvBridge()
        self.marker_length = marker_length
        self.use_depth = use_depth
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        self.camera_matrix = None
        self.dist_coeffs = None
        self.initial_poses_published = set()

        # Publishers for initial poses
        self.pose_pubs = {
            marker_id: rospy.Publisher(f"/arucoinitialpose_{marker_id}", PoseStamped, queue_size=1)
            for marker_id in [1, 2, 3]
        }

        if self.use_depth:
            rospy.loginfo("Using depth for distance estimation")
            rgb_sub = Subscriber("/oak/rgb/image_raw", Image)
            depth_sub = Subscriber("/oak/depth/image_raw", Image)
            self.sync = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=5, slop=0.1)
            self.sync.registerCallback(self.sync_callback)
        else:
            rospy.loginfo("Using marker size and camera intrinsics for distance")
            self.image_sub = rospy.Subscriber("/oak/rgb/image_raw", Image, self.image_callback)

        self.camera_info_sub = rospy.Subscriber("/oak/rgb/camera_info", CameraInfo, self.camera_info_callback)

    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            rospy.loginfo("Received camera calibration")
            self.camera_matrix = np.array(msg.K).reshape(3, 3)
            self.dist_coeffs = np.array(msg.D)[:5]
            rospy.logdebug(f"Camera Matrix:\n{self.camera_matrix}")
            rospy.logdebug(f"Distortion Coefficients: {self.dist_coeffs}")
            self.camera_info_sub.unregister()
            rospy.loginfo("Camera info subscriber unregistered")

    def publish_pose(self, marker_id, rvec, tvec, timestamp):
        if marker_id in self.initial_poses_published:
            return

        pose_msg = PoseStamped()
        pose_msg.header.stamp = timestamp
        pose_msg.header.frame_id = "camera_optical_frame"

        pose_msg.pose.position.x = tvec[0]
        pose_msg.pose.position.y = tvec[1]
        pose_msg.pose.position.z = tvec[2]

        # Convert rotation vector to quaternion
        if rvec is not None:
            rot_matrix, _ = cv2.Rodrigues(rvec)
            roll = np.arctan2(rot_matrix[2, 1], rot_matrix[2, 2])
            pitch = np.arctan2(-rot_matrix[2, 0], np.sqrt(rot_matrix[2, 1] ** 2 + rot_matrix[2, 2] ** 2))
            yaw = np.arctan2(rot_matrix[1, 0], rot_matrix[0, 0])
            quat = quaternion_from_euler(roll, pitch, yaw)
        else:
            quat = quaternion_from_euler(0, 0, 0)  # Default orientation for depth-only mode

        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]

        self.pose_pubs[marker_id].publish(pose_msg)
        self.initial_poses_published.add(marker_id)
        rospy.loginfo(f"Published initial pose for marker {marker_id}")

    def get_marker_center_depth(self, depth_image, marker_corners, window_size=5):
        center = np.mean(marker_corners, axis=0).astype(int)
        center_x, center_y = center

        h, w = depth_image.shape
        x_start = max(0, center_x - window_size // 2)
        x_end = min(w, center_x + window_size // 2 + 1)
        y_start = max(0, center_y - window_size // 2)
        y_end = min(h, center_y + window_size // 2 + 1)

        window = depth_image[y_start:y_end, x_start:x_end]
        valid_depths = window[window > 0]

        if len(valid_depths) > 0:
            z = np.median(valid_depths) / 1000.0  # Convert mm to meters
            # Back-project to 3D using pinhole camera model
            x = (center_x - self.camera_matrix[0, 2]) * z / self.camera_matrix[0, 0]
            y = (center_y - self.camera_matrix[1, 2]) * z / self.camera_matrix[1, 1]
            return np.array([x, y, z])
        return None

    def sync_callback(self, rgb_msg, depth_msg):
        if self.camera_matrix is None:
            rospy.logwarn_throttle(5, "Waiting for camera calibration...")
            return

        try:
            rgb_frame = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            depth_frame = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
            self.process_frame(rgb_frame, depth_frame)
        except Exception as e:
            rospy.logerr(f"Failed to convert image messages: {e}")

    def image_callback(self, msg):
        if self.camera_matrix is None:
            rospy.logwarn_throttle(5, "Waiting for camera calibration...")
            return

        try:
            rgb_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_frame(rgb_frame)
        except Exception as e:
            rospy.logerr(f"Failed to convert image message: {e}")

    def process_frame(self, rgb_frame, depth_frame=None):
        corners, ids, rejected = cv2.aruco.detectMarkers(rgb_frame, self.aruco_dict, parameters=self.aruco_params)

        poses = {}

        if ids is not None:
            timestamp = rospy.Time.now()

            if not self.use_depth:
                try:
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corners, self.marker_length, self.camera_matrix, self.dist_coeffs
                    )
                except Exception as e:
                    rospy.logerr(f"Failed to estimate marker poses: {e}")
                    return

            for i in range(len(ids)):
                marker_id = ids[i][0]
                if marker_id in [1, 2, 3]:
                    if self.use_depth and depth_frame is not None:
                        position = self.get_marker_center_depth(depth_frame, corners[i][0])
                        if position is not None:
                            poses[marker_id] = {"tvec": position, "corners": corners[i][0]}
                            self.publish_pose(marker_id, None, position, timestamp)
                    else:
                        poses[marker_id] = {"rvec": rvecs[i][0], "tvec": tvecs[i][0], "corners": corners[i][0]}
                        self.publish_pose(marker_id, rvecs[i][0], tvecs[i][0], timestamp)

            rgb_frame = cv2.aruco.drawDetectedMarkers(rgb_frame, corners, ids)

            y_pos = 30
            for marker_id in sorted(poses.keys()):
                x, y, z = poses[marker_id]["tvec"]
                method = "Depth" if self.use_depth else "PnP"
                position_text = f"Marker {marker_id} ({method}): X:{x:.3f}m Y:{y:.3f}m Z:{z:.3f}m"
                cv2.putText(rgb_frame, position_text, (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                y_pos += 30

                if not self.use_depth:
                    cv2.drawFrameAxes(
                        rgb_frame,
                        self.camera_matrix,
                        self.dist_coeffs,
                        poses[marker_id]["rvec"],
                        poses[marker_id]["tvec"],
                        self.marker_length / 2,
                    )

        cv2.imshow("Frame", rgb_frame)
        cv2.waitKey(1)


def main():
    rospy.init_node("aruco_detector_node")
    use_depth = rospy.get_param("~use_depth", True)

    try:
        detector = ArucoDetectorROS(use_depth=use_depth)
        rospy.loginfo(f"ArUco detector node started with depth={'enabled' if use_depth else 'disabled'}")
        rospy.spin()
    except Exception as e:
        rospy.logerr(f"Node crashed: {e}")
    finally:
        cv2.destroyAllWindows()
        rospy.loginfo("Node shutting down")


if __name__ == "__main__":
    main()
