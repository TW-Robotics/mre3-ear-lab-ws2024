import cv2
import numpy as np
import yaml


class ArucoDetector:
    def __init__(self, calibration_file, marker_length=0.0717):
        self.marker_length = marker_length
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        self.camera_matrix, self.dist_coeffs = self._load_calibration(calibration_file)

    def _load_calibration(self, calibration_file):
        with open(calibration_file, "r") as f:
            calib_data = yaml.safe_load(f)

        camera_matrix = np.array(calib_data["camera_matrix"]["data"]).reshape(3, 3)
        dist_coeffs = np.array(calib_data["distortion_coefficients"]["data"])[:5]

        return camera_matrix, dist_coeffs

    def detect_markers(self, frame):
        corners, ids, rejected = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)

        poses = {}
        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs
            )

            for i in range(len(ids)):
                marker_id = ids[i][0]
                if marker_id in [1, 2, 3]:
                    poses[marker_id] = {"rvec": rvecs[i][0], "tvec": tvecs[i][0], "corners": corners[i][0]}

                    # # Draw marker ID and position
                    # center = np.mean(corners[i][0], axis=0)
                    # x, y, z = tvecs[i][0]
                    # position_text = f"ID:{marker_id} X:{x:.2f}m Y:{y:.2f}m Z:{z:.2f}m"
                    # cv2.putText(
                    #     frame,
                    #     position_text,
                    #     (int(center[0]), int(center[1] - 50)),
                    #     cv2.FONT_HERSHEY_SIMPLEX,
                    #     0.5,
                    #     (0, 255, 0),
                    #     2,
                    # )

            frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            for marker_id, pose in poses.items():
                cv2.drawFrameAxes(
                    frame, self.camera_matrix, self.dist_coeffs, pose["rvec"], pose["tvec"], self.marker_length / 2
                )

        return frame, poses

    def process_video(self, video_path, output_path=None):
        cap = cv2.VideoCapture(video_path)

        if output_path:
            fourcc = cv2.VideoWriter_fourcc(*"XVID")
            out = cv2.VideoWriter(output_path, fourcc, 30.0, (int(cap.get(3)), int(cap.get(4))))

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            frame, poses = self.detect_markers(frame)

            # Draw positions in top-left corner
            y_pos = 30
            for marker_id in sorted(poses.keys()):
                pose = poses[marker_id]
                x, y, z = pose["tvec"]
                position_text = f"Marker {marker_id}: X:{x:.2f}m Y:{y:.2f}m Z:{z:.2f}m"
                cv2.putText(frame, position_text, (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                y_pos += 30

            if output_path:
                out.write(frame)

            cv2.imshow("Frame", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        cap.release()
        if output_path:
            out.release()
        cv2.destroyAllWindows()
