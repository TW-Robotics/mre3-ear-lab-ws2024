from aruco_detect import ArucoDetectorROS


def main():
    detector = ArucoDetectorROS(calibration_file="./camera_intrinsic.yaml")
    detector.process_video("./aruco_marker.mp4", "output.avi")


if __name__ == "__main__":
    main()
