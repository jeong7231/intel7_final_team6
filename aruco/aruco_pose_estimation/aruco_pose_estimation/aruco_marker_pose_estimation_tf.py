#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image
from tf2_ros import TransformBroadcaster

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

# ==============================
# ArUco dictionaries
# ==============================
ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}

# ==============================
# ROS2 Node
# ==============================
class ArucoNode(Node):
    def __init__(self):
        super().__init__('aruco_node')

        # Parameters
        self.declare_parameter("aruco_dictionary_name", "DICT_ARUCO_ORIGINAL")
        self.declare_parameter("aruco_marker_side_length", 0.10)
        self.declare_parameter(
            "camera_calibration_parameters_filename",
            "/home/ubuntu/ros2_ws/src/aruco_pose_estimation/calibration.yaml"
        )
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("aruco_marker_name", "aruco_marker")

        # Read params
        aruco_dictionary_name = self.get_parameter("aruco_dictionary_name").value
        self.aruco_marker_side_length = self.get_parameter("aruco_marker_side_length").value
        self.camera_calibration_parameters_filename = self.get_parameter("camera_calibration_parameters_filename").value
        image_topic = self.get_parameter("image_topic").value
        self.aruco_marker_name = self.get_parameter("aruco_marker_name").value

        # Load calibration (ROS YAML, skip %YAML header if exists)
        with open(self.camera_calibration_parameters_filename, "r") as f:
            lines = f.readlines()
        if lines[0].startswith("%YAML"):
            lines = lines[1:]
        calib_data = yaml.safe_load("".join(lines))

        self.mtx = np.array(calib_data["camera_matrix"]["data"], dtype=np.float32).reshape((3, 3))
        self.dst = np.array(calib_data["distortion_coefficients"]["data"], dtype=np.float32).reshape((1, 5))

        # Dictionary + detector
        self.this_aruco_dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_dictionary_name])
        self.this_aruco_parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.this_aruco_dictionary, self.this_aruco_parameters)

        self.get_logger().info(f"[INFO] detecting '{aruco_dictionary_name}' markers...")

        # Subscriber
        self.subscription = self.create_subscription(
            Image, image_topic, self.listener_callback, qos_profile=qos_profile_sensor_data
        )

        # TF broadcaster
        self.tfbroadcaster = TransformBroadcaster(self)

        # CvBridge
        self.bridge = CvBridge()

    def listener_callback(self, data):
        # Convert ROS image to OpenCV (force BGR8)
        current_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

        # Detect markers
        corners, marker_ids, rejected = self.detector.detectMarkers(current_frame)

        if marker_ids is not None:
            cv2.aruco.drawDetectedMarkers(current_frame, corners, marker_ids)

            # 3D object points (marker corners in its own frame)
            half_size = self.aruco_marker_side_length / 2.0
            obj_points = np.array([
                [-half_size,  half_size, 0],
                [ half_size,  half_size, 0],
                [ half_size, -half_size, 0],
                [-half_size, -half_size, 0]
            ], dtype=np.float32)

            for i, marker_id in enumerate(marker_ids):
                img_points = corners[i][0].astype(np.float32)

                success, rvec, tvec = cv2.solvePnP(
                    obj_points, img_points, self.mtx, self.dst
                )
                if not success:
                    continue

                # TF message
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = "default_cam"   # webcam frame_id
                t.child_frame_id = f"{self.aruco_marker_name}_{marker_id[0]}"

                t.transform.translation.x = float(tvec[0][0])
                t.transform.translation.y = float(tvec[1][0])
                t.transform.translation.z = float(tvec[2][0])

                rotation_matrix = cv2.Rodrigues(rvec)[0]
                quat = R.from_matrix(rotation_matrix).as_quat()

                t.transform.rotation.x = float(quat[0])
                t.transform.rotation.y = float(quat[1])
                t.transform.rotation.z = float(quat[2])
                t.transform.rotation.w = float(quat[3])

                self.tfbroadcaster.sendTransform(t)
                cv2.drawFrameAxes(current_frame, self.mtx, self.dst, rvec, tvec, 0.05)

        cv2.imshow("camera", current_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

