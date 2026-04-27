#!/usr/bin/env python3
import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import message_filters
from geometry_msgs.msg import Pose

import numpy as np
import cv2

# Local imports
from aruco_pose_estimation.utils import ARUCO_DICT
from aruco_pose_estimation.pose_estimation import pose_estimation

# ROS2 message imports
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseArray, TransformStamped
from aruco_interfaces.msg import ArucoMarkers
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from tf2_ros import TransformBroadcaster

class ArucoNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("aruco_node")

        self.initialize_parameters()

        # Validating dictionary
        try:
            # Handle both old and new OpenCV attribute naming for dictionary access
            if hasattr(cv2.aruco, self.dictionary_id_name):
                dictionary_id = cv2.aruco.__getattribute__(self.dictionary_id_name)
            else:
                # Fallback for some 4.x versions
                dictionary_id = getattr(cv2.aruco, self.dictionary_id_name)
        except AttributeError:
            self.get_logger().error(f"bad aruco_dictionary_id: {self.dictionary_id_name}")
            return

        # Set up subscriptions
        self.info_sub = self.create_subscription(
            CameraInfo, self.info_topic, self.info_callback, qos_profile_sensor_data
        )

        if (bool(self.use_depth_input)):
            self.image_sub = message_filters.Subscriber(self, Image, self.image_topic, qos_profile=qos_profile_sensor_data)
            self.depth_image_sub = message_filters.Subscriber(self, Image, self.depth_image_topic, qos_profile=qos_profile_sensor_data)
            self.synchronizer = message_filters.ApproximateTimeSynchronizer(
                [self.image_sub, self.depth_image_sub], queue_size=10, slop=0.05
            )
            self.synchronizer.registerCallback(self.rgb_depth_sync_callback)
        else:
            self.image_sub = self.create_subscription(
                Image, self.image_topic, self.image_callback, qos_profile_sensor_data
            )

        # Set up publishers
        self.poses_pub = self.create_publisher(PoseArray, self.markers_visualization_topic, 10)
        self.markers_pub = self.create_publisher(ArucoMarkers, self.detected_markers_topic, 10)
        self.image_pub = self.create_publisher(Image, self.output_image_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None
        self.filtered_pose_by_id = {}

        # --- LEGACY API FIX FOR OPENCV 4.5.4 ---
        # Instead of ArucoDetector, we use the Dictionary and Parameters separately
        self.aruco_dictionary = cv2.aruco.Dictionary_get(dictionary_id)
        self.aruco_parameters = cv2.aruco.DetectorParameters_create()
        # We store them in a way the pose_estimation function expects for legacy
        self.aruco_detector = (self.aruco_dictionary, self.aruco_parameters) 

        self.bridge = CvBridge()

    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)
        self.get_logger().info("Camera info received and processed.")
        self.destroy_subscription(self.info_sub)

    def image_callback(self, img_msg: Image):
        if self.info_msg is None:
            return

        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="rgb8")
        markers = ArucoMarkers()
        pose_array = PoseArray()

        if self.camera_frame == "":
            markers.header.frame_id = self.info_msg.header.frame_id
            pose_array.header.frame_id = self.info_msg.header.frame_id
        else:
            markers.header.frame_id = self.camera_frame
            pose_array.header.frame_id = self.camera_frame

        markers.header.stamp = img_msg.header.stamp
        pose_array.header.stamp = img_msg.header.stamp

        # Call the pose estimation function
        # Note: We pass the tuple (dict, params) as aruco_detector
        frame, pose_array, markers = pose_estimation(
            rgb_frame=cv_image, 
            depth_frame=None,
            aruco_detector=self.aruco_detector,
            marker_size=self.marker_size, 
            matrix_coefficients=self.intrinsic_mat,
            distortion_coefficients=self.distortion, 
            pose_array=pose_array, 
            markers=markers
        )

        filtered_markers, filtered_pose_array = self.apply_pose_filter(
            markers=markers,
            stamp=img_msg.header.stamp,
            frame_id=markers.header.frame_id,
        )

        self.publish_robot_transform(filtered_markers)
        self.poses_pub.publish(filtered_pose_array)
        self.markers_pub.publish(filtered_markers)

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "rgb8"))


    def rgb_depth_sync_callback(self, rgb_msg: Image, depth_msg: Image):
        if self.info_msg is None:
            return

        cv_depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="16UC1")
        cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="rgb8")

        markers = ArucoMarkers()
        pose_array = PoseArray()

        if self.camera_frame == "":
            markers.header.frame_id = self.info_msg.header.frame_id
            pose_array.header.frame_id = self.info_msg.header.frame_id
        else:
            markers.header.frame_id = self.camera_frame
            pose_array.header.frame_id = self.camera_frame

        markers.header.stamp = rgb_msg.header.stamp
        pose_array.header.stamp = rgb_msg.header.stamp

        frame, pose_array, markers = pose_estimation(
            rgb_frame=cv_image, 
            depth_frame=cv_depth_image,
            aruco_detector=self.aruco_detector,
            marker_size=self.marker_size, 
            matrix_coefficients=self.intrinsic_mat,
            distortion_coefficients=self.distortion, 
            pose_array=pose_array, 
            markers=markers
        )

        filtered_markers, filtered_pose_array = self.apply_pose_filter(
            markers=markers,
            stamp=rgb_msg.header.stamp,
            frame_id=markers.header.frame_id,
        )

        self.publish_robot_transform(filtered_markers)
        self.poses_pub.publish(filtered_pose_array)
        self.markers_pub.publish(filtered_markers)

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "rgb8"))


    def stamp_to_seconds(self, stamp) -> float:
        return float(stamp.sec) + (float(stamp.nanosec) * 1e-9)


    def normalize_quaternion(self, quat: np.ndarray) -> np.ndarray:
        norm = np.linalg.norm(quat)
        if norm < 1e-12:
            return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        return quat / norm


    def make_zero_pose(self) -> Pose:
        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = 0.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0
        return pose


    def is_zero_pose(self, pose: Pose) -> bool:
        return (
            pose.position.x == 0.0
            and pose.position.y == 0.0
            and pose.position.z == 0.0
            and pose.orientation.x == 0.0
            and pose.orientation.y == 0.0
            and pose.orientation.z == 0.0
            and pose.orientation.w == 1.0
        )


    def smooth_pose(self, previous_pose: Pose, current_pose: Pose) -> Pose:
        smoothed = Pose()
        alpha = float(self.pose_filter_alpha)

        smoothed.position.x = (alpha * current_pose.position.x) + ((1.0 - alpha) * previous_pose.position.x)
        smoothed.position.y = (alpha * current_pose.position.y) + ((1.0 - alpha) * previous_pose.position.y)
        smoothed.position.z = (alpha * current_pose.position.z) + ((1.0 - alpha) * previous_pose.position.z)

        previous_quat = np.array([
            previous_pose.orientation.x,
            previous_pose.orientation.y,
            previous_pose.orientation.z,
            previous_pose.orientation.w,
        ], dtype=np.float64)
        current_quat = np.array([
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z,
            current_pose.orientation.w,
        ], dtype=np.float64)

        previous_quat = self.normalize_quaternion(previous_quat)
        current_quat = self.normalize_quaternion(current_quat)

        # Keep quaternion continuity to avoid sudden 180-degree flips.
        if np.dot(previous_quat, current_quat) < 0.0:
            current_quat = -current_quat

        blended_quat = self.normalize_quaternion(
            (alpha * current_quat) + ((1.0 - alpha) * previous_quat)
        )

        smoothed.orientation.x = float(blended_quat[0])
        smoothed.orientation.y = float(blended_quat[1])
        smoothed.orientation.z = float(blended_quat[2])
        smoothed.orientation.w = float(blended_quat[3])
        return smoothed


    def apply_pose_filter(self, markers: ArucoMarkers, stamp, frame_id: str):
        current_time = self.stamp_to_seconds(stamp)

        for idx, marker_id in enumerate(markers.marker_ids):
            marker_id = int(marker_id)
            if marker_id == 0:
                continue

            measured_pose = markers.poses[idx]
            if measured_pose.position.x == 0.0 and measured_pose.position.y == 0.0:
                continue

            if marker_id in self.filtered_pose_by_id:
                previous_pose = self.filtered_pose_by_id[marker_id]["pose"]
                filtered_pose = self.smooth_pose(previous_pose, measured_pose)
            else:
                filtered_pose = measured_pose

            self.filtered_pose_by_id[marker_id] = {
                "pose": filtered_pose,
                "last_seen": current_time,
            }

        filtered_markers = ArucoMarkers()
        filtered_pose_array = PoseArray()
        filtered_markers.header.stamp = stamp
        filtered_markers.header.frame_id = frame_id
        filtered_pose_array.header.stamp = stamp
        filtered_pose_array.header.frame_id = frame_id

        fixed_marker_ids = self.target_marker_ids
        for marker_id in fixed_marker_ids:
            filtered_markers.marker_ids.append(marker_id)

            if marker_id == 0:
                filtered_markers.poses.append(self.make_zero_pose())
                filtered_pose_array.poses.append(self.make_zero_pose())
                continue

            marker_state = self.filtered_pose_by_id.get(marker_id)
            if marker_state is None:
                filtered_markers.poses.append(self.make_zero_pose())
                filtered_pose_array.poses.append(self.make_zero_pose())
                continue

            age = current_time - marker_state["last_seen"]
            if age > float(self.marker_hold_time_sec):
                del self.filtered_pose_by_id[marker_id]
                filtered_markers.poses.append(self.make_zero_pose())
                filtered_pose_array.poses.append(self.make_zero_pose())
                continue

            filtered_markers.poses.append(marker_state["pose"])
            filtered_pose_array.poses.append(marker_state["pose"])

        return filtered_markers, filtered_pose_array


    def publish_robot_transform(self, markers: ArucoMarkers):
        transforms = []
        for i, m_id in enumerate(markers.marker_ids):
            # If m_id is 0, we can publish it relative to the camera frame if we have its tvec/rvec.
            # However, according to pose_estimation.py, all poses in 'markers.poses' are already 
            # relative to ID 0 if ID 0 is found. 
            # So for all m_id != 0, parent is marker_0.
            
            pose = markers.poses[i]
            if self.is_zero_pose(pose):
                continue

            transform = TransformStamped()
            transform.header.stamp = markers.header.stamp
            
            if m_id == 0:
                # This would be an identity transform if using marker_0 as parent.
                # Usually we skip it or use camera_link as parent for marker_0.
                # But since the user wants to ignore camera frame and care about relative 
                # to ID 0, we'll focus on the children of marker_0.
                continue

            transform.header.frame_id = "marker_0"
            transform.child_frame_id = f"marker_{m_id}"
            transform.transform.translation.x = pose.position.x
            transform.transform.translation.y = pose.position.y
            transform.transform.translation.z = pose.position.z
            transform.transform.rotation = pose.orientation
            transforms.append(transform)

        if transforms:
            self.tf_broadcaster.sendTransform(transforms)


    def initialize_parameters(self):
        self.declare_parameter("marker_size", 0.0625)
        self.declare_parameter("aruco_dictionary_id", "DICT_5X5_250")
        self.declare_parameter("use_depth_input", False)
        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("depth_image_topic", "/camera/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/camera_info")
        self.declare_parameter("camera_frame", "camera_link")
        self.declare_parameter("detected_markers_topic", "/aruco_markers")
        self.declare_parameter("markers_visualization_topic", "/aruco_poses")
        self.declare_parameter("output_image_topic", "/aruco_image")
        self.declare_parameter("pose_filter_alpha", 0.35)
        self.declare_parameter("marker_hold_time_sec", 0.30)
        self.declare_parameter("target_marker_ids", [0, 1, 2, 3, 4])

        self.marker_size = self.get_parameter("marker_size").value
        self.dictionary_id_name = self.get_parameter("aruco_dictionary_id").value
        self.use_depth_input = self.get_parameter("use_depth_input").value
        self.image_topic = self.get_parameter("image_topic").value
        self.depth_image_topic = self.get_parameter("depth_image_topic").value
        self.info_topic = self.get_parameter("camera_info_topic").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.detected_markers_topic = self.get_parameter("detected_markers_topic").value
        self.markers_visualization_topic = self.get_parameter("markers_visualization_topic").value
        self.output_image_topic = self.get_parameter("output_image_topic").value
        self.pose_filter_alpha = self.get_parameter("pose_filter_alpha").value
        self.marker_hold_time_sec = self.get_parameter("marker_hold_time_sec").value
        self.target_marker_ids = [int(marker_id) for marker_id in self.get_parameter("target_marker_ids").value]

def main():
    rclpy.init()
    node = ArucoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
