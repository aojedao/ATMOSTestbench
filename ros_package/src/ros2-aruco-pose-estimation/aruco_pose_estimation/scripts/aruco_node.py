#!/usr/bin/env python3
import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import message_filters

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

        if len(markers.marker_ids) > 0:
            self.poses_pub.publish(pose_array)
            self.markers_pub.publish(markers)
            self.publish_robot_transform(markers)

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "rgb8"))

    def rgb_depth_sync_callback(self, rgb_msg: Image, depth_msg: Image):
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

        if len(markers.marker_ids) > 0:
            self.poses_pub.publish(pose_array)
            self.markers_pub.publish(markers)
            self.publish_robot_transform(markers)

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "rgb8"))

    def publish_robot_transform(self, markers: ArucoMarkers):
        if 4 not in markers.marker_ids:
            return

        robot_index = markers.marker_ids.index(4)
        robot_pose = markers.poses[robot_index]

        transform = TransformStamped()
        transform.header.stamp = markers.header.stamp
        transform.header.frame_id = "marker_0"
        transform.child_frame_id = "marker_4"
        transform.transform.translation.x = robot_pose.position.x
        transform.transform.translation.y = robot_pose.position.y
        transform.transform.translation.z = robot_pose.position.z
        transform.transform.rotation = robot_pose.orientation

        self.tf_broadcaster.sendTransform(transform)

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

def main():
    rclpy.init()
    node = ArucoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
