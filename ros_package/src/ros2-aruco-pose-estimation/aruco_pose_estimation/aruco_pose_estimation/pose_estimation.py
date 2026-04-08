#!/usr/bin/env python3

# Code taken and readapted from:
# https://github.com/GSNCodes/ArUCo-Markers-Pose-Estimation-Generation-Python/tree/main

# Python imports
import numpy as np
import cv2
import tf_transformations

# ROS2 imports
from rclpy.impl import rcutils_logger

# ROS2 message imports
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from aruco_interfaces.msg import ArucoMarkers

# utils import python code
from aruco_pose_estimation.utils import aruco_display


def pose_estimation(rgb_frame: np.array, depth_frame: np.array, aruco_detector: tuple, marker_size: float,
                    matrix_coefficients: np.array, distortion_coefficients: np.array,
                    pose_array: PoseArray, markers: ArucoMarkers) -> list:
    '''
    rgb_frame - Frame from the RGB camera stream
    depth_frame - Depth frame from the depth camera stream
    matrix_coefficients - Intrinsic matrix of the calibrated camera
    distortion_coefficients - Distortion coefficients associated with your camera
    pose_array - PoseArray message to be published
    markers - ArucoMarkers message to be published
    '''

    # Legacy OpenCV 4.5.4 Fix
    aruco_dictionary, aruco_parameters = aruco_detector
    corners, marker_ids, rejected = cv2.aruco.detectMarkers(rgb_frame, aruco_dictionary, parameters=aruco_parameters)

    frame_processed = rgb_frame.copy()
    logger = rcutils_logger.RcutilsLogger(name="aruco_node")

    # If NO markers are detected, return early
    if len(corners) == 0 or marker_ids is None:
        return frame_processed, pose_array, markers

    logger.debug("Detected {} markers.".format(len(corners)))

    # PHASE 1: Extract all absolute poses into a dictionary
    detected_poses = {}
    
    for i, marker_id in enumerate(marker_ids):
        m_id = int(marker_id[0])
        
        # Estimate absolute pose relative to camera
        tvec, rvec, quat = my_estimatePoseSingleMarkers(
            corners=corners[i], 
            marker_size=marker_size,
            camera_matrix=matrix_coefficients,
            distortion=distortion_coefficients
        )
        
        detected_poses[m_id] = {
            'tvec': tvec,
            'rvec': rvec,
            'quat': quat,
            'corners': corners[i]
        }

        # Draw 2D bounding boxes and absolute axes on the frame
        frame_processed = aruco_display(corners=[corners[i]], ids=np.array([[m_id]]), image=frame_processed)
        frame_processed = cv2.drawFrameAxes(image=frame_processed, cameraMatrix=matrix_coefficients,
                                            distCoeffs=distortion_coefficients, rvec=rvec, tvec=tvec,
                                            length=0.05, thickness=3)

    # PHASE 2: Build the Planar Reference Frame (Origin at ID 0)
    if 0 in detected_poses:
        # 1. Set Origin
        origin_tvec = detected_poses[0]['tvec'].flatten()

        # 2. Define X-Axis (Vector from 0 to 1)
        if 1 in detected_poses:
            p1 = detected_poses[1]['tvec'].flatten()
            v_x = p1 - origin_tvec
        else:
            # Fallback if 1 is hidden: use Marker 0's native rotation
            rot_0, _ = cv2.Rodrigues(detected_poses[0]['rvec'])
            v_x = rot_0[:, 0] 
            
        v_x = v_x / np.linalg.norm(v_x) # Normalize

        # 3. Define Y-Axis (Vector from 0 to 3)
        if 3 in detected_poses:
            p3 = detected_poses[3]['tvec'].flatten()
            v_y_temp = p3 - origin_tvec
        else:
            rot_0, _ = cv2.Rodrigues(detected_poses[0]['rvec'])
            v_y_temp = rot_0[:, 1]

        # 4. Gram-Schmidt process to guarantee perfect 90-degree orthogonal axes
        v_z = np.cross(v_x, v_y_temp)
        v_z = v_z / np.linalg.norm(v_z)

        v_y = np.cross(v_z, v_x)
        v_y = v_y / np.linalg.norm(v_y)

        # 5. Build 4x4 Transformation Matrix (Camera to Plane)
        T_cam_to_plane = np.eye(4)
        T_cam_to_plane[0:3, 0] = v_x
        T_cam_to_plane[0:3, 1] = v_y
        T_cam_to_plane[0:3, 2] = v_z
        T_cam_to_plane[0:3, 3] = origin_tvec

        # 6. Invert to get (Plane to Camera) for relative calculations
        T_plane_to_cam = np.linalg.inv(T_cam_to_plane)

        # PHASE 3 & 4: Calculate relative positions and publish
        for m_id, data in detected_poses.items():
            # Extract absolute camera translation as 4x1 vector
            p_cam = np.array([data['tvec'][0,0], data['tvec'][1,0], data['tvec'][2,0], 1.0])
            
            # MULTIPLY: Convert to Relative Frame
            p_rel = np.dot(T_plane_to_cam, p_cam)

            # Convert rotations to relative frame
            rot_cam, _ = cv2.Rodrigues(data['rvec'])
            rot_rel = np.dot(T_plane_to_cam[0:3, 0:3], rot_cam)
            
            rot_rel_4x4 = np.eye(4, dtype=np.float32)
            rot_rel_4x4[0:3, 0:3] = rot_rel
            quat_rel = tf_transformations.quaternion_from_matrix(rot_rel_4x4)
            quat_rel = quat_rel / np.linalg.norm(quat_rel)

            # Construct ROS Pose Message (Now in ID 0's Coordinate System!)
            pose = Pose()
            pose.position.x = float(p_rel[0])
            pose.position.y = float(p_rel[1])
            pose.position.z = float(p_rel[2]) # Should be close to 0.0 for markers on the plane
            
            pose.orientation.x = float(quat_rel[0])
            pose.orientation.y = float(quat_rel[1])
            pose.orientation.z = float(quat_rel[2])
            pose.orientation.w = float(quat_rel[3])

            pose_array.poses.append(pose)
            markers.poses.append(pose)
            markers.marker_ids.append(m_id)

            # Specifically target ID 4 to report its X and Y location
            if m_id == 4:
                logger.info("--------------------------------------------------")
                logger.info("🎯 TARGET ID 4 FOUND")
                logger.info(f"Location relative to ID 0 Origin:")
                logger.info(f"X: {p_rel[0]:.4f} meters")
                logger.info(f"Y: {p_rel[1]:.4f} meters")
                logger.info(f"Z: {p_rel[2]:.4f} meters (Height off plane)")
                logger.info("--------------------------------------------------")

    else:
        logger.warn("Waiting for Marker 0 (Origin) to establish the planar coordinate system...")
        # If 0 isn't visible, we skip publishing to enforce the strict relative frame requirement.

    return frame_processed, pose_array, markers


def my_estimatePoseSingleMarkers(corners, marker_size, camera_matrix, distortion) -> tuple:
    '''
    This will estimate the rvec and tvec for each of the marker corners detected
    '''
    marker_points = np.array([[-marker_size / 2.0, marker_size / 2.0, 0],
                              [marker_size / 2.0, marker_size / 2.0, 0],
                              [marker_size / 2.0, -marker_size / 2.0, 0],
                              [-marker_size / 2.0, -marker_size / 2.0, 0]], dtype=np.float32)

    # solvePnP returns the rotation and translation vectors
    retval, rvec, tvec = cv2.solvePnP(objectPoints=marker_points, imagePoints=corners,
                                        cameraMatrix=camera_matrix, distCoeffs=distortion, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    rvec = rvec.reshape(3, 1)
    tvec = tvec.reshape(3, 1)
       
    rot, jacobian = cv2.Rodrigues(rvec)
    rot_matrix = np.eye(4, dtype=np.float32)
    rot_matrix[0:3, 0:3] = rot

    # convert rotation matrix to quaternion
    quaternion = tf_transformations.quaternion_from_matrix(rot_matrix)
    norm_quat = np.linalg.norm(quaternion)
    quaternion = quaternion / norm_quat

    return tvec, rvec, quaternion


def depth_to_pointcloud_centroid(depth_image: np.array, intrinsic_matrix: np.array,
                                 corners: np.array) -> np.array:
    """
    Retained original function to maintain code dependencies.
    """
    height, width = depth_image.shape
    corners_indices = np.array([(int(x), int(y)) for x, y in corners[0]])

    for x, y in corners_indices:
        if x < 0 or x >= width or y < 0 or y >= height:
            raise ValueError("One or more corners are outside the image bounds.")

    x_min = int(min(corners_indices[:, 0]))
    x_max = int(max(corners_indices[:, 0]))
    y_min = int(min(corners_indices[:, 1]))
    y_max = int(max(corners_indices[:, 1]))

    points = []
    for x in range(x_min, x_max):
        for y in range(y_min, y_max):
            if is_pixel_in_polygon(pixel=(x, y), corners=corners_indices):
                points.append([x, y, depth_image[y, x]])

    points = np.array(points, dtype=np.uint16)
   
    pointcloud = []
    for x, y, d in points:
        z = d / 1000.0
        x = (x - intrinsic_matrix[0, 2]) * z / intrinsic_matrix[0, 0]
        y = (y - intrinsic_matrix[1, 2]) * z / intrinsic_matrix[1, 1]
        pointcloud.append([x, y, z])

    centroid = np.mean(np.array(pointcloud, dtype=np.uint16), axis=0)
    return centroid


def is_pixel_in_polygon(pixel: tuple, corners: np.array) -> bool:
    """
    Retained original function to maintain code dependencies.
    """
    num_intersections = 0
    for i in range(len(corners)):
        x1, y1 = corners[i]
        x2, y2 = corners[(i + 1) % len(corners)]

        if (y1 <= pixel[1] < y2) or (y2 <= pixel[1] < y1):
            x_intersection = (x2 - x1) * (pixel[1] - y1) / (y2 - y1) + x1
            if x_intersection > pixel[0]:
                num_intersections += 1

    return num_intersections % 2 == 1
