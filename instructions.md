# Camera Calibration and Setup

This document describes the process for calibrating the camera and running the ArUco pose estimation system.

## Camera Calibration

To obtain accurate pose estimation, we use the standard `ros2 cameracalibrator` tool.

1.  **Calibration Process**: Run the ROS 2 camera calibrator with a checkerboard to generate the camera parameters (Intrinsic Matrix `K`, Distortion Coefficients `D`, etc.).
2.  **Implementation**: Once the parameters are obtained, they are plugged directly into the camera driver node.

## Camera Driver: `camera_direct.py`

Due to compatibility issues with the **Insta360 Ace Pro 2** camera and standard ROS 2 camera drivers, we developed a custom "driver" called `camera_direct.py`.

This driver:
- Uses a robust GStreamer pipeline to access the camera via `/dev/video0`.
- Decodes the JPEG stream at 640x360 @ 30fps.
- Manually publishes both the `/image_raw` and `/camera_info` topics.
- Injects the calibration parameters directly into the `CameraInfo` message.

To run the camera driver:
```bash
python3 camera_direct.py
```

## Running ArUco Pose Estimation

Once the camera driver is running and publishing the calibrated image and info topics, you can start the ArUco node and visualization.

### 1. Start the ArUco Node
Run the following command to start the pose estimation. Adjust the `marker_size` if necessary (current value is 7.5cm).

```bash
ros2 run aruco_pose_estimation aruco_node.py --ros-args \
  -p marker_size:=0.075 \
  -p aruco_dictionary_id:="DICT_4X4_50" \
  -p image_topic:="/image_raw" \
  -p camera_info_topic:="/camera_info"
```

### 2. Visualize the Output
To see the camera stream with the detected markers and axes, run:

```bash
ros2 run rqt_image_view rqt_image_view
```
