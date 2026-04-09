# ATMOS Testbench

Welcome to the ATMOS Testbench. This repository contains the framework and tooling necessary to validate the motion, speed, and acceleration of the ATMOS robot. 

We generate our own "MoCap" ground truth data using a top-down camera system that tracks ArUco fiducial markers. This ground truth is then actively compared against logs collected from the robot's onboard Pixhawk and RealSense VIO system.

## Where to Look First
If you are an agent, developer, or a user looking for directions on what to do next, please consult the **[ROADMAP.md](./ROADMAP.md)** file. It is the living document that details our progress, the strict step-by-step procedure we are following, and our immediate next tasks.

## Project Structure
- `fiducial_mocap/`: Contains all code for the ground-truth top-down camera tracking, ArUco marker generation, and camera calibration.
- `pixhawk_data/`: Contains all parsing logic and data handlers for the Pixhawk logs and RealSense VIO data.
- `logs/`: Directory where generated datasets and results are stored. All log files contain timestamps in their names to avoid overwriting historical data.

## Getting Started
*(Instructions here are currently in development as we build out the modules outlined in the ROADMAP.md).*

Please interact with the developer step-by-step; do not proceed with generating massive chunks of code without prior approval or reviewing the roadmap.

## Fiducial Marker Generation (Phase 2.1)

Use the marker generator to create 5 ArUco markers (4 world corners + 1 robot marker).

Command:

```bash
python3 fiducial_mocap/generate_markers.py
```

What it generates (all timestamped):
- Individual marker cards sized for 15 cm x 15 cm printing, each with a black cut outline.
- A labeled contact sheet that shows corner/robot names with marker IDs.
- A CSV manifest with role-to-ID mapping and output filenames.

Default role map:
- `CORNER_NW` -> ArUco ID 0
- `CORNER_NE` -> ArUco ID 1
- `CORNER_SW` -> ArUco ID 2
- `CORNER_SE` -> ArUco ID 3
- `ROBOT` -> ArUco ID 4

Notes:
- Default output directory is `fiducial_mocap/markers/`.
- Default render DPI is 300. For accurate physical size, print at 100% scale (no fit-to-page).

## Camera Calibration (Phase 2.2)

For optimal accuracy, we use the standard `ros2 cameracalibrator` tool to obtain camera intrinsics.

1.  **Calibration**: Run the calibrator with a standard checkerboard.
2.  **Implementation**: The resulting parameters (K and D matrices) are manually plugged into the camera driver.

## Camera Driver: `camera_direct.py`

Due to compatibility issues with the **Insta360 Ace Pro 2** camera, we use a custom driver called `camera_direct.py`. This script handles the GStreamer pipeline and injects the calibration parameters directly into the ROS 2 `CameraInfo` messages.

To run the camera driver:
```bash
python3 camera_direct.py
```

## MoCap Tracker (Phase 2.3)

We use the `aruco_pose_estimation` ROS 2 package for real-time tracking.

### 1. Run the ArUco Node
```bash
ros2 run aruco_pose_estimation aruco_node.py --ros-args \
  -p marker_size:=0.075 \
  -p aruco_dictionary_id:="DICT_4X4_50" \
  -p image_topic:="/image_raw" \
  -p camera_info_topic:="/camera_info"
```

### 2. Visualize the Output
```bash
ros2 run rqt_image_view rqt_image_view
```
