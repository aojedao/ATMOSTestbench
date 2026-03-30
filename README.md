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

Use the calibration script to estimate camera intrinsics from a chessboard pattern.

Live capture mode (interactive):

```bash
python3 fiducial_mocap/camera_calibration.py --save-captures
```

Offline mode from existing images:

```bash
python3 fiducial_mocap/camera_calibration.py --image-dir path/to/calib_images
```

Defaults:
- Chessboard inner corners: `9 x 6`
- Square size: `0.024 m`
- Required valid frames (live mode): `20`
- Output directory: `fiducial_mocap/calibration/`

Output:
- Timestamped calibration YAML, e.g. `camera_calibration_YYYYMMDD_HHMMSS.yaml`
- Optional timestamped capture images when `--save-captures` is enabled

## MoCap Tracker Surface Visualization (Phase 2.3 - Step 1)

Use the tracker script to visualize the table surface from 4 corner markers and draw
an XYZ reference frame at the back-left corner.

Default run:

```bash
python3 fiducial_mocap/mocap_tracker.py --camera-index 0
```

Example with custom table marker IDs (for IDs 1,3,5,7):

```bash
python3 fiducial_mocap/mocap_tracker.py --corner-ids 1,3,5,7
```

Notes:
- Corner ID order is: `back_left,back_right,front_right,front_left`.
- The script auto-loads the newest calibration file from `fiducial_mocap/calibration/` unless `--calibration-file` is provided.
- Press `q` to exit.
