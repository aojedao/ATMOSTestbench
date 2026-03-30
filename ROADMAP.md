# ATMOS Testbench Roadmap & Guidelines

This document serves as the central tracking file and roadmap for the ATMOS Testbench project. It is intended for both the developer and AI agents to understand the current state of the project, the step-by-step process, and what needs to be implemented next.

## Core Directives for Agents
- **Highly Interactive Process**: Do NOT pre-generate the entire codebase at once. Each module/component must be developed step-by-step. Receive approval from the user before finalizing logic, making major design decisions, or moving to the next module.
- **Log Formatting**: **ALL** scripts that produce logs or output data (images, CSVs, tracking data) MUST include a timestamp in the filename (e.g., `mocap_log_20260330_143000.csv`) so that historical data is preserved.
- **Documentation**: Provide clear, standard docstrings and keep the `README.md` updated as the primary reference manual.

---

## Roadmap

### Phase 1: Environment & Architecture Setup
- [x] Establish the initial project structure (`fiducial_mocap/`, `pixhawk_data/`).
- [x] Set up `ROADMAP.md` (this file) and update `README.md`.
- [ ] Define the `requirements.txt` based on the agreed-upon tech stack.

### Phase 2: Fiducial MoCap System (`fiducial_mocap/`)
- [x] **2.1 Marker Generation**: Create script to generate the ArUco markers (4 for reference, 1 for the ATMOS robot). Ensure output files are safely timestamped or version-controlled.
- [x] **2.2 Camera Calibration**: Develop a script to capture chessboard patterns and calculate the camera intrinsic matrix. Save the matrix locally (e.g., `calibration.yaml`).
- [ ] **2.3 Tracker Logic**: Develop the main `mocap_tracker.py` to process the live camera feed, identify the 4 ground markers as the world frame, project the robot marker, and timestamp/log its X, Y, and Yaw data.
- [ ] **2.4 Validation/Testing**: Test the tracker manually and confirm real-time visualization and log generation are working.

### Phase 3: Pixhawk Data Integration (`pixhawk_data/`)
- [ ] **3.1 Parsing Scripts**: Create utility scripts to parse Pixhawk telemetry data logs and VIO data.
- [ ] **3.2 Format Standardization**: Ensure output formats match or can be easily synced with the standard MoCap timestamps and data structure.

### Phase 4: Data Comparison & Analysis
- [ ] **4.1 Synchronization**: Develop logic to read the MoCap logs and Pixhawk logs, and interpolate/align them using timestamp matching.
- [ ] **4.2 Processing/Visualization**: Plot distance, velocity, and acceleration comparisons to validate ground-truth motion vs. onboard telemetry.
