#!/usr/bin/env python3
"""Calibrate a fisheye camera from chessboard images for the ATMOS testbench.

This script supports two workflows:
1) Interactive capture from a live camera feed.
2) Offline calibration from an existing image directory.

Calibration outputs are timestamped to preserve historical records.
"""

from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np
import yaml


WINDOW_NAME = "ATMOS Fisheye Camera Calibration"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Capture chessboard corners and compute fisheye camera intrinsics. "
            "Results are written to a timestamped YAML file."
        )
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("fiducial_mocap/calibration"),
        help="Directory for calibration YAML and optional capture images.",
    )
    parser.add_argument(
        "--image-dir",
        type=Path,
        default=None,
        help="If set, load chessboard images from this directory instead of live capture.",
    )
    parser.add_argument(
        "--camera-index",
        type=int,
        default=0,
        help="Camera index for live capture mode (default: 0).",
    )
    parser.add_argument(
        "--pattern-cols",
        type=int,
        default=5,
        help="Number of inner corners per chessboard row (default: 9).",
    )
    parser.add_argument(
        "--pattern-rows",
        type=int,
        default=4,
        help="Number of inner corners per chessboard column (default: 6).",
    )
    parser.add_argument(
        "--square-size-m",
        type=float,
        default=0.035,
        help="Physical chessboard square size in meters (default: 0.035).",
    )
    parser.add_argument(
        "--required-frames",
        type=int,
        default=20,
        help="Required valid chessboard detections for calibration (default: 20).",
    )
    parser.add_argument(
        "--save-captures",
        action="store_true",
        help="Save accepted live-capture frames with timestamped filenames.",
    )
    return parser.parse_args()


def build_object_points(
    pattern_size: Tuple[int, int], square_size_m: float
) -> np.ndarray:
    cols, rows = pattern_size
    grid = np.zeros((1, rows * cols, 3), dtype=np.float32)
    grid[0, :, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    grid *= square_size_m
    return grid


def detect_chessboard_corners(
    gray: np.ndarray, pattern_size: Tuple[int, int]
) -> Tuple[bool, Optional[np.ndarray]]:
    flags = (
        cv2.CALIB_CB_ADAPTIVE_THRESH
        | cv2.CALIB_CB_FAST_CHECK
        | cv2.CALIB_CB_NORMALIZE_IMAGE
    )
    found, corners = cv2.findChessboardCorners(gray, pattern_size, flags)

    if not found:
        return False, None

    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        30,
        0.1,
    )
    refined = cv2.cornerSubPix(
        gray,
        corners,
        winSize=(3, 3),
        zeroZone=(-1, -1),
        criteria=criteria,
    )
    return True, refined


def collect_points_from_images(
    image_paths: List[Path], pattern_size: Tuple[int, int], object_template: np.ndarray
) -> Tuple[List[np.ndarray], List[np.ndarray], Tuple[int, int]]:
    object_points: List[np.ndarray] = []
    image_points: List[np.ndarray] = []
    image_size: Optional[Tuple[int, int]] = None

    for image_path in image_paths:
        frame = cv2.imread(str(image_path))
        if frame is None:
            print(f"[WARN] Skipping unreadable image: {image_path}")
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        current_size = (gray.shape[1], gray.shape[0])
        if image_size is None:
            image_size = current_size
        elif image_size != current_size:
            print(
                f"[WARN] Skipping {image_path.name}: size {current_size} "
                f"does not match first image size {image_size}."
            )
            continue

        found, corners = detect_chessboard_corners(gray, pattern_size)
        if not found or corners is None:
            print(f"[WARN] Chessboard not detected: {image_path.name}")
            continue

        object_points.append(object_template.copy())
        image_points.append(corners)
        image_size = current_size
        print(f"[OK] Accepted: {image_path.name}")

    if image_size is None:
        raise RuntimeError("No valid images with detectable chessboard corners were found.")

    return object_points, image_points, image_size


def collect_points_from_camera(
    camera_index: int,
    pattern_size: Tuple[int, int],
    object_template: np.ndarray,
    required_frames: int,
    save_captures: bool,
    captures_dir: Path,
    timestamp: str,
) -> Tuple[List[np.ndarray], List[np.ndarray], Tuple[int, int]]:
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera index {camera_index}.")

    object_points: List[np.ndarray] = []
    image_points: List[np.ndarray] = []
    image_size: Optional[Tuple[int, int]] = None

    print("Live fisheye capture started.")
    print("Controls: 'c' capture frame when chessboard is detected, 'q' or ESC quit.")

    try:
        while len(image_points) < required_frames:
            ok, frame = cap.read()
            if not ok:
                print("[WARN] Failed to read frame from camera.")
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = detect_chessboard_corners(gray, pattern_size)

            display = frame.copy()
            status = "FOUND" if found else "NOT FOUND"
            color = (0, 200, 0) if found else (0, 0, 220)
            cv2.putText(
                display,
                f"Chessboard: {status}",
                (16, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                color,
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                display,
                f"Captured: {len(image_points)}/{required_frames}",
                (16, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                display,
                "Press c to capture, q to quit",
                (16, 90),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

            if found and corners is not None:
                cv2.drawChessboardCorners(display, pattern_size, corners, found)

            cv2.imshow(WINDOW_NAME, display)
            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), 27):
                break

            if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                break

            if key == ord("c"):
                if not found or corners is None:
                    print("[WARN] Cannot capture: chessboard not detected.")
                    continue

                object_points.append(object_template.copy())
                image_points.append(corners)
                image_size = (gray.shape[1], gray.shape[0])
                idx = len(image_points)
                print(f"[OK] Captured frame {idx}/{required_frames}")

                if save_captures:
                    captures_dir.mkdir(parents=True, exist_ok=True)
                    capture_name = f"calib_capture_{timestamp}_{idx:03d}.png"
                    cv2.imwrite(str(captures_dir / capture_name), frame)

    finally:
        if cap.isOpened():
            cap.release()
        try:
            cv2.destroyWindow(WINDOW_NAME)
        except cv2.error:
            pass
        cv2.destroyAllWindows()
        cv2.waitKey(1)

    if image_size is None or len(image_points) < 3:
        raise RuntimeError(
            "Insufficient valid captures for calibration. Need at least 3 good frames."
        )

    return object_points, image_points, image_size


def compute_mean_reprojection_error_fisheye(
    object_points: List[np.ndarray],
    image_points: List[np.ndarray],
    rvecs: List[np.ndarray],
    tvecs: List[np.ndarray],
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> float:
    total_error = 0.0
    for idx, obj_pts in enumerate(object_points):
        projected, _ = cv2.fisheye.projectPoints(
            obj_pts,
            rvecs[idx],
            tvecs[idx],
            camera_matrix,
            dist_coeffs,
        )

        # OpenCV can return corner arrays as Nx1x2 or 1xNx2 depending on call path.
        # Normalize both to Nx2 so cv2.norm receives same-sized inputs.
        observed_xy = np.asarray(image_points[idx], dtype=np.float64).reshape(-1, 2)
        projected_xy = np.asarray(projected, dtype=np.float64).reshape(-1, 2)

        if observed_xy.shape != projected_xy.shape:
            raise RuntimeError(
                f"Reprojection shape mismatch at frame {idx}: "
                f"observed {observed_xy.shape}, projected {projected_xy.shape}."
            )

        error = cv2.norm(observed_xy, projected_xy, cv2.NORM_L2) / len(projected_xy)
        total_error += float(error)
    return total_error / max(1, len(object_points))


def calibrate_fisheye(
    object_points: List[np.ndarray],
    image_points: List[np.ndarray],
    image_size: Tuple[int, int],
) -> Tuple[float, np.ndarray, np.ndarray, List[np.ndarray], List[np.ndarray]]:
    num_ok = len(object_points)
    camera_matrix = np.zeros((3, 3), dtype=np.float64)
    dist_coeffs = np.zeros((4, 1), dtype=np.float64)
    rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for _ in range(num_ok)]
    tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for _ in range(num_ok)]

    calibration_flags = (
        cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC
        + cv2.fisheye.CALIB_CHECK_COND
        + cv2.fisheye.CALIB_FIX_SKEW
    )

    objp64 = [np.asarray(o, dtype=np.float64) for o in object_points]
    imgp64 = [np.asarray(i, dtype=np.float64) for i in image_points]

    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.fisheye.calibrate(
        objp64,
        imgp64,
        image_size,
        camera_matrix,
        dist_coeffs,
        rvecs,
        tvecs,
        calibration_flags,
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6),
    )
    return rms, camera_matrix, dist_coeffs, rvecs, tvecs


def main() -> None:
    args = parse_args()
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    pattern_size = (args.pattern_cols, args.pattern_rows)
    object_template = build_object_points(pattern_size, args.square_size_m)

    if args.image_dir is not None:
        image_dir = args.image_dir
        if not image_dir.exists():
            raise FileNotFoundError(f"Image directory does not exist: {image_dir}")

        image_paths = sorted(
            [
                p
                for p in image_dir.iterdir()
                if p.suffix.lower() in {".png", ".jpg", ".jpeg", ".bmp", ".tiff"}
            ]
        )
        if not image_paths:
            raise RuntimeError(f"No image files found in directory: {image_dir}")

        object_points, image_points, image_size = collect_points_from_images(
            image_paths=image_paths,
            pattern_size=pattern_size,
            object_template=object_template,
        )
    else:
        captures_dir = output_dir / "captures"
        object_points, image_points, image_size = collect_points_from_camera(
            camera_index=args.camera_index,
            pattern_size=pattern_size,
            object_template=object_template,
            required_frames=args.required_frames,
            save_captures=args.save_captures,
            captures_dir=captures_dir,
            timestamp=timestamp,
        )

    if len(image_points) < 3:
        raise RuntimeError("Need at least 3 valid corner sets to calibrate camera.")

    rms, camera_matrix, dist_coeffs, rvecs, tvecs = calibrate_fisheye(
        object_points,
        image_points,
        image_size,
    )
    mean_error = compute_mean_reprojection_error_fisheye(
        object_points,
        image_points,
        rvecs,
        tvecs,
        camera_matrix,
        dist_coeffs,
    )

    calibration_data = {
        "timestamp": timestamp,
        "camera_model": "fisheye",
        "dim": [int(image_size[0]), int(image_size[1])],
        "image_width": int(image_size[0]),
        "image_height": int(image_size[1]),
        "pattern_cols": int(args.pattern_cols),
        "pattern_rows": int(args.pattern_rows),
        "square_size_m": float(args.square_size_m),
        "num_valid_images": int(len(image_points)),
        "rms_reprojection_error": float(rms),
        "mean_reprojection_error": float(mean_error),
        "camera_matrix": camera_matrix.tolist(),
        "distortion_coefficients": dist_coeffs.tolist(),
        "K": camera_matrix.tolist(),
        "D": dist_coeffs.tolist(),
    }

    calibration_name = f"camera_calibration_{timestamp}.yaml"
    calibration_path = output_dir / calibration_name
    with calibration_path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(calibration_data, f, sort_keys=False)

    print("Fisheye calibration complete.")
    print(f"- Valid images: {len(image_points)}")
    print(f"- RMS reprojection error: {rms:.6f}")
    print(f"- Mean reprojection error: {mean_error:.6f}")
    print(f"- DIM={image_size}")
    print(f"- K=np.array({camera_matrix.tolist()})")
    print(f"- D=np.array({dist_coeffs.tolist()})")
    print(f"- Saved: {calibration_path.resolve()}")


if __name__ == "__main__":
    main()
