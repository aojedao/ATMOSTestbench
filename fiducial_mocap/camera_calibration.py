#!/usr/bin/env python3
"""Calibrate a camera from chessboard images for the ATMOS testbench.

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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Capture chessboard corners and compute camera intrinsics. "
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
        default=9,
        help="Number of inner corners per chessboard row (default: 9).",
    )
    parser.add_argument(
        "--pattern-rows",
        type=int,
        default=6,
        help="Number of inner corners per chessboard column (default: 6).",
    )
    parser.add_argument(
        "--square-size-m",
        type=float,
        default=0.024,
        help="Physical chessboard square size in meters (default: 0.024).",
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
    grid = np.zeros((rows * cols, 3), dtype=np.float32)
    grid[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    grid *= square_size_m
    return grid


def detect_chessboard_corners(
    gray: np.ndarray, pattern_size: Tuple[int, int]
) -> Tuple[bool, Optional[np.ndarray]]:
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE

    if hasattr(cv2, "findChessboardCornersSB"):
        found, corners = cv2.findChessboardCornersSB(gray, pattern_size, flags)
    else:
        found, corners = cv2.findChessboardCorners(gray, pattern_size, flags)

    if not found:
        return False, None

    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        30,
        1e-3,
    )
    refined = cv2.cornerSubPix(
        gray,
        corners,
        winSize=(11, 11),
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
        found, corners = detect_chessboard_corners(gray, pattern_size)
        if not found or corners is None:
            print(f"[WARN] Chessboard not detected: {image_path.name}")
            continue

        object_points.append(object_template.copy())
        image_points.append(corners)
        image_size = (gray.shape[1], gray.shape[0])
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

    print("Live capture started.")
    print("Controls: 'c' capture frame when chessboard is detected, 'q' quit.")

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

            cv2.imshow("ATMOS Camera Calibration", display)
            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
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
        cap.release()
        cv2.destroyAllWindows()

    if image_size is None or len(image_points) < 3:
        raise RuntimeError(
            "Insufficient valid captures for calibration. Need at least 3 good frames."
        )

    return object_points, image_points, image_size


def compute_mean_reprojection_error(
    object_points: List[np.ndarray],
    image_points: List[np.ndarray],
    rvecs: List[np.ndarray],
    tvecs: List[np.ndarray],
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> float:
    total_error = 0.0
    for idx, obj_pts in enumerate(object_points):
        projected, _ = cv2.projectPoints(
            obj_pts,
            rvecs[idx],
            tvecs[idx],
            camera_matrix,
            dist_coeffs,
        )
        error = cv2.norm(image_points[idx], projected, cv2.NORM_L2) / len(projected)
        total_error += float(error)
    return total_error / max(1, len(object_points))


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

    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        object_points,
        image_points,
        image_size,
        None,
        None,
    )
    mean_error = compute_mean_reprojection_error(
        object_points,
        image_points,
        rvecs,
        tvecs,
        camera_matrix,
        dist_coeffs,
    )

    calibration_data = {
        "timestamp": timestamp,
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
    }

    calibration_name = f"camera_calibration_{timestamp}.yaml"
    calibration_path = output_dir / calibration_name
    with calibration_path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(calibration_data, f, sort_keys=False)

    print("Calibration complete.")
    print(f"- Valid images: {len(image_points)}")
    print(f"- RMS reprojection error: {rms:.6f}")
    print(f"- Mean reprojection error: {mean_error:.6f}")
    print(f"- Saved: {calibration_path.resolve()}")


if __name__ == "__main__":
    main()
