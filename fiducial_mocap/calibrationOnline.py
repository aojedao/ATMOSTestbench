#!/usr/bin/env python3

import argparse
import glob
from pathlib import Path

import cv2
import numpy as np


CHECKERBOARD = (4, 5)
SUBPIX_CRITERIA = (
    cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
    30,
    0.1,
)
CALIBRATION_FLAGS = (
    cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC
    + cv2.fisheye.CALIB_CHECK_COND
    + cv2.fisheye.CALIB_FIX_SKEW
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Fisheye camera calibration from chessboard images.")
    parser.add_argument(
        "--images-glob",
        type=str,
        default="fiducial_mocap/calibration/pictures/*.jpeg",
        help="Glob pattern for calibration images.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

    img_shape = None
    objpoints = []
    imgpoints = []

    images = sorted(glob.glob(args.images_glob))
    if not images:
        print(f"No images found. Check --images-glob: {args.images_glob}")
        return 1

    for fname in images:
        img = cv2.imread(fname)
        if img is None:
            print(f"Skipping unreadable image: {fname}")
            continue

        print(f"Processing {Path(fname).name}: {img.shape}")

        if img_shape is None:
            img_shape = img.shape[:2]
        elif img_shape != img.shape[:2]:
            raise ValueError("All images must share the same size.")

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(
            gray,
            CHECKERBOARD,
            cv2.CALIB_CB_ADAPTIVE_THRESH
            + cv2.CALIB_CB_FAST_CHECK
            + cv2.CALIB_CB_NORMALIZE_IMAGE,
        )

        if ret:
            objpoints.append(objp)
            cv2.cornerSubPix(gray, corners, (3, 3), (-1, -1), SUBPIX_CRITERIA)
            imgpoints.append(corners)

    n_ok = len(objpoints)
    if n_ok == 0 or img_shape is None:
        print("No valid checkerboard detections. Calibration cannot run.")
        return 1

    k = np.zeros((3, 3))
    d = np.zeros((4, 1))
    rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for _ in range(n_ok)]
    tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for _ in range(n_ok)]

    rms, _, _, _, _ = cv2.fisheye.calibrate(
        objpoints,
        imgpoints,
        img_shape[::-1],
        k,
        d,
        rvecs,
        tvecs,
        CALIBRATION_FLAGS,
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6),
    )

    print(f"Found {n_ok} valid images for calibration")
    print(f"RMS={rms}")
    print(f"DIM={img_shape[::-1]}")
    print(f"K=np.array({k.tolist()})")
    print(f"D=np.array({d.tolist()})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())