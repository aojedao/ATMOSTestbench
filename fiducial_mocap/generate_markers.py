#!/usr/bin/env python3
"""Generate printable ArUco markers for the ATMOS testbench.

Outputs are timestamped to avoid overwriting historical files.

Default marker role map:
- ID 0 -> CORNER_NW
- ID 1 -> CORNER_NE
- ID 2 -> CORNER_SW
- ID 3 -> CORNER_SE
- ID 4 -> ROBOT
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import List

import cv2
import numpy as np


@dataclass(frozen=True)
class MarkerSpec:
    role: str
    marker_id: int


DEFAULT_MARKERS: List[MarkerSpec] = [
    MarkerSpec(role="CORNER_NW", marker_id=0),
    MarkerSpec(role="CORNER_NE", marker_id=1),
    MarkerSpec(role="CORNER_SW", marker_id=2),
    MarkerSpec(role="CORNER_SE", marker_id=3),
    MarkerSpec(role="ROBOT", marker_id=4),
]


def cm_to_px(value_cm: float, dpi: int) -> int:
    return int(round((value_cm / 2.54) * dpi))


def make_marker_tile(
    marker_id: int,
    tile_size_px: int,
    quiet_zone_ratio: float,
    cut_line_thickness_px: int,
    aruco_dict: cv2.aruco.Dictionary,
) -> np.ndarray:
    tile = np.full((tile_size_px, tile_size_px), 255, dtype=np.uint8)

    quiet_zone_px = max(8, int(round(tile_size_px * quiet_zone_ratio)))
    marker_size_px = tile_size_px - (2 * quiet_zone_px)
    if marker_size_px <= 0:
        raise ValueError("Marker size became non-positive. Reduce quiet-zone ratio.")

    marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size_px)
    y0 = quiet_zone_px
    y1 = y0 + marker_size_px
    x0 = quiet_zone_px
    x1 = x0 + marker_size_px
    tile[y0:y1, x0:x1] = marker_img

    # Cut line is drawn on the final 15x15 cm card perimeter.
    cv2.rectangle(
        tile,
        (0, 0),
        (tile_size_px - 1, tile_size_px - 1),
        color=0,
        thickness=cut_line_thickness_px,
    )

    return tile


def build_contact_sheet(
    tiles: List[np.ndarray],
    labels: List[str],
    tile_size_px: int,
    dpi: int,
) -> np.ndarray:
    cols = 3
    rows = int(np.ceil(len(tiles) / cols))

    margin_px = cm_to_px(0.7, dpi)
    label_area_px = cm_to_px(1.3, dpi)

    cell_w = tile_size_px + (2 * margin_px)
    cell_h = tile_size_px + label_area_px + (2 * margin_px)

    sheet_h = rows * cell_h
    sheet_w = cols * cell_w
    sheet = np.full((sheet_h, sheet_w), 255, dtype=np.uint8)

    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 1.2
    thickness = 2

    for idx, (tile, label) in enumerate(zip(tiles, labels)):
        row = idx // cols
        col = idx % cols

        x_start = col * cell_w + margin_px
        y_start = row * cell_h + margin_px
        x_end = x_start + tile_size_px
        y_end = y_start + tile_size_px

        sheet[y_start:y_end, x_start:x_end] = tile

        label_y = y_end + int(label_area_px * 0.6)
        (label_w, _), _ = cv2.getTextSize(label, font, font_scale, thickness)
        label_x = x_start + max(0, (tile_size_px - label_w) // 2)
        cv2.putText(
            sheet,
            label,
            (label_x, label_y),
            font,
            font_scale,
            color=0,
            thickness=thickness,
            lineType=cv2.LINE_AA,
        )

    return sheet


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Generate 5 ArUco markers for ATMOS with 15x15 cm cards, "
            "black cut outline, and timestamped output files."
        )
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("fiducial_mocap/markers"),
        help="Directory where generated marker files are saved.",
    )
    parser.add_argument(
        "--marker-size-cm",
        type=float,
        default=15.0,
        help="Physical marker card size in cm (default: 15.0).",
    )
    parser.add_argument(
        "--dpi",
        type=int,
        default=300,
        help="Raster DPI used for printable outputs (default: 300).",
    )
    parser.add_argument(
        "--quiet-zone-ratio",
        type=float,
        default=0.06,
        help=(
            "White border ratio around marker body inside each 15x15 card "
            "(default: 0.06)."
        ),
    )
    parser.add_argument(
        "--dictionary",
        type=str,
        default="DICT_4X4_50",
        help="OpenCV ArUco dictionary name (default: DICT_4X4_50).",
    )

    return parser.parse_args()


def main() -> None:
    args = parse_args()
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    if not hasattr(cv2.aruco, args.dictionary):
        raise ValueError(f"Unknown dictionary: {args.dictionary}")

    dict_id = getattr(cv2.aruco, args.dictionary)
    aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)

    tile_size_px = cm_to_px(args.marker_size_cm, args.dpi)
    cut_line_thickness_px = max(2, tile_size_px // 300)

    marker_rows = []
    tiles = []
    labels = []

    for spec in DEFAULT_MARKERS:
        tile = make_marker_tile(
            marker_id=spec.marker_id,
            tile_size_px=tile_size_px,
            quiet_zone_ratio=args.quiet_zone_ratio,
            cut_line_thickness_px=cut_line_thickness_px,
            aruco_dict=aruco_dict,
        )

        marker_file = (
            f"aruco_{spec.role.lower()}_id{spec.marker_id}_{timestamp}.png"
        )
        marker_path = output_dir / marker_file
        cv2.imwrite(str(marker_path), tile)

        marker_rows.append(
            {
                "role": spec.role,
                "marker_id": spec.marker_id,
                "filename": marker_file,
                "marker_size_cm": args.marker_size_cm,
                "dpi": args.dpi,
                "dictionary": args.dictionary,
            }
        )
        tiles.append(tile)
        labels.append(f"{spec.role} (ID {spec.marker_id})")

    sheet = build_contact_sheet(
        tiles=tiles,
        labels=labels,
        tile_size_px=tile_size_px,
        dpi=args.dpi,
    )
    sheet_file = f"aruco_contact_sheet_{timestamp}.png"
    cv2.imwrite(str(output_dir / sheet_file), sheet)

    manifest_file = f"aruco_manifest_{timestamp}.csv"
    with (output_dir / manifest_file).open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "role",
                "marker_id",
                "filename",
                "marker_size_cm",
                "dpi",
                "dictionary",
            ],
        )
        writer.writeheader()
        writer.writerows(marker_rows)

    print("Generated marker assets:")
    for row in marker_rows:
        print(f"- {row['role']}: {row['filename']}")
    print(f"- CONTACT_SHEET: {sheet_file}")
    print(f"- MANIFEST: {manifest_file}")
    print(f"Output directory: {output_dir.resolve()}")


if __name__ == "__main__":
    main()
