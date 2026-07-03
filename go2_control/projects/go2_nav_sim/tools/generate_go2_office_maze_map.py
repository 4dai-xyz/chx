#!/usr/bin/env python3
"""Generate a deterministic Nav2 occupancy map for Go2 navigation tests."""

from __future__ import annotations

from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[3]
OUT_DIR = PROJECT_ROOT / "projects/go2_nav_sim/maps"
MAP_NAME = "go2_office_maze"

RESOLUTION = 0.05
WIDTH = 240
HEIGHT = 240
ORIGIN_X = -6.0
ORIGIN_Y = -6.0

FREE = np.uint8(254)
OCCUPIED = np.uint8(0)


def world_to_col(x: float) -> int:
    return int((x - ORIGIN_X) / RESOLUTION)


def world_to_row(y: float) -> int:
    row_from_bottom = int((y - ORIGIN_Y) / RESOLUTION)
    return HEIGHT - 1 - row_from_bottom


def paint_rect(image: np.ndarray, x0: float, y0: float, x1: float, y1: float, value: np.uint8 = OCCUPIED) -> None:
    x_min, x_max = sorted((x0, x1))
    y_min, y_max = sorted((y0, y1))
    col0 = max(0, min(WIDTH - 1, world_to_col(x_min)))
    col1 = max(0, min(WIDTH - 1, world_to_col(x_max)))
    row0 = max(0, min(HEIGHT - 1, world_to_row(y_max)))
    row1 = max(0, min(HEIGHT - 1, world_to_row(y_min)))
    image[row0 : row1 + 1, col0 : col1 + 1] = value


def paint_wall_v(image: np.ndarray, x: float, y0: float, y1: float, thickness: float = 0.18) -> None:
    half = thickness * 0.5
    paint_rect(image, x - half, y0, x + half, y1)


def paint_wall_h(image: np.ndarray, y: float, x0: float, x1: float, thickness: float = 0.18) -> None:
    half = thickness * 0.5
    paint_rect(image, x0, y - half, x1, y + half)


def build_map() -> np.ndarray:
    image = np.full((HEIGHT, WIDTH), FREE, dtype=np.uint8)

    # Outer boundary.
    paint_rect(image, -6.0, -6.0, 6.0, -5.78)
    paint_rect(image, -6.0, 5.78, 6.0, 6.0)
    paint_rect(image, -6.0, -6.0, -5.78, 6.0)
    paint_rect(image, 5.78, -6.0, 6.0, 6.0)

    # Interior office/corridor walls with deliberate door gaps.
    for y0, y1 in [(-5.6, -2.0), (-0.8, 2.3), (3.2, 5.6)]:
        paint_wall_v(image, -3.7, y0, y1)
    for y0, y1 in [(-5.6, -3.4), (-1.6, 1.0), (2.8, 5.6)]:
        paint_wall_v(image, -0.8, y0, y1)
    for y0, y1 in [(-5.6, -2.7), (-1.1, 2.2), (3.5, 5.6)]:
        paint_wall_v(image, 2.3, y0, y1)

    for x0, x1 in [(-5.6, -4.4), (-2.7, 0.2), (1.6, 5.6)]:
        paint_wall_h(image, -3.6, x0, x1)
    for x0, x1 in [(-5.6, -3.9), (-2.4, -1.0), (0.4, 2.0), (3.4, 5.6)]:
        paint_wall_h(image, -0.8, x0, x1)
    for x0, x1 in [(-5.6, -4.2), (-2.8, -1.0), (0.0, 1.6), (3.0, 5.6)]:
        paint_wall_h(image, 2.0, x0, x1)
    for x0, x1 in [(-5.6, -3.8), (-2.2, 0.4), (1.9, 5.6)]:
        paint_wall_h(image, 4.2, x0, x1)

    # Furniture-like obstacles. These add local planner pressure without blocking the whole map.
    furniture = [
        (-5.0, -5.0, -4.4, -4.2),
        (-2.4, -5.0, -1.5, -4.4),
        (3.5, -5.0, 4.6, -4.5),
        (4.0, -1.8, 4.8, -1.2),
        (-5.0, 0.2, -4.3, 0.9),
        (-2.0, 0.3, -1.4, 0.9),
        (0.8, 0.6, 1.4, 1.2),
        (3.8, 0.8, 4.6, 1.5),
        (-5.2, 3.0, -4.5, 3.7),
        (-1.0, 3.2, -0.2, 3.8),
        (1.0, 4.7, 1.8, 5.2),
        (4.5, 4.5, 5.1, 5.1),
    ]
    for rect in furniture:
        paint_rect(image, *rect)

    # Keep wide, clearly navigable routes for the first complex-map Nav2 tests.
    # robot_radius + inflation_radius is about 0.47 m, so 1.6-2.0 m corridors
    # leave enough clearance for DWB sampling and costmap discretization.
    paint_rect(image, -1.0, -1.0, 5.55, 1.0, FREE)
    paint_rect(image, 3.75, -1.0, 5.55, 4.1, FREE)
    paint_rect(image, 3.75, 2.6, 5.55, 4.1, FREE)
    paint_rect(image, -1.0, -3.35, 1.0, 1.0, FREE)
    paint_rect(image, -4.75, -3.35, 1.0, -1.65, FREE)
    paint_rect(image, -4.75, -3.35, -3.25, -1.65, FREE)

    return image


def write_pgm(path: Path, image: np.ndarray) -> None:
    with path.open("wb") as f:
        f.write(f"P5\n# {MAP_NAME} generated for Go2 Nav2 tests\n{WIDTH} {HEIGHT}\n255\n".encode("ascii"))
        f.write(image.tobytes())


def write_yaml(path: Path) -> None:
    path.write_text(
        "\n".join(
            [
                f"image: {MAP_NAME}.pgm",
                f"resolution: {RESOLUTION:.6f}",
                f"origin: [{ORIGIN_X:.6f}, {ORIGIN_Y:.6f}, 0.000000]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.196",
                "",
            ]
        ),
        encoding="utf-8",
    )


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    image = build_map()
    write_pgm(OUT_DIR / f"{MAP_NAME}.pgm", image)
    write_yaml(OUT_DIR / f"{MAP_NAME}.yaml")
    print(OUT_DIR / f"{MAP_NAME}.yaml")


if __name__ == "__main__":
    main()
