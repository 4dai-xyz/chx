#!/usr/bin/env python3
"""V3 前置: BEV 坐标系方向校准 — 生成 9 种 transform 的候选图 + 报告。

用户根据候选图人工选定与真实商场路线一致的 transform, 再进入 ScaRF-inspired
稠密重建。

输入:
    --static-map        output/route_A_v2/best/static_map.npy
    --static-map-meta   output/route_A_v2/best/static_map_meta.json
    --camera-json       output/route_A_v2/camera_trajectory_route_A_v2.json
    --people-json       output/route_A_v2/people_tracks_route_A_v2.json
    --output-dir        output/route_A_v3_scarf/alignment_candidates

输出:
    output/route_A_v3_scarf/alignment_candidates/
    ├── identity.png
    ├── mirror_x.png
    ├── mirror_y.png
    ├── rotate_180.png
    ├── swap_xy.png
    ├── swap_xy_mirror_x.png
    ├── swap_xy_mirror_y.png
    ├── rotate_90_cw.png
    ├── rotate_90_ccw.png
    ├── alignment_report.md
    └── alignment_candidates_index.json
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

PKG_ROOT = Path(__file__).resolve().parents[1]
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))

from people_bev_tracker.bev_alignment import (
    TRANSFORMS,
    load_camera_bev_and_headings,
    load_people_final_positions,
    render_alignment_candidate,
)


def _resolve(p: str, root: Path) -> str:
    pp = Path(p)
    return str(pp if pp.is_absolute() else (root / p).resolve())


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--static-map", required=True)
    ap.add_argument("--static-map-meta", required=True)
    ap.add_argument("--camera-json", required=True)
    ap.add_argument("--people-json", default=None)
    ap.add_argument("--output-dir", required=True)
    ap.add_argument("--heading-stride", type=int, default=200,
                    help="每 N 帧画一个 heading arrow (默认 200)")
    ap.add_argument("--people-last-frames", type=int, default=50,
                    help="用最后 N 帧的 active people 位置 (默认 50)")
    args = ap.parse_args()

    root = Path(__file__).resolve().parents[3]
    map_npy = _resolve(args.static_map, root)
    map_meta_json = _resolve(args.static_map_meta, root)
    cam_json = _resolve(args.camera_json, root)
    ppl_json = _resolve(args.people_json, root) if args.people_json else None
    out_dir = Path(_resolve(args.output_dir, root))
    out_dir.mkdir(parents=True, exist_ok=True)

    for p, lbl in [(map_npy, "static_map"), (map_meta_json, "static_map_meta"),
                   (cam_json, "camera_json")]:
        if not Path(p).exists():
            raise SystemExit(f"missing {lbl}: {p}")

    grid = np.load(map_npy)
    with open(map_meta_json, "r", encoding="utf-8") as f:
        meta = json.load(f)

    R_align = np.asarray(meta.get("R_align", np.eye(3).tolist()), dtype=np.float64)
    bev_axes = tuple(meta.get("bev_axes", ["x", "z"]))

    traj_bev, heading_samples = load_camera_bev_and_headings(
        cam_json, R_align, bev_axes=bev_axes,
        heading_stride=int(args.heading_stride),
    )
    print(f"[calibrate] camera traj = {traj_bev.shape[0]} pose, "
          f"headings = {len(heading_samples)}, resolution = {meta['resolution_unit_per_px']}")

    ppl_pts = None
    if ppl_json and Path(ppl_json).exists():
        ppl_pts = load_people_final_positions(ppl_json, use_last_frames=int(args.people_last_frames))
        print(f"[calibrate] people final positions = {ppl_pts.shape[0]}")

    saved: dict[str, str] = {}
    for t in TRANSFORMS:
        img = render_alignment_candidate(
            transform=t,
            grid=grid,
            meta=meta,
            camera_bev_xy=traj_bev,
            heading_bev_xy_list=heading_samples,
            people_final_positions=ppl_pts,
        )
        p = out_dir / f"{t}.png"
        cv2.imwrite(str(p), img)
        saved[t] = str(p)
        print(f"[calibrate] wrote {p.name}  {img.shape[1]}x{img.shape[0]}")

    # ------------- alignment_report.md -------------
    md = _make_alignment_report_md(
        saved=saved,
        camera_json=cam_json,
        static_map_npy=map_npy,
        static_map_meta=meta,
        heading_stride=int(args.heading_stride),
        n_traj=traj_bev.shape[0],
        n_headings=len(heading_samples),
        n_people=(ppl_pts.shape[0] if ppl_pts is not None else 0),
    )
    (out_dir / "alignment_report.md").write_text(md, encoding="utf-8")
    print(f"[calibrate] wrote alignment_report.md")

    # ------------- alignment_candidates_index.json -------------
    idx = {
        "transforms": list(TRANSFORMS),
        "images": {t: str(Path(p).name) for t, p in saved.items()},
        "input": {
            "static_map": map_npy,
            "static_map_meta": map_meta_json,
            "camera_json": cam_json,
            "people_json": ppl_json,
        },
        "static_map_meta_summary": {
            "width_px": meta["width_px"],
            "height_px": meta["height_px"],
            "resolution_unit_per_px": meta["resolution_unit_per_px"],
            "origin_world": meta["origin_world"],
            "bev_axes": meta.get("bev_axes"),
            "R_align_used": True,
        },
        "recommended_first_look": [
            "mirror_x", "mirror_y",
            "swap_xy_mirror_x", "swap_xy_mirror_y",
        ],
        "recommended_reason": (
            "User reported 'BEV like upside-down view; left/right turns swapped'. "
            "Pure 180° rotation does NOT flip left/right handedness — must be a mirror."
        ),
        "next_step": (
            "Human confirms which candidate matches real-world route, then run:\n"
            "  echo '{\"selected_transform\": \"<name>\", ...}' > "
            "output/route_A_v3_scarf/alignment_selected.json\n"
            "and re-run offline_pipeline_A.py (via wrapper) with the selected transform."
        ),
    }
    with open(out_dir / "alignment_candidates_index.json", "w", encoding="utf-8") as f:
        json.dump(idx, f, ensure_ascii=False, indent=2)

    print(f"[calibrate] done. {len(saved)} candidates in {out_dir}")
    return 0


def _make_alignment_report_md(
    saved: dict[str, str],
    camera_json: str,
    static_map_npy: str,
    static_map_meta: dict,
    heading_stride: int,
    n_traj: int,
    n_headings: int,
    n_people: int,
) -> str:
    return f"""# BEV alignment candidates report (Route A V3 pre-step)

> Purpose: pick a BEV coordinate transform so that the map matches the real
> mall route (real left-turn shows as left-turn in BEV; real right-turn as
> right-turn). This is a **prerequisite** for ScaRF-style dense reconstruction —
> without it, every downstream layer (nav grid, tricolor, dense pcd, video)
> inherits the wrong direction.

## 1. What is the current V2 BEV convention?

Data flow (V2):

```
world XYZ  (DPVO frame, first camera = identity)
  → R_align (ground-normal rotated onto world +Y)
  → select_bev_axes(["x", "z"])          → BEV (x, y)
  → px = W/2 + (x - origin_x) / resolution
    py = H/2 - (y - origin_y) / resolution
```

Meta used:

* static_map: `{Path(static_map_npy).name}`
* width_px × height_px:   `{static_map_meta['width_px']}` × `{static_map_meta['height_px']}`
* resolution_unit_per_px: `{static_map_meta['resolution_unit_per_px']}`
* origin_world (BEV):     `{static_map_meta['origin_world']}`
* bev_axes:               `{static_map_meta.get('bev_axes')}`
* R_align:                (rotates ground_normal → world +Y; stored in meta)

## 2. Why could this look like a "upside-down / mirrored" view?

* **DPVO's world frame has no global north/east semantics.** The first camera
  frame is identity, so "world +X / +Z" depends purely on how the first
  frame was pointing.
* **`R_align` only fixes vertical.** It aligns the ground normal onto
  world +Y (i.e. "up = down"). It does NOT determine which horizontal
  direction should be "north", nor whether the coordinate system is
  right-handed or left-handed *relative to the real world*.
* **Monocular scale + reflection ambiguity.** Purely monocular pipelines have
  no way to disambiguate between a scene and its mirror image.
* **Result:** the map may correctly show geometry, but with left/right
  handedness flipped — so real-world "turn left" becomes "turn right"
  in BEV. This is a **mirror** (chirality flip), not a rotation.

Pure rotations (`rotate_180`, `rotate_90_cw`, `rotate_90_ccw`) preserve
handedness — they cannot fix left/right swap on their own. Mirrors
(`mirror_x`, `mirror_y`) and diagonal transpose (`swap_xy` and its variants)
change handedness and CAN fix left/right swap.

## 3. Nine candidate transforms

All transforms operate on **aligned BEV (x, y) coordinates** (before pixel
rasterization). They are applied uniformly to camera trajectory, heading,
people, static_map grid, free/occupied masks, and dense point cloud.

| # | Transform          | Formula              | Handedness | Typical fix                                     |
|:-:|:---                |:---                  |:---:       |:---                                             |
| 1 | `identity`         | `[ x,  y]`           | keep       | (baseline — same as V2)                         |
| 2 | `mirror_x`         | `[-x,  y]`           | flip       | Left/right swapped                              |
| 3 | `mirror_y`         | `[ x, -y]`           | flip       | Up/down swapped (aka forward/backward)          |
| 4 | `rotate_180`       | `[-x, -y]`           | keep       | Whole map upside-down but turns still consistent|
| 5 | `swap_xy`          | `[ y,  x]`           | flip       | X/Z axes were transposed                        |
| 6 | `swap_xy_mirror_x` | `[-y,  x]`           | keep       | (composite) — rotate + mirror                   |
| 7 | `swap_xy_mirror_y` | `[ y, -x]` = 90° CW  | keep       | Same as `rotate_90_cw`                          |
| 8 | `rotate_90_cw`     | `[ y, -x]`           | keep       | Pure 90° clockwise                              |
| 9 | `rotate_90_ccw`    | `[-y,  x]`           | keep       | Pure 90° counter-clockwise                      |

(Note: `swap_xy_mirror_y` and `rotate_90_cw` are algebraically identical, but
we keep both names in the codebase for clarity when combining with other
operations later.)

## 4. Overlays on each candidate

Each `<transform>.png` has:

* **Background**: `static_map_tricolor` (black = occupied, white = free, gray = unknown)
* **Camera trajectory**: orange polyline ({n_traj} poses)
* **START**: green filled circle + label
* **END**: red filled circle + label
* **Heading arrows**: orange arrows every `{heading_stride}` frames ({n_headings} arrows)
* **People**: orange dots from last 50 frames of `people_tracks_route_A_v2.json` ({n_people} unique tracks)
* **HUD** (top): transform name

## 5. Recommendation for **THIS** case

User reported:

> "Current BEV looks like an upside-down / from-below view. Real-world
> left-turns appear as right-turns."

That is a **handedness flip**, not a pure rotation. So look **first** at:

1. **`mirror_x.png`** — flips x; if the map was 'looking from below' this
   is the most common fix
2. **`mirror_y.png`** — flips y; try if `mirror_x` looks 'flipped forward-back'
3. **`swap_xy_mirror_x.png`** — combined; try if the shape 'rotated 90° AND
   mirrored'
4. **`swap_xy_mirror_y.png`** — combined

Only if none of the mirrors look right, fall back to:

5. `rotate_180.png` — but this preserves left/right, so if you originally
   said "turns are reversed", rotate_180 will still show them reversed.
6. `rotate_90_cw.png` / `rotate_90_ccw.png` — same caveat.

`identity.png` is the current V2 output (baseline).

## 6. How to confirm

1. Open each recommended `.png` (start with `mirror_x.png`).
2. Compare START (green) → END (red) polyline against your memory of the
   actual walk. Check turns:
   - Real left-turn → the BEV polyline should visibly bend to the **left**
     as you follow START → END.
   - Real right-turn → bend to the **right**.
3. Also check the last heading arrow at the END: it should point in the
   direction the person was walking at the end of the video.
4. Pick the transform whose polyline matches. Write the choice to:

```json
// output/route_A_v3_scarf/alignment_selected.json
{{
  "selected_transform": "mirror_x",
  "reason": "user confirmed this candidate matches real-world turning direction",
  "source": "manual"
}}
```

## 7. Files in this directory

| File                                     | Content                          |
|:---                                      |:---                              |
| `identity.png`                           | current V2 (no transform)        |
| `mirror_x.png`                           | flip x — try first               |
| `mirror_y.png`                           | flip y                           |
| `rotate_180.png`                         | 180° rotation                    |
| `swap_xy.png`                            | transpose                        |
| `swap_xy_mirror_x.png`                   | transpose + flip x               |
| `swap_xy_mirror_y.png`                   | transpose + flip y               |
| `rotate_90_cw.png`                       | 90° clockwise                    |
| `rotate_90_ccw.png`                      | 90° counter-clockwise            |
| `alignment_report.md`                    | this file                        |
| `alignment_candidates_index.json`        | machine-readable index           |

## 8. What happens next (after user confirms)

After user writes `alignment_selected.json`:

1. Re-render `output/route_A_v3_scarf/aligned_preview/nav_binary_map.png`,
   `.../static_map_tricolor.png`, `.../paper_style_global_view.png`,
   `.../final_frame_alignment_preview.png` using the selected transform.
2. Extend `offline_pipeline_A.py` (and `tune_static_map_v2.py`, future ScaRF
   scripts) to read the same transform from a shared config, and apply
   it uniformly via `apply_bev_alignment_xy` / `apply_bev_alignment_heading`.
3. Store the choice into every downstream `static_map_meta.json` under
   `"bev_alignment": {{"enabled": true, "transform": "<name>"}}`.
4. Only then start V3 ScaRF-inspired dense reconstruction.

## 9. Important constraints (repeated from doc 06)

* Do **not** apply the transform by flipping the final PNG with `cv2.flip`.
  Do it in the coordinate layer so trajectory / heading / people / grid all
  stay consistent.
* Do **not** modify official `project code/DPVO`, `VGGT`, `KV-tracker`,
  `ScaRF-SLAM` code.
* Do **not** replace DPVO trajectory with anything else.
* Do **not** merge dynamic people into `static_map`.
* Do **not** proceed to ScaRF-style dense reconstruction until this
  alignment is confirmed.
"""


if __name__ == "__main__":
    raise SystemExit(main())
