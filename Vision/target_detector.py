"""Bullseye target detection utilities for RoboCup Rescue.

This module wraps the circle-and-color analysis used by the old
`test_targets.py` script into a reusable API so it can be called from the
main vision detector, the service, or standalone scripts.
"""

from __future__ import annotations

import argparse
import os
import time
from collections import deque
from pathlib import Path
from typing import Deque

os.environ.setdefault("OPENCV_VIDEOIO_PRIORITY_OBSENSOR", "0")

import cv2
import numpy as np
from ultralytics import YOLO

import Constants
from detector import VisionDetector
from protocol import (
    CAM_LEFT,
    CAM_RIGHT,
    VICTIM_FAKE_TARGET,
    VICTIM_HARMED,
    VICTIM_NONE,
    VICTIM_STABLE,
    VICTIM_UNHARMED,
)

EXPECTED_ORDER = ["green", "black", "red", "blue", "yellow"]
N_RINGS = 5
TEMPORAL_VOTE_SIZE = 5
BASE_DIR = Path(__file__).resolve().parent

COLOR_VALUE = {
    "green": 1,
    "black": -2,
    "red": -1,
    "blue": 2,
    "yellow": 0,
    "unknown": 0,
}

VICTIM_NAMES = {
    0: "Unharmed_victim",
    1: "Stable_victim",
    2: "Harmed_victim",
}

BOX_COLOR = {
    "Unharmed_victim": (0, 200, 0),
    "Stable_victim": (0, 200, 200),
    "Harmed_victim": (0, 100, 255),
    "Fake_target": (100, 100, 100),
}

TARGET_LABEL_TO_VICTIM_ID = {
    "Unharmed_victim": VICTIM_UNHARMED,
    "Stable_victim": VICTIM_STABLE,
    "Harmed_victim": VICTIM_HARMED,
    "Fake_target": VICTIM_FAKE_TARGET,
}


def resolve_model_path(model_arg: str = "target.pt") -> Path:
    model_path = Path(model_arg).expanduser()
    hsu_dir = BASE_DIR / "Detection" / "HSU"
    candidates = []

    if model_path.is_absolute():
        candidates.append(model_path)
    else:
        candidates.append(Path.cwd() / model_path)
        candidates.append(BASE_DIR / model_path)
        candidates.append(BASE_DIR / "weights" / model_path)
        candidates.append(hsu_dir / model_path)
        candidates.append(hsu_dir / "weights" / model_path)

    for candidate in candidates:
        if candidate.exists():
            return candidate.resolve()

    searched_paths = "\n".join(f"- {path}" for path in candidates)
    raise FileNotFoundError(
        "YOLO model (.pt) was not found.\n"
        "Pass --model target.pt (if it is in weights) or a valid path.\n"
        f"Searched paths:\n{searched_paths}"
    )


def _get_geom_setting(camera_id: int | None, name: str, default):
    if camera_id == CAM_RIGHT:
        return getattr(
            Constants,
            f"target_right_{name}",
            getattr(Constants, f"target_{name}", default),
        )
    if camera_id == CAM_LEFT:
        return getattr(
            Constants,
            f"target_left_{name}",
            getattr(Constants, f"target_{name}", default),
        )
    return getattr(Constants, f"target_{name}", default)


def _is_verbose(camera_id: int | None) -> bool:
    return bool(_get_geom_setting(camera_id, "verbose", False))


def _is_fast_mode(camera_id: int | None) -> bool:
    return bool(_get_geom_setting(camera_id, "fast_mode", True))


def detect_target_roi_with_yolo(
    img,
    model,
    conf: float = 0.4,
    pad_ratio: float = 0.05,
    min_pad: int = 4,
    force_square: bool = False,
):
    """Detect the target with YOLO and return the largest bbox plus ROI."""
    h_img, w_img = img.shape[:2]
    results = model(img, conf=conf, verbose=False)[0]

    best = None
    for box in results.boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
        score = float(box.conf[0])

        x1 = max(0, min(x1, w_img - 1))
        y1 = max(0, min(y1, h_img - 1))
        x2 = max(0, min(x2, w_img))
        y2 = max(0, min(y2, h_img))
        if x2 <= x1 or y2 <= y1:
            continue

        area = (x2 - x1) * (y2 - y1)
        if best is None or area > best["area"]:
            best = {"x1": x1, "y1": y1, "x2": x2, "y2": y2, "conf": score, "area": area}

    if best is None:
        return None

    box_w = best["x2"] - best["x1"]
    box_h = best["y2"] - best["y1"]
    pad = max(int(min_pad), int(min(box_w, box_h) * float(pad_ratio)))

    if force_square:
        cx = (best["x1"] + best["x2"]) // 2
        cy = (best["y1"] + best["y2"]) // 2
        half = int(max(box_w, box_h) / 2) + pad
        rx1 = max(0, cx - half)
        ry1 = max(0, cy - half)
        rx2 = min(w_img, cx + half)
        ry2 = min(h_img, cy + half)
    else:
        rx1 = max(0, best["x1"] - pad)
        ry1 = max(0, best["y1"] - pad)
        rx2 = min(w_img, best["x2"] + pad)
        ry2 = min(h_img, best["y2"] + pad)

    roi = img[ry1:ry2, rx1:rx2]
    if roi.size == 0:
        return None

    return {"bbox": (rx1, ry1, rx2, ry2), "conf": best["conf"], "roi": roi}


def classify_color_hsv(h, s, v):
    """Classify a single pixel by HSV values."""
    if v < 55:
        return "black"
    if 85 < h <= 135 and s > 40 and v < 80:
        return "black"
    if s < 35 and v > 180:
        return "white"
    if s < 35:
        return "gray"
    if 18 <= h <= 40 and s > 50:
        return "yellow"
    if 40 < h <= 85 and s > 30:
        return "green"
    if 85 < h <= 135 and s > 40:
        return "blue"
    if (h <= 18 or h >= 165) and s > 40 and v > 50:
        return "red"
    return "unknown"


def find_largest_circle(
    img,
    min_radius_ratio: float = 0.15,
    min_circularity: float = 0.50,
    contour_min_area: float = 500.0,
    border_allow_ratio: float = 0.0,
    fast_mode: bool = False,
):
    """Find the biggest circle via Hough sweep + contour fallback."""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (9, 9), 2)
    h_img, w_img = img.shape[:2]
    max_r = max(h_img, w_img) // 2
    min_r = int(min(h_img, w_img) * float(min_radius_ratio))
    border_allow = int(min(h_img, w_img) * float(border_allow_ratio))

    if fast_mode:
        dp_values = [1.0, 1.4]
        p1_values = [60, 100]
        p2_values = [24, 36, 50]
    else:
        dp_values = [1.0, 1.2, 1.5, 1.8]
        p1_values = [50, 80, 120]
        p2_values = [22, 30, 40, 50, 70]

    best = None
    for dp in dp_values:
        for p1 in p1_values:
            for p2 in p2_values:
                circles = cv2.HoughCircles(
                    blurred,
                    cv2.HOUGH_GRADIENT,
                    dp=dp,
                    minDist=max(h_img, w_img) // 4,
                    param1=p1,
                    param2=p2,
                    minRadius=min_r,
                    maxRadius=max_r,
                )
                if circles is not None:
                    for c in circles[0]:
                        ccx, ccy, cr = int(c[0]), int(c[1]), int(c[2])
                        if ccx - cr < -border_allow or ccx + cr >= w_img + border_allow:
                            continue
                        if ccy - cr < -border_allow or ccy + cr >= h_img + border_allow:
                            continue
                        if best is None or cr > best[2]:
                            best = (ccx, ccy, cr)
                if fast_mode and best is not None:
                    break
            if fast_mode and best is not None:
                break
        if fast_mode and best is not None:
            break

    edges = cv2.Canny(blurred, 30, 100)
    edges = cv2.dilate(edges, None, iterations=1)
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < float(contour_min_area):
            continue
        peri = cv2.arcLength(cnt, True)
        if peri == 0:
            continue
        circ = 4 * np.pi * area / (peri**2)
        if circ > float(min_circularity):
            (cx, cy), radius = cv2.minEnclosingCircle(cnt)
            r = int(radius)
            ccx, ccy = int(cx), int(cy)
            if r < min_r:
                continue
            if ccx - r < -border_allow or ccx + r >= w_img + border_allow:
                continue
            if ccy - r < -border_allow or ccy + r >= h_img + border_allow:
                continue
            if best is None or r > best[2]:
                best = (ccx, ccy, r)

    if best is None:
        print("[WARN] No circle fully within image bounds found")
        return None
    return best


def classify_ring(
    hsv,
    cx,
    cy,
    r_inner,
    r_outer,
    n_angles: int = 90,
    min_radial_samples: int = 5,
    radial_divisor: int = 2,
):
    """Classify a ring region by HSV majority vote on sampled pixels."""
    h_img, w_img = hsv.shape[:2]
    angles = np.linspace(0, 2 * np.pi, n_angles, endpoint=False)
    n_radial = max(
        int(min_radial_samples), (r_outer - r_inner) // max(1, int(radial_divisor))
    )
    radii = np.linspace(r_inner, r_outer, n_radial, endpoint=False)

    votes = {}
    for r in radii:
        xs = (cx + r * np.cos(angles)).astype(int)
        ys = (cy + r * np.sin(angles)).astype(int)
        valid = (xs >= 0) & (xs < w_img) & (ys >= 0) & (ys < h_img)
        for x, y, ok in zip(xs, ys, valid):
            if not ok:
                continue
            h, s, v = int(hsv[y, x, 0]), int(hsv[y, x, 1]), int(hsv[y, x, 2])
            c = classify_color_hsv(h, s, v)
            if c not in ("unknown", "white", "gray"):
                votes[c] = votes.get(c, 0) + 1

    if not votes:
        return "unknown"
    return max(votes, key=votes.get)


def classify_victim(ring_colors):
    """Map detected ring colors to a victim label."""
    total = sum(COLOR_VALUE.get(c, 0) for c in ring_colors)
    label = VICTIM_NAMES.get(total, "Fake_target")
    return label, total


def vote_label(label_history: Deque[str], new_label: str):
    """Temporal majority vote over recent frames."""
    label_history.append(new_label)
    vote_counts = {}
    for lbl in label_history:
        vote_counts[lbl] = vote_counts.get(lbl, 0) + 1
    return max(vote_counts, key=vote_counts.get)


def detect_bullseye_frame(
    frame,
    model=None,
    yolo_conf: float = 0.4,
    label_history=None,
    camera_id: int | None = None,
):
    img = frame.copy()
    orig = frame.copy()
    bbox = None
    yolo_score = None

    if model is not None:
        roi_pad_ratio = float(_get_geom_setting(camera_id, "roi_pad_ratio", 0.05))
        roi_min_pad = int(_get_geom_setting(camera_id, "roi_min_pad", 4))
        roi_force_square = bool(_get_geom_setting(camera_id, "roi_force_square", False))

        yolo_result = detect_target_roi_with_yolo(
            img,
            model,
            conf=yolo_conf,
            pad_ratio=roi_pad_ratio,
            min_pad=roi_min_pad,
            force_square=roi_force_square,
        )
        if yolo_result is None:
            if _is_verbose(camera_id):
                print("[WARN] YOLO did not find a valid target box")
            return None
        bbox = yolo_result["bbox"]
        yolo_score = yolo_result["conf"]
        img = yolo_result["roi"]
        if _is_verbose(camera_id):
            print(f"[INFO] YOLO bbox={bbox} conf={yolo_score:.2f}")

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h_img, w_img = img.shape[:2]

    result = find_largest_circle(
        img,
        min_radius_ratio=float(
            _get_geom_setting(camera_id, "circle_min_radius_ratio", 0.15)
        ),
        min_circularity=float(
            _get_geom_setting(camera_id, "circle_min_circularity", 0.50)
        ),
        contour_min_area=float(
            _get_geom_setting(camera_id, "circle_contour_min_area", 500.0)
        ),
        border_allow_ratio=float(
            _get_geom_setting(camera_id, "circle_border_allow_ratio", 0.0)
        ),
        fast_mode=_is_fast_mode(camera_id),
    )
    if result is None:
        if _is_verbose(camera_id):
            print("[INFO] No valid target found (no circle fully inside the image)")
        return None
    cx, cy, outer_r = result
    if _is_verbose(camera_id):
        print(f"[INFO] Main circle -> center=({cx},{cy}) radius={outer_r}")

    Y, X = np.ogrid[:h_img, :w_img]
    dist = np.sqrt((X - cx) ** 2 + (Y - cy) ** 2).astype(np.float32)

    ring_data = []
    ring_n_angles = int(_get_geom_setting(camera_id, "ring_n_angles", 90))
    ring_min_radial = int(_get_geom_setting(camera_id, "ring_min_radial_samples", 5))
    ring_radial_divisor = int(_get_geom_setting(camera_id, "ring_radial_divisor", 2))

    for i in range(N_RINGS):
        r_outer = int(outer_r * (N_RINGS - i) / N_RINGS)
        r_inner = int(outer_r * (N_RINGS - i - 1) / N_RINGS)

        color = classify_ring(
            hsv,
            cx,
            cy,
            r_inner,
            r_outer,
            n_angles=ring_n_angles,
            min_radial_samples=ring_min_radial,
            radial_divisor=ring_radial_divisor,
        )

        mask = ((dist >= r_inner) & (dist < r_outer)).astype(np.uint8)
        bgr_px = img[mask == 1]
        hsv_px = hsv[mask == 1]
        avg_rgb = (
            tuple(np.mean(bgr_px, axis=0).astype(int)[[2, 1, 0]])
            if len(bgr_px)
            else (0, 0, 0)
        )
        avg_hsv = (
            tuple(np.mean(hsv_px, axis=0).astype(int)) if len(hsv_px) else (0, 0, 0)
        )

        ring_data.append(
            {
                "ring": i,
                "r_outer": r_outer,
                "r_inner": r_inner,
                "color": color,
                "avg_rgb": avg_rgb,
                "avg_hsv": avg_hsv,
                "n_pixels": len(bgr_px),
            }
        )

    detected = [d["color"] for d in ring_data]
    victim_label, ring_sum = classify_victim(detected)
    stable_label = victim_label
    if label_history is not None:
        stable_label = vote_label(label_history, victim_label)

    if _is_verbose(camera_id):
        print()
        print("=" * 70)
        print("  BULLSEYE ANALYSIS (outside -> inside)")
        print("=" * 70)
        for d in ring_data:
            r, g, b = d["avg_rgb"]
            h, s, v = d["avg_hsv"]
            exp = EXPECTED_ORDER[d["ring"]]
            ok = "OK" if d["color"] == exp else "FAIL"
            val = COLOR_VALUE.get(d["color"], 0)
            print(
                f"  Ring {d['ring']} | {d['color']:>8s} (exp: {exp:>7s}) [{ok:>4s}] "
                f"val={val:+d} | RGB=({r:3d},{g:3d},{b:3d}) | HSV=({h:3d},{s:3d},{v:3d}) | "
                f"r=[{d['r_inner']:3d},{d['r_outer']:3d}]"
            )

        print(f"\n  Detected : {detected}")
        print(f"  Expected : {EXPECTED_ORDER}")
        print(f"  Ring sum : {ring_sum}  →  raw={victim_label}  stable={stable_label}")
        print(
            f"\n  {'[OK] VALID TARGET' if victim_label != 'Fake_target' else '[!!] FAKE TARGET'}"
            f"  —  {stable_label}"
        )
        print("=" * 70)

    vis = orig.copy()
    color_draw = {
        "green": (0, 255, 0),
        "black": (80, 80, 80),
        "red": (0, 0, 255),
        "blue": (255, 0, 0),
        "yellow": (0, 255, 255),
        "unknown": (128, 128, 128),
    }

    gx, gy = cx, cy
    if bbox is not None:
        x1, y1, x2, y2 = bbox
        gx, gy = cx + x1, cy + y1
        bcolor = BOX_COLOR.get(stable_label, (255, 255, 255))
        cv2.rectangle(vis, (x1, y1), (x2, y2), bcolor, 2)
        cv2.putText(
            vis,
            f"YOLO {yolo_score:.2f}",
            (x1, max(15, y1 - 7)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            bcolor,
            2,
        )

    cv2.circle(vis, (gx, gy), outer_r, (255, 255, 255), 2)
    for d in ring_data:
        c = color_draw.get(d["color"], (128, 128, 128))
        cv2.circle(vis, (gx, gy), d["r_outer"], c, 2)
        cv2.circle(vis, (gx, gy), d["r_inner"], c, 1)
        lx = int(gx + d["r_outer"] * 0.707)
        ly = int(gy - d["r_outer"] * 0.707)
        vh, vw = vis.shape[:2]
        lx = max(5, min(lx, vw - 70))
        ly = max(15, min(ly, vh - 5))
        cv2.putText(
            vis, d["color"], (lx, ly), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 2
        )
        cv2.putText(
            vis,
            d["color"],
            (lx, ly),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (255, 255, 255),
            1,
        )
    cv2.circle(vis, (gx, gy), 4, (0, 0, 255), -1)

    tag_color = BOX_COLOR.get(stable_label, (200, 200, 200))
    cv2.putText(
        vis, stable_label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 3
    )
    cv2.putText(
        vis, stable_label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, tag_color, 2
    )

    sum_text = f"sum={ring_sum}  rings: {' '.join(detected)}"
    cv2.putText(
        vis, sum_text, (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1
    )

    return {
        "center": (cx, cy),
        "outer_radius": outer_r,
        "rings": ring_data,
        "detected": detected,
        "ring_sum": ring_sum,
        "victim_label": victim_label,
        "stable_label": stable_label,
        "bbox": bbox,
        "yolo_conf": yolo_score,
        "vis": vis,
    }


class TargetDetector:
    """Reusable target detector that keeps the YOLO model in memory."""

    def __init__(self, model_path: str = "target.pt", conf: float = 0.4) -> None:
        self.model_path = resolve_model_path(model_path)
        self.model = YOLO(str(self.model_path))
        self.conf = conf

    def detect_frame(self, frame, label_history=None):
        return detect_bullseye_frame(
            frame,
            model=self.model,
            yolo_conf=self.conf,
            label_history=label_history,
            camera_id=None,
        )

    def detect_camera_frame(
        self, detector: VisionDetector, camera_id: int, label_history=None
    ):
        ok, frame = detector.read_frame(camera_id)
        if not ok or frame is None:
            return None
        return detect_bullseye_frame(
            frame,
            model=self.model,
            yolo_conf=self.conf,
            label_history=label_history,
            camera_id=camera_id,
        )

    @staticmethod
    def victim_id_from_result(result) -> int:
        if not result:
            return VICTIM_NONE
        raw_label = result.get("victim_label")
        if isinstance(raw_label, str):
            mapped = TARGET_LABEL_TO_VICTIM_ID.get(raw_label)
            if mapped is not None:
                return mapped

        stable_label = result.get("stable_label")
        if isinstance(stable_label, str):
            mapped = TARGET_LABEL_TO_VICTIM_ID.get(stable_label)
            if mapped is not None:
                return mapped

        return VICTIM_NONE


def parse_camera_mode(source_text: str):
    token = str(source_text).strip().lower()
    if token in ("0", "right", "r"):
        return [("RIGHT", CAM_RIGHT)]
    if token in ("1", "left", "l"):
        return [("LEFT", CAM_LEFT)]
    if token in ("both", "all", "lr", "rl"):
        return [("RIGHT", CAM_RIGHT), ("LEFT", CAM_LEFT)]
    return None


def run_camera_with_detector(
    camera_streams, model=None, yolo_conf: float = 0.4, show: bool = True
):
    detector = VisionDetector()
    has_display = (
        bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")) and show
    )
    histories = {name: deque(maxlen=TEMPORAL_VOTE_SIZE) for name, _ in camera_streams}
    frame_shape_logged = set()

    try:
        if has_display:
            print("[INFO] Camera preview active. Press Q to quit.")
        else:
            print("[HEADLESS] DISPLAY not found or --show disabled.")
            print("[HEADLESS] Press Ctrl+C to stop.")

        while True:
            status_parts = []
            for cam_name, cam_id in camera_streams:
                ok, frame = detector.read_frame(cam_id)
                if not ok or frame is None:
                    status_parts.append(f"{cam_name}=NO_FRAME")
                    if has_display:
                        blank = np.full((480, 640, 3), 255, dtype=np.uint8)
                        cv2.putText(
                            blank,
                            f"{cam_name}: camera read failed",
                            (20, 35),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.8,
                            (0, 0, 255),
                            2,
                        )
                        cv2.imshow(f"Bullseye {cam_name}", blank)
                    continue

                if cam_name not in frame_shape_logged:
                    frame_h, frame_w = frame.shape[:2]
                    print(f"[INFO] {cam_name} frame shape={frame_w}x{frame_h}")
                    frame_shape_logged.add(cam_name)

                result = detect_bullseye_frame(
                    frame,
                    model=model,
                    yolo_conf=yolo_conf,
                    label_history=histories[cam_name],
                    camera_id=cam_id,
                )
                vis = frame if result is None else result["vis"]
                label = "NONE" if result is None else result["stable_label"]
                status_parts.append(f"{cam_name}={label}")

                if has_display:
                    cv2.imshow(f"Bullseye {cam_name}", vis)

            if not has_display:
                print("[HEADLESS] " + " | ".join(status_parts))
                time.sleep(0.35)
                continue

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
    finally:
        detector.close()
        cv2.destroyAllWindows()


def detect_bullseye(image_path, model=None, yolo_conf: float = 0.4, label_history=None):
    img = cv2.imread(image_path)
    if img is None:
        print(f"[ERROR] Cannot read image: {image_path}")
        return None

    result = detect_bullseye_frame(
        img, model=model, yolo_conf=yolo_conf, label_history=label_history
    )
    if result is None:
        return None

    out_path = os.path.splitext(image_path)[0] + "_result.png"
    cv2.imwrite(out_path, result["vis"])
    print(f"\n[INFO] Visualization: {out_path}")
    return result


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Bullseye detector with victim classification"
    )
    parser.add_argument(
        "image_path",
        nargs="?",
        default="",
        help="Path to input image (optional if --source is used)",
    )
    parser.add_argument(
        "--source",
        default="",
        help="Source: image/video path, camera index, or left/right/both (default: 0)",
    )
    parser.add_argument("--model", default="target.pt", help="YOLO model (.pt) path")
    parser.add_argument(
        "--conf", type=float, default=0.4, help="YOLO confidence threshold"
    )
    parser.add_argument(
        "--no-yolo", action="store_true", help="Skip YOLO and analyze full image"
    )
    parser.add_argument(
        "--show", action="store_true", default=True, help="Show visualization window"
    )
    args = parser.parse_args()

    input_source = args.source.strip() if args.source else ""
    if not input_source:
        input_source = args.image_path.strip() if args.image_path else ""
    if not input_source:
        input_source = "0"

    yolo_model = None
    if not args.no_yolo:
        model_path = resolve_model_path(args.model)
        print(f"[INFO] Loading YOLO model: {model_path}")
        yolo_model = YOLO(str(model_path))

    label_history = deque(maxlen=TEMPORAL_VOTE_SIZE)
    img_extensions = (".jpg", ".jpeg", ".png", ".bmp", ".webp")
    is_image = input_source.lower().endswith(img_extensions)

    if is_image:
        detect_bullseye(
            input_source,
            model=yolo_model,
            yolo_conf=args.conf,
            label_history=label_history,
        )
    else:
        camera_mode = parse_camera_mode(input_source)
        if camera_mode is not None:
            run_camera_with_detector(
                camera_mode, model=yolo_model, yolo_conf=args.conf, show=args.show
            )
            print("[INFO] Done.")
        else:
            source = int(input_source) if input_source.isdigit() else input_source
            cap = cv2.VideoCapture(source)
            if not cap.isOpened():
                print(f"[ERROR] Could not open source: {input_source}")
                raise SystemExit(1)

            print("[INFO] Processing... press Q to quit.")
            while True:
                ok, frame = cap.read()
                if not ok:
                    break

                result = detect_bullseye_frame(
                    frame,
                    model=yolo_model,
                    yolo_conf=args.conf,
                    label_history=label_history,
                )
                vis = frame if result is None else result["vis"]

                if args.show:
                    cv2.imshow("Bullseye Detector", vis)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break

            cap.release()
            cv2.destroyAllWindows()
            print("[INFO] Done.")


if __name__ == "__main__":
    main()
