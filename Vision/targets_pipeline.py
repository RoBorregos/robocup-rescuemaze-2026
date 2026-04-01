
import argparse
import os
from pathlib import Path
from collections import deque

import cv2
import numpy as np
from ultralytics import YOLO

from detector import VisionDetector
from protocol import CAM_LEFT, CAM_RIGHT

BASE_DIR = Path(__file__).resolve().parent


def resolve_model_path(model_arg: str) -> Path:
    model_path = Path(model_arg).expanduser()
    candidates = []

    if model_path.is_absolute():
        candidates.append(model_path)
    else:
        candidates.append(Path.cwd() / model_path)
        candidates.append(BASE_DIR / model_path)
        candidates.append(BASE_DIR / "weights" / model_path)

    for candidate in candidates:
        if candidate.exists():
            return candidate.resolve()

    searched_paths = "\n".join(f"- {path}" for path in candidates)
    raise FileNotFoundError(
        "YOLO model (.pt) was not found.\n"
        "Pass --model target.pt (if it is in weights) or a valid path.\n"
        f"Searched paths:\n{searched_paths}"
    )

# Minimum confidence to accept a YOLO detection
YOLO_CONF = 0.4

# HSV ranges for each ring color
# Format: list of (lower, upper) in HSV — supports split ranges (e.g., red)
HSV_RANGES = {
    "Black":    [
        (np.array([0,   0,   0]),  np.array([180,  90,  65])),
        (np.array([0,   0,   0]),  np.array([180,  70,  45])),
    ],
    "Red":      [
        (np.array([0,  100,  60]), np.array([12,  255, 255])),
        (np.array([170,100,  60]), np.array([180, 255, 255])),
        (np.array([0,   80,  45]), np.array([8,   255, 220])),
        (np.array([172, 80,  45]), np.array([180, 255, 220])),
    ],
    "Yellow":   [
        (np.array([16,  90,  70]), np.array([40,  255, 255])),
        (np.array([20,  70,  50]), np.array([36,  255, 220])),
    ],
    "Green":    [
        (np.array([35,  50,  45]), np.array([90,  255, 255])),
        (np.array([42,  35,  35]), np.array([95,  220, 210])),
    ],
    "Blue":     [
        (np.array([88,  70,  45]), np.array([135, 255, 255])),
        (np.array([95,  50,  35]), np.array([140, 220, 210])),
    ],
}

COLOR_VALUE = {
    "Black":    -2,
    "Red":      -1,
    "Yellow":    0,
    "Green":     1,
    "Blue":      2,
}

# BGR color used to draw each class on screen
DRAW_COLOR = {
    "Black":    (50,  50,  50),
    "Red":      (0,   0,  210),
    "Yellow":   (0,  210, 240),
    "Green":    (0,  155,  30),
    "Blue":     (210, 100, 30),
}

VICTIM_NAMES = {
    0: "Unharmed_victim",
    1: "Stable_victim",
    2: "Harmed_victim",
}

RING_SAMPLE_ANGLES = (0, 45, 90, 135, 180, 225, 270, 315)
TEMPORAL_VOTE_SIZE = 5


def apply_white_balance(frame_bgr):
    """
    Apply a simple gray-world white balance to reduce color cast.
    """
    f = frame_bgr.astype(np.float32)
    b_mean = float(np.mean(f[:, :, 0]))
    g_mean = float(np.mean(f[:, :, 1]))
    r_mean = float(np.mean(f[:, :, 2]))
    avg = (b_mean + g_mean + r_mean) / 3.0

    for channel, channel_mean in enumerate((b_mean, g_mean, r_mean)):
        if channel_mean > 0:
            f[:, :, channel] *= avg / channel_mean

    return np.clip(f, 0, 255).astype(np.uint8)


def apply_clahe(frame_bgr):
    """
    Apply CLAHE to the luminance channel to improve contrast under uneven light.
    """
    lab = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2LAB)
    l_channel, a_channel, b_channel = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    l_channel = clahe.apply(l_channel)
    lab = cv2.merge((l_channel, a_channel, b_channel))
    return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)


def normalize_lighting(frame_bgr):
    """
    Normalize illumination before color classification.
    """
    balanced = apply_white_balance(frame_bgr)
    return apply_clahe(balanced)

def classify_color_at(hsv_img, cx, cy, sample_r=4):
    """
    Classify ring color by sampling a patch with radius `sample_r`
    around point (cx, cy) in the HSV image.
    Returns the color name with highest presence in that patch.
    """
    h, w = hsv_img.shape[:2]
    if h == 0 or w == 0:
        return "Black"

    sample_r = max(1, int(sample_r))
    cx = int(np.clip(cx, 0, w - 1))
    cy = int(np.clip(cy, 0, h - 1))

    x0 = max(0, cx - sample_r)
    x1 = min(w, cx + sample_r + 1)
    y0 = max(0, cy - sample_r)
    y1 = min(h, cy + sample_r + 1)
    patch = hsv_img[y0:y1, x0:x1]

    if x0 >= x1 or y0 >= y1 or patch.size == 0:
        return "Black"

    best_color = "Black"
    best_count = 0

    for color_name, ranges in HSV_RANGES.items():
        mask = np.zeros(patch.shape[:2], dtype=np.uint8)
        for lo, hi in ranges:
            mask |= cv2.inRange(patch, lo, hi)
        count = int(np.sum(mask > 0))
        if count > best_count:
            best_count = count
            best_color = color_name

    return best_color


def classify_ring_color(hsv_img, cx, cy, radius, sample_r=4, sample_angles=RING_SAMPLE_ANGLES):
    """
    Classify a ring color by sampling 8 angles around the ring radius
    and returning the majority color.
    """
    h, w = hsv_img.shape[:2]
    if h == 0 or w == 0:
        return "Black"

    votes = []
    radius = max(1, int(radius))

    for angle_deg in sample_angles:
        angle_rad = np.deg2rad(angle_deg)
        sample_x = int(np.clip(cx + radius * np.cos(angle_rad), 0, w - 1))
        sample_y = int(np.clip(cy + radius * np.sin(angle_rad), 0, h - 1))
        votes.append(classify_color_at(hsv_img, sample_x, sample_y, sample_r=sample_r))

    vote_counts = {}
    for color_name in votes:
        vote_counts[color_name] = vote_counts.get(color_name, 0) + 1

    best_color = "Black"
    best_count = -1
    for color_name, count in vote_counts.items():
        if count > best_count:
            best_count = count
            best_color = color_name

    return best_color


def analyze_rings(roi_bgr):
    """
        Given a BGR ROI of the target, detect the 5 concentric rings and
        return (ring_values, ring_colors, debug_img).

        Strategy:
            - Convert to grayscale + blur
            - Use HoughCircles to find the outer ring (ring 5)
            - Split radius into 5 equal bands
            - Sample HSV color at midpoint of each band
    """
    h, w = roi_bgr.shape[:2]
    roi_bgr = normalize_lighting(roi_bgr)
    gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 1.5)
    hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)

    debug = roi_bgr.copy()
    ring_colors = ["Black"] * 5
    ring_values = [-2] * 5

    # ── Detect outer circle using Hough ──────────────────────────────────────
    min_dim = min(h, w)
    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=min_dim // 2,          # only one circle is expected
        param1=60,
        param2=30,
        minRadius=int(min_dim * 0.25),
        maxRadius=int(min_dim * 0.55),
    )

    if circles is None:
        # Fallback: assume target fills the ROI
        cx, cy = w // 2, h // 2
        outer_r = min_dim // 2 - 2
    else:
        candidates = np.round(circles[0]).astype(int)
        largest_idx = int(np.argmax(candidates[:, 2]))
        c = candidates[largest_idx]
        cx, cy, outer_r = int(c[0]), int(c[1]), int(c[2])

    cx = int(np.clip(cx, 0, w - 1))
    cy = int(np.clip(cy, 0, h - 1))
    outer_r = max(2, min(outer_r, min_dim // 2))

    # ── Split into 5 rings and classify each one ─────────────────────────────
    # Ring 1 = center (R1), Ring 5 = outer (R5)
    for i in range(5):
        # Inner and outer radius of ring i (0-indexed from center)
        r_outer = int(outer_r * (i + 1) / 5)
        r_inner = int(outer_r * i / 5)
        r_mid   = (r_outer + r_inner) // 2  # radius of sample point

        color_name = classify_ring_color(
            hsv,
            cx,
            cy,
            r_mid,
            sample_r=max(3, r_mid // 4),
        )
        ring_colors[i] = color_name
        ring_values[i] = COLOR_VALUE[color_name]

        # Draw ring outer boundary on debug image
        cv2.circle(debug, (cx, cy), r_outer, DRAW_COLOR[color_name], 1)
        for angle_deg in RING_SAMPLE_ANGLES:
            angle_rad = np.deg2rad(angle_deg)
            sample_x = int(np.clip(cx + r_mid * np.cos(angle_rad), 0, w - 1))
            sample_y = int(np.clip(cy + r_mid * np.sin(angle_rad), 0, h - 1))
            cv2.circle(debug, (sample_x, sample_y), 2, DRAW_COLOR[color_name], -1)

    # Draw center
    cv2.circle(debug, (cx, cy), 3, (255, 255, 255), -1)

    return ring_values, ring_colors, debug



def classify_victim(ring_values):
    s = sum(ring_values)
    label = VICTIM_NAMES.get(s, "Fake_target")
    return label, s


def vote_label(label_history, new_label):
    label_history.append(new_label)
    vote_counts = {}
    for label in label_history:
        vote_counts[label] = vote_counts.get(label, 0) + 1

    best_label = new_label
    best_count = -1
    for label, count in vote_counts.items():
        if count > best_count:
            best_count = count
            best_label = label

    return best_label


def process_frame(frame, model, label_history=None):
    """
    Run the full pipeline on a BGR frame.
    Return the annotated frame.
    """
    results = model(frame, conf=YOLO_CONF, verbose=False)[0]
    annotated = frame.copy()

    for box in results.boxes:
        # ── YOLO bounding box ──────────────────────────────────────────────
        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
        conf = float(box.conf[0])

        # Small padding to avoid cutting target edges
        pad = max(4, int((x2 - x1) * 0.05))
        rx1 = max(0, x1 - pad)
        ry1 = max(0, y1 - pad)
        rx2 = min(frame.shape[1], x2 + pad)
        ry2 = min(frame.shape[0], y2 + pad)

        roi = frame[ry1:ry2, rx1:rx2]
        if roi.size == 0:
            continue

        # ── Ring analysis ──────────────────────────────────────────────────
        ring_values, ring_colors, debug_roi = analyze_rings(roi)

        # ── Final classification ───────────────────────────────────────────
        victim_label, ring_sum = classify_victim(ring_values)
        stable_label = victim_label
        if label_history is not None:
            stable_label = vote_label(label_history, victim_label)

        # ── Annotate frame ─────────────────────────────────────────────────
        # Bounding box color by class
        box_color = {
            "Unharmed_victim": (0, 200, 0),
            "Stable_victim":   (0, 200, 200),
            "Harmed_victim":   (0, 100, 255),
            "Fake_target":     (100, 100, 100),
        }.get(stable_label, (255, 255, 255))

        cv2.rectangle(annotated, (x1, y1), (x2, y2), box_color, 2)

        # Main label
        label_text = f"{stable_label}  (raw={victim_label}, sum={ring_sum})  {conf:.2f}"
        cv2.putText(annotated, label_text,
                    (x1, max(y1 - 8, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, box_color, 2)

        # Rings below bounding box
        rings_text = "  ".join(
            f"R{i+1}:{ring_colors[i][0]}{ring_values[i]:+d}"
            for i in range(5)
        )
        cv2.putText(annotated, rings_text,
                    (x1, y2 + 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

        # Paste debug ROI (detected rings) in top-right corner
        th, tw = debug_roi.shape[:2]
        scale = min(120 / tw, 120 / th)
        dw, dh = int(tw * scale), int(th * scale)
        thumb = cv2.resize(debug_roi, (dw, dh))

        fx = min(frame.shape[1] - dw, x2 + 4)
        fy = max(0, y1)
        if fy + dh <= frame.shape[0] and fx + dw <= frame.shape[1]:
            annotated[fy:fy+dh, fx:fx+dw] = thumb

    return annotated


def main():
    parser = argparse.ArgumentParser(description="Target analysis pipeline")
    parser.add_argument("--model",  default="target.pt",
                        help="Path to trained YOLO model (.pt)")
    parser.add_argument("--source", default="0",
                        help="Source: image/video path or camera index (default: 0)")
    parser.add_argument("--save",   default="",
                        help="Save output to this path (e.g., out.jpg or out.mp4)")
    parser.add_argument("--headless", action="store_true",
                        help="Disable OpenCV windows (useful in SSH/headless)")
    args = parser.parse_args()

    model_path = resolve_model_path(args.model)
    print(f"  Loading model: {model_path}")
    model = YOLO(str(model_path))
    label_history = deque(maxlen=TEMPORAL_VOTE_SIZE)

    # ── Detect whether source is image or video/camera ─────────────────────
    img_extensions = (".jpg", ".jpeg", ".png", ".bmp", ".webp")
    is_image = args.source.lower().endswith(img_extensions)

    if is_image:
        frame = cv2.imread(args.source)
        if frame is None:
            print(f"  Could not read image: {args.source}")
            return

        result = process_frame(frame, model, label_history=label_history)

        if args.save:
            cv2.imwrite(args.save, result)
            print(f" Saved to {args.save}")

        cv2.imshow("Target Pipeline", result)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    else:
        # Video or camera
        source = int(args.source) if args.source.isdigit() else args.source
        has_display = (not args.headless) and (not os.environ.get("SSH_CONNECTION")) and bool(
            os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
        )

        camera_handler = None
        cap = None
        camera_id = None
        if isinstance(source, int):
            camera_handler = VisionDetector()
            camera_id = CAM_LEFT if source == CAM_LEFT else CAM_RIGHT
            print(f"  Using VisionDetector camera backend for CAM={camera_id}")
        else:
            cap = cv2.VideoCapture(source)
            if not cap.isOpened():
                print(f"   Could not open source: {args.source}")
                return

        writer = None
        if args.save and cap is not None:
            fps = cap.get(cv2.CAP_PROP_FPS) or 30
            fw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            fh = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            writer = cv2.VideoWriter(
                args.save,
                cv2.VideoWriter_fourcc(*"mp4v"),
                fps,
                (fw, fh),
            )

        print("  Processing... press Q to quit.")
        no_frame_count = 0
        while True:
            if camera_handler is not None:
                ret, frame = camera_handler.read_frame(camera_id)
            else:
                ret, frame = cap.read()

            if not ret:
                no_frame_count += 1
                if no_frame_count >= 20:
                    print("  No frames received from source; stopping.")
                    break
                continue
            no_frame_count = 0

            if writer is None and args.save:
                frame_h, frame_w = frame.shape[:2]
                writer = cv2.VideoWriter(
                    args.save,
                    cv2.VideoWriter_fourcc(*"mp4v"),
                    30,
                    (frame_w, frame_h),
                )

            result = process_frame(frame, model, label_history=label_history)

            if has_display:
                cv2.imshow("Target Pipeline", result)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            if writer:
                writer.write(result)

        if cap is not None:
            cap.release()
        if camera_handler is not None:
            camera_handler.close()
        if writer:
            writer.release()
        cv2.destroyAllWindows()
        print("  Done.")


if __name__ == "__main__":
    main()