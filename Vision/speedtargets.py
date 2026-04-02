"""
RoboCup Rescue - Color Acuity Bullseye Target Detector
=======================================================
1. Finds the LARGEST circle in the image (main target)
2. Divides it into 5 equal concentric rings
3. Classifies each ring's color via HSV majority vote
4. Validates against expected: GREEN, BLACK, RED, BLUE, YELLOW
5. Classifies victim: Unharmed / Stable / Harmed / Fake Target
6. Reports result with temporal vote stabilization

Fuentes soportadas:
  --source 0              → Cámara USB índice 0
  --source 1              → Cámara USB índice 1
  --csi                   → Cámara CSI (Raspberry Pi, libcamera/GStreamer)
  --source video.mp4      → Archivo de video
  --source imagen.jpg     → Imagen estática

Usage:
    python targetnew.py --source 0 --model target.pt
    python targetnew.py --csi --model target.pt
    python targetnew.py --source path/to/image.jpg --model target.pt
"""

import argparse
import cv2
import numpy as np
import os
from pathlib import Path
from collections import deque
from ultralytics import YOLO

EXPECTED_ORDER     = ["green", "black", "red", "blue", "yellow"]
N_RINGS            = 5
BASE_DIR           = Path(__file__).resolve().parent
TEMPORAL_VOTE_SIZE = 5

COLOR_VALUE = {
    "green":   1,
    "black":  -2,
    "red":    -1,
    "blue":    2,
    "yellow":  0,
    "unknown": 0,
}

VICTIM_NAMES = {
    0: "Unharmed_victim",
    1: "Stable_victim",
    2: "Harmed_victim",
}

BOX_COLOR = {
    "Unharmed_victim": (0, 200, 0),
    "Stable_victim":   (0, 200, 200),
    "Harmed_victim":   (0, 100, 255),
    "Fake_target":     (100, 100, 100),
}


# ── Model path ────────────────────────────────────────────────────────────────
def resolve_model_path(model_arg: str) -> Path:
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


# ── Apertura de cámara ────────────────────────────────────────────────────────
def build_csi_pipeline(width: int, height: int, fps: int) -> str:
    """Pipeline GStreamer para cámara CSI en Raspberry Pi con libcamera."""
    return (
        f"libcamerasrc ! "
        f"video/x-raw,width={width},height={height},framerate={fps}/1 ! "
        f"videoconvert ! appsink drop=true max-buffers=1"
    )


def open_camera(args) -> cv2.VideoCapture:
    """
    Abre la fuente de video correcta según los argumentos.
    Soporta: CSI (libcamera), USB, video file.
    """
    use_csi = getattr(args, "csi", False)

    if use_csi:
        pipeline = build_csi_pipeline(args.cam_width, args.cam_height, args.cam_fps)
        print(f"[INFO] Abriendo cámara CSI con GStreamer...")
        print(f"[INFO] Pipeline: {pipeline}")
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            raise RuntimeError(
                "No se pudo abrir la cámara CSI.\n"
                "Verifica: libcamera instalado, cámara habilitada "
                "en raspi-config y OpenCV compilado con GStreamer."
            )
        return cap

    # USB o archivo de video
    source_str = args.source.strip() if args.source else "0"
    source     = int(source_str) if source_str.isdigit() else source_str

    if isinstance(source, int):
        print(f"[INFO] Abriendo cámara USB índice {source}...")
        cap = cv2.VideoCapture(source)
        if not cap.isOpened():
            # Fallback: intentar con V4L2 explícitamente (Linux/RPi)
            print(f"[WARN] Reintentando con CAP_V4L2...")
            cap = cv2.VideoCapture(source, cv2.CAP_V4L2)
    else:
        print(f"[INFO] Abriendo archivo de video: {source}")
        cap = cv2.VideoCapture(source)

    if not cap.isOpened():
        raise RuntimeError(f"No se pudo abrir la fuente: {source}")

    return cap


def configure_camera(cap: cv2.VideoCapture, args) -> None:
    """Aplica resolución y FPS a la cámara si es USB."""
    use_csi = getattr(args, "csi", False)
    if use_csi:
        return  # CSI ya viene configurada desde el pipeline

    source_str = args.source.strip() if args.source else "0"
    if source_str.isdigit():
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  args.cam_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.cam_height)
        cap.set(cv2.CAP_PROP_FPS,          args.cam_fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)   # evita frames acumulados (lag)

        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        f = cap.get(cv2.CAP_PROP_FPS)
        print(f"[INFO] Cámara configurada: {w}x{h} @ {f:.0f} FPS")


# ── YOLO ROI ──────────────────────────────────────────────────────────────────
def detect_target_roi_with_yolo(img, model, conf=0.4):
    """Detect target with YOLO and return largest bbox + cropped ROI."""
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
            best = {"x1": x1, "y1": y1, "x2": x2, "y2": y2,
                    "conf": score, "area": area}

    if best is None:
        return None

    pad = max(4, int(min(best["x2"] - best["x1"],
                         best["y2"] - best["y1"]) * 0.05))
    rx1 = max(0, best["x1"] - pad)
    ry1 = max(0, best["y1"] - pad)
    rx2 = min(w_img, best["x2"] + pad)
    ry2 = min(h_img, best["y2"] + pad)

    roi = img[ry1:ry2, rx1:rx2]
    if roi.size == 0:
        return None

    return {"bbox": (rx1, ry1, rx2, ry2), "conf": best["conf"], "roi": roi}


# ── Color HSV ─────────────────────────────────────────────────────────────────
def classify_color_hsv(h, s, v):
    if v < 55:                                           return "black"
    if 85 < h <= 135 and s > 40 and v < 80:             return "black"  # azul oscuro
    if s < 35 and v > 180:                               return "white"
    if s < 35:                                           return "gray"
    if 18 <= h <= 35 and s > 50:                         return "yellow"
    if 35 < h <= 85 and s > 30:                          return "green"
    if 85 < h <= 135 and s > 40:                         return "blue"
    if (h <= 18 or h >= 165) and s > 40 and v > 50:     return "red"
    return "unknown"


# ── Círculo principal ─────────────────────────────────────────────────────────
def find_largest_circle(img):
    gray    = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (9, 9), 2)
    h_img, w_img = img.shape[:2]
    max_r = max(h_img, w_img) // 2
    min_r = int(min(h_img, w_img) * 0.15)

    best = None
    for dp in [1.0, 1.2, 1.5]:
        for p1 in [50, 80, 120]:
            for p2 in [30, 50, 70]:
                circles = cv2.HoughCircles(
                    blurred, cv2.HOUGH_GRADIENT,
                    dp=dp, minDist=max(h_img, w_img) // 4,
                    param1=p1, param2=p2,
                    minRadius=min_r, maxRadius=max_r,
                )
                if circles is not None:
                    for c in circles[0]:
                        ccx, ccy, cr = int(c[0]), int(c[1]), int(c[2])
                        if ccx - cr < 0 or ccx + cr >= w_img: continue
                        if ccy - cr < 0 or ccy + cr >= h_img: continue
                        if best is None or cr > best[2]:
                            best = (ccx, ccy, cr)

    # Contour fallback
    edges = cv2.Canny(blurred, 30, 100)
    edges = cv2.dilate(edges, None, iterations=1)
    cnts, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in cnts:
        area = cv2.contourArea(cnt)
        if area < 500: continue
        peri = cv2.arcLength(cnt, True)
        if peri == 0: continue
        if 4 * np.pi * area / peri ** 2 > 0.5:
            (cx, cy), radius = cv2.minEnclosingCircle(cnt)
            r = int(radius); ccx, ccy = int(cx), int(cy)
            if r < min_r: continue
            if ccx - r < 0 or ccx + r >= w_img: continue
            if ccy - r < 0 or ccy + r >= h_img: continue
            if best is None or r > best[2]:
                best = (ccx, ccy, r)

    if best is None:
        print("[WARN] No circle fully within image bounds found")
    return best


# ── Anillos ───────────────────────────────────────────────────────────────────
def classify_ring(hsv, cx, cy, r_inner, r_outer, n_angles=90):
    h_img, w_img = hsv.shape[:2]
    angles  = np.linspace(0, 2 * np.pi, n_angles, endpoint=False)
    n_radial = max(5, (r_outer - r_inner) // 2)
    radii   = np.linspace(r_inner, r_outer, n_radial, endpoint=False)

    votes = {}
    for r in radii:
        xs = (cx + r * np.cos(angles)).astype(int)
        ys = (cy + r * np.sin(angles)).astype(int)
        valid = (xs >= 0) & (xs < w_img) & (ys >= 0) & (ys < h_img)
        for x, y, ok in zip(xs, ys, valid):
            if not ok: continue
            h, s, v = int(hsv[y, x, 0]), int(hsv[y, x, 1]), int(hsv[y, x, 2])
            c = classify_color_hsv(h, s, v)
            if c not in ("unknown", "white", "gray"):
                votes[c] = votes.get(c, 0) + 1

    return max(votes, key=votes.get) if votes else "unknown"


# ── Víctima ───────────────────────────────────────────────────────────────────
def classify_victim(ring_colors):
    total = sum(COLOR_VALUE.get(c, 0) for c in ring_colors)
    return VICTIM_NAMES.get(total, "Fake_target"), total


def vote_label(label_history, new_label):
    label_history.append(new_label)
    counts = {}
    for lbl in label_history:
        counts[lbl] = counts.get(lbl, 0) + 1
    return max(counts, key=counts.get)


# ── Frame principal ───────────────────────────────────────────────────────────
def detect_bullseye_frame(frame, model=None, yolo_conf=0.4, label_history=None):
    img        = frame.copy()
    orig       = frame.copy()
    bbox       = None
    yolo_score = None

    if model is not None:
        yolo_result = detect_target_roi_with_yolo(img, model, conf=yolo_conf)
        if yolo_result is None:
            print("[WARN] YOLO did not find a valid target box")
            return None
        bbox       = yolo_result["bbox"]
        yolo_score = yolo_result["conf"]
        img        = yolo_result["roi"]
        print(f"[INFO] YOLO bbox={bbox} conf={yolo_score:.2f}")

    hsv            = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h_img, w_img   = img.shape[:2]

    result = find_largest_circle(img)
    if result is None:
        print("[INFO] No valid target found")
        return None
    cx, cy, outer_r = result
    print(f"[INFO] Circle → center=({cx},{cy}) r={outer_r}")

    Y, X = np.ogrid[:h_img, :w_img]
    dist = np.sqrt((X - cx) ** 2 + (Y - cy) ** 2).astype(np.float32)

    ring_data = []
    for i in range(N_RINGS):
        r_outer = int(outer_r * (N_RINGS - i) / N_RINGS)
        r_inner = int(outer_r * (N_RINGS - i - 1) / N_RINGS)
        color   = classify_ring(hsv, cx, cy, r_inner, r_outer)

        mask    = ((dist >= r_inner) & (dist < r_outer)).astype(np.uint8)
        bgr_px  = img[mask == 1]
        hsv_px  = hsv[mask == 1]
        avg_rgb = tuple(np.mean(bgr_px, axis=0).astype(int)[[2, 1, 0]]) if len(bgr_px) else (0, 0, 0)
        avg_hsv = tuple(np.mean(hsv_px, axis=0).astype(int)) if len(hsv_px) else (0, 0, 0)

        ring_data.append({
            "ring": i, "r_outer": r_outer, "r_inner": r_inner,
            "color": color, "avg_rgb": avg_rgb,
            "avg_hsv": avg_hsv, "n_pixels": len(bgr_px),
        })

    detected     = [d["color"] for d in ring_data]
    victim_label, ring_sum = classify_victim(detected)
    stable_label = vote_label(label_history, victim_label) if label_history is not None else victim_label

    # Consola
    print()
    print("=" * 70)
    print("  BULLSEYE ANALYSIS (outside -> inside)")
    print("=" * 70)
    for d in ring_data:
        r, g, b = d["avg_rgb"]
        h, s, v = d["avg_hsv"]
        exp = EXPECTED_ORDER[d["ring"]]
        ok  = "OK" if d["color"] == exp else "FAIL"
        val = COLOR_VALUE.get(d["color"], 0)
        print(f"  Ring {d['ring']} | {d['color']:>8s} (exp: {exp:>7s}) [{ok:>4s}] "
              f"val={val:+d} | RGB=({r:3d},{g:3d},{b:3d}) | "
              f"HSV=({h:3d},{s:3d},{v:3d}) | r=[{d['r_inner']:3d},{d['r_outer']:3d}]")
    print(f"\n  Detected : {detected}")
    print(f"  Expected : {EXPECTED_ORDER}")
    print(f"  Ring sum : {ring_sum}  →  raw={victim_label}  stable={stable_label}")
    print(f"\n  {'[OK] VALID TARGET' if victim_label != 'Fake_target' else '[!!] FAKE TARGET'}"
          f"  —  {stable_label}")
    print("=" * 70)

    # Visualización
    vis        = orig.copy()
    color_draw = {
        "green": (0, 255, 0), "black": (80, 80, 80), "red": (0, 0, 255),
        "blue": (255, 0, 0),  "yellow": (0, 255, 255), "unknown": (128, 128, 128),
    }

    gx, gy = cx, cy
    if bbox is not None:
        x1, y1, x2, y2 = bbox
        gx, gy = cx + x1, cy + y1
        bcolor = BOX_COLOR.get(stable_label, (255, 255, 255))
        cv2.rectangle(vis, (x1, y1), (x2, y2), bcolor, 2)
        cv2.putText(vis, f"YOLO {yolo_score:.2f}", (x1, max(15, y1 - 7)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, bcolor, 2)

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
        cv2.putText(vis, d["color"], (lx, ly), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 2)
        cv2.putText(vis, d["color"], (lx, ly), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    cv2.circle(vis, (gx, gy), 4, (0, 0, 255), -1)

    tag_color = BOX_COLOR.get(stable_label, (200, 200, 200))
    cv2.putText(vis, stable_label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 3)
    cv2.putText(vis, stable_label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, tag_color, 2)
    cv2.putText(vis, f"sum={ring_sum}  rings: {' '.join(detected)}", (10, 55),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1)

    return {
        "center": (cx, cy), "outer_radius": outer_r,
        "rings": ring_data, "detected": detected,
        "ring_sum": ring_sum, "victim_label": victim_label,
        "stable_label": stable_label, "bbox": bbox,
        "yolo_conf": yolo_score, "vis": vis,
    }


def detect_bullseye(image_path, model=None, yolo_conf=0.4, label_history=None):
    img = cv2.imread(image_path)
    if img is None:
        print(f"[ERROR] Cannot read image: {image_path}")
        return None
    result = detect_bullseye_frame(img, model=model, yolo_conf=yolo_conf,
                                   label_history=label_history)
    if result is None:
        return None
    out_path = os.path.splitext(image_path)[0] + "_result.png"
    cv2.imwrite(out_path, result["vis"])
    print(f"\n[INFO] Visualization: {out_path}")
    return result


# ── Entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Bullseye detector with victim classification")
    parser.add_argument("image_path", nargs="?", default="",
                        help="Ruta a imagen (opcional si se usa --source o --csi)")
    parser.add_argument("--source", default="",
                        help="Fuente: ruta imagen/video o índice cámara USB (ej: 0, 1)")
    parser.add_argument("--csi", action="store_true", default=False,
                        help="Usar cámara CSI de Raspberry Pi via libcamera/GStreamer")
    parser.add_argument("--cam-width",  type=int, default=640,
                        help="Ancho de captura de cámara (default: 640)")
    parser.add_argument("--cam-height", type=int, default=480,
                        help="Alto de captura de cámara (default: 480)")
    parser.add_argument("--cam-fps",    type=int, default=30,
                        help="FPS de captura de cámara (default: 30)")
    parser.add_argument("--model", default="target.pt", help="Modelo YOLO (.pt)")
    parser.add_argument("--conf",  type=float, default=0.4, help="Umbral de confianza YOLO")
    parser.add_argument("--no-yolo", action="store_true", help="Saltar YOLO, analizar imagen completa")
    parser.add_argument("--show",  action="store_true", default=True,
                        help="Mostrar ventana de visualización")
    parser.add_argument("--save",  default="",
                        help="Guardar salida (imagen: out.png, video: out.mp4)")
    args = parser.parse_args()

    # Resolver fuente
    input_source = args.source.strip() if args.source else args.image_path.strip()

    # Cargar modelo YOLO
    yolo_model = None
    if not args.no_yolo:
        model_path = resolve_model_path(args.model)
        print(f"[INFO] Cargando modelo YOLO: {model_path}")
        yolo_model = YOLO(str(model_path))

    label_history = deque(maxlen=TEMPORAL_VOTE_SIZE)
    img_extensions = (".jpg", ".jpeg", ".png", ".bmp", ".webp")

    # ── Modo imagen estática ──────────────────────────────────────────────────
    is_image = (not args.csi) and input_source.lower().endswith(img_extensions)

    if is_image:
        result = detect_bullseye(input_source, model=yolo_model,
                                 yolo_conf=args.conf, label_history=label_history)
        if result and args.show:
            cv2.imshow("Bullseye Detector", result["vis"])
            cv2.waitKey(0)
            cv2.destroyAllWindows()

    # ── Modo video / cámara ───────────────────────────────────────────────────
    else:
        # Si no se dio fuente y no es CSI, usar cámara 0
        if not args.csi and not input_source:
            args.source = "0"

        try:
            cap = open_camera(args)
        except RuntimeError as e:
            print(f"[ERROR] {e}")
            raise SystemExit(1)

        configure_camera(cap, args)

        # Preparar writer de video si se pidió guardar
        writer = None
        if args.save and not args.save.lower().endswith(img_extensions):
            fps = cap.get(cv2.CAP_PROP_FPS) or 30
            fw  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            fh  = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            writer = cv2.VideoWriter(
                args.save,
                cv2.VideoWriter_fourcc(*"mp4v"),
                fps, (fw, fh),
            )
            print(f"[INFO] Guardando video en: {args.save}")

        print("[INFO] Corriendo... presiona Q para salir.")
        while True:
            ok, frame = cap.read()
            if not ok:
                print("[WARN] Frame no disponible, terminando.")
                break

            result = detect_bullseye_frame(
                frame, model=yolo_model,
                yolo_conf=args.conf,
                label_history=label_history,
            )
            vis = result["vis"] if result else frame

            if args.show:
                cv2.imshow("Bullseye Detector", vis)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            if writer:
                writer.write(vis)

        cap.release()
        if writer:
            writer.release()
        cv2.destroyAllWindows()
        print("[INFO] Listo.")