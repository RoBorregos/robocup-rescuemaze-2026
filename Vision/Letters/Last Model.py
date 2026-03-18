import cv2
import importlib
import time
from ultralytics import YOLO
from pathlib import Path
import numpy as np

BASE_DIR = Path(__file__).resolve().parent
MODEL_PATH = BASE_DIR / "weights" / "best.pt"
model = YOLO(str(MODEL_PATH))


def init_csi_camera(camera_num, width=320, height=240):
    picamera2_module = importlib.import_module("picamera2")
    libcamera_module = importlib.import_module("libcamera")

    Picamera2 = picamera2_module.Picamera2
    controls = libcamera_module.controls
    Transform = libcamera_module.Transform

    camera = Picamera2(camera_num=camera_num)
    camera.configure(
        camera.create_preview_configuration(
            main={"size": (width, height), "format": "RGB888"},
            transform=Transform(hflip=1, vflip=1)
        )
    )
    camera.start()
    time.sleep(0.2)

    try:
        camera.set_controls({"AfMode": controls.AfModeEnum.Continuous})
    except Exception:
        pass

    return camera


def detect_and_annotate(frame, side):
    enhanced_frame = cv2.convertScaleAbs(frame, alpha=1.3, beta=30)
    results = model.predict(
        enhanced_frame,
        imgsz=416,
        conf=0.6,
        verbose=False
    )

    annotated = results[0].plot()
    detections = []

    for result in results:
        for box in result.boxes:
            cls_id = int(box.cls[0])
            label = model.names[cls_id]
            confidence = float(box.conf[0])
            if confidence > 0.6:
                detections.append((label, confidence))
                print(f"[DETECT] {side}: {label} ({confidence:.2f})")

    color = (0, 255, 0) if side == "LEFT" else (255, 128, 0)
    cv2.putText(
        annotated,
        f"CAM {side}",
        (10, 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        color,
        2
    )

    if detections:
        summary = " | ".join([f"{label} {confidence:.2f}" for label, confidence in detections])
        cv2.rectangle(annotated, (0, 210), (320, 240), (0, 0, 0), -1)
        cv2.putText(
            annotated,
            summary,
            (5, 232),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 255, 255),
            1
        )

    return annotated, detections


def main():
    camera_left = None
    camera_right = None

    try:
        camera_left = init_csi_camera(camera_num=0)
        print("[INFO] Camera 0 started (LEFT)")
        camera_right = init_csi_camera(camera_num=1)
        print("[INFO] Camera 1 started (RIGHT)")
    except Exception as error:
        print(f"[ERROR] Failed to initialize CSI cameras: {error}")
        return

    print("\n[INFO] Starting dual detection... Press ESC to exit\n")

    try:
        while True:
            frame_left = camera_left.capture_array()
            frame_right = camera_right.capture_array()

            if frame_left is None or frame_right is None:
                print("[ERROR] Frame not received from one or both cameras")
                break

            ann_left, det_left = detect_and_annotate(frame_left, "LEFT")
            ann_right, det_right = detect_and_annotate(frame_right, "RIGHT")

            combined = np.hstack((ann_left, ann_right))
            cv2.line(combined, (320, 0), (320, 240), (255, 255, 255), 2)

            if det_left or det_right:
                sides = []
                if det_left:
                    sides.append("LEFT")
                if det_right:
                    sides.append("RIGHT")
                message = "DETECTION: " + " & ".join(sides)
                cv2.rectangle(combined, (0, 0), (640, 28), (0, 0, 180), -1)
                cv2.putText(
                    combined,
                    message,
                    (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (255, 255, 255),
                    2
                )

            cv2.imshow("Dual YOLO Detection", combined)
            if cv2.waitKey(1) & 0xFF == 27:
                break
    finally:
        cv2.destroyAllWindows()
        if camera_left is not None:
            camera_left.stop()
        if camera_right is not None:
            camera_right.stop()
        print("[INFO] Cameras stopped")


if __name__ == "__main__":
    main()