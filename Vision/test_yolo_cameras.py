#!/usr/bin/env python3
from __future__ import annotations

import os
import time
import cv2
import numpy as np

from vision_detector import VisionDetector
from vision_protocol import CAM_LEFT, CAM_RIGHT


def infer_and_draw(detector: VisionDetector, frame, label: str):
    results = detector.model.predict(
        frame,
        conf=detector.conf,
        iou=detector.iou,
        imgsz=detector.imgsz,
        device=detector.device,
        verbose=False,
    )

    if not results:
        overlay = frame.copy()
        cv2.putText(overlay, f"{label}: no result", (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        return overlay

    result = results[0]
    annotated = result.plot()
    boxes = result.boxes
    count = int(len(boxes)) if boxes is not None else 0

    status = f"{label}: detections={count}"
    cv2.putText(annotated, status, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
    return annotated


def run_preview() -> None:
    detector = VisionDetector()
    has_display = bool(os.environ.get("DISPLAY"))
    try:
        print("Press q to quit preview")
        if not has_display:
            print("[HEADLESS] DISPLAY not found; running console mode.")
            print("[HEADLESS] Press Ctrl+C to stop.")
        else:
            print("If windows don't appear, run with desktop (not SSH headless).")

        while True:
            ok_r, frame_r = detector.cap_right.read() if detector.cap_right is not None else (False, None)
            ok_l, frame_l = detector.cap_left.read() if detector.cap_left is not None else (False, None)

            if ok_r and frame_r is not None:
                annotated_r = infer_and_draw(detector, frame_r, "RIGHT")
                if has_display:
                    cv2.imshow("RIGHT", annotated_r)
            else:
                blank = np.full((480, 640, 3), 255, dtype=np.uint8)
                cv2.putText(blank, "RIGHT: camera read failed", (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                if has_display:
                    cv2.imshow("RIGHT", blank)

            if ok_l and frame_l is not None:
                annotated_l = infer_and_draw(detector, frame_l, "LEFT")
                if has_display:
                    cv2.imshow("LEFT", annotated_l)
            else:
                blank = np.full((480, 640, 3), 255, dtype=np.uint8)
                cv2.putText(blank, "LEFT: camera read failed", (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                if has_display:
                    cv2.imshow("LEFT", blank)

            if not has_display:
                print(f"[HEADLESS] RIGHT_OK={ok_r} LEFT_OK={ok_l}")
                time.sleep(0.4)
                continue

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
    finally:
        detector.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    run_preview()
