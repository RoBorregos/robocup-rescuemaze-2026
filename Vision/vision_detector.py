from __future__ import annotations

from pathlib import Path

import cv2
from ultralytics import YOLO

import Constants
from vision_protocol import (
    CAM_LEFT,
    CAM_RIGHT,
    VICTIM_NONE,
    VICTIM_OMEGA,
    VICTIM_PHI,
    VICTIM_PSI,
)


def resolve_model_path() -> Path:
    base = Path(__file__).resolve().parent
    onnx_path = base / "weights" / "best.onnx"
    pt_path = base / "weights" / "best.pt"
    if onnx_path.exists():
        return onnx_path
    if pt_path.exists():
        return pt_path
    raise FileNotFoundError("No model found in Vision/weights (expected best.onnx or best.pt)")


class VisionDetector:
    def __init__(self) -> None:
        self.model_path = resolve_model_path()
        self.model = YOLO(str(self.model_path))
        self.conf = getattr(Constants, "vision_conf_threshold", 0.45)
        self.imgsz = getattr(Constants, "vision_imgsz", 640)

        self.cam_right_idx = getattr(Constants, "camera_right_index", 0)
        self.cam_left_idx = getattr(Constants, "camera_left_index", 1)

        self.cap_right = cv2.VideoCapture(self.cam_right_idx)
        self.cap_left = cv2.VideoCapture(self.cam_left_idx)

        if not self.cap_right.isOpened() and not self.cap_left.isOpened():
            raise RuntimeError("Could not open any camera (RIGHT/LEFT)")

    def close(self) -> None:
        if self.cap_right is not None:
            self.cap_right.release()
        if self.cap_left is not None:
            self.cap_left.release()

    @staticmethod
    def _class_to_victim_id(class_name: str) -> int:
        text = class_name.strip().lower()
        if "phi" in text or text == "h" or "harm" in text:
            return VICTIM_PHI
        if "psi" in text or text == "s" or "stable" in text:
            return VICTIM_PSI
        if "omega" in text or text == "u" or "unharm" in text:
            return VICTIM_OMEGA
        return VICTIM_NONE

    def detect_victim(self, camera_id: int) -> int:
        cap = self.cap_right if camera_id == CAM_RIGHT else self.cap_left
        if cap is None or not cap.isOpened():
            return VICTIM_NONE

        ok, frame = cap.read()
        if not ok or frame is None:
            return VICTIM_NONE

        results = self.model.predict(frame, conf=self.conf, imgsz=self.imgsz, verbose=False)
        if not results:
            return VICTIM_NONE

        result = results[0]
        boxes = result.boxes
        names = result.names
        if boxes is None or len(boxes) == 0:
            return VICTIM_NONE

        best_idx = int(boxes.conf.argmax().item())
        class_id = int(boxes.cls[best_idx].item())
        class_name = names.get(class_id, str(class_id)) if isinstance(names, dict) else str(class_id)
        return self._class_to_victim_id(class_name)
