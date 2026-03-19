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
    if pt_path.exists():
        return pt_path
    if onnx_path.exists():
        return onnx_path
    raise FileNotFoundError("No model found in Vision/weights (expected best.onnx or best.pt)")


class VisionDetector:
    def __init__(self) -> None:
        self.model_path = resolve_model_path()
        self.model = YOLO(str(self.model_path))
        self.conf = getattr(Constants, "vision_conf_threshold", 0.45)
        self.imgsz = getattr(Constants, "vision_imgsz", 640)
        self.iou = getattr(Constants, "vision_iou_threshold", 0.50)
        self.device = getattr(Constants, "vision_device", "cpu")
        self.frame_width = getattr(Constants, "vision_frame_width", 640)
        self.frame_height = getattr(Constants, "vision_frame_height", 480)

        self.cam_right_idx = getattr(Constants, "camera_right_index", 0)
        self.cam_left_idx = getattr(Constants, "camera_left_index", 1)

        self.cap_right = cv2.VideoCapture(self.cam_right_idx)
        self.cap_left = cv2.VideoCapture(self.cam_left_idx)
        self._configure_capture(self.cap_right)
        self._configure_capture(self.cap_left)

        if not self.cap_right.isOpened() or not self.cap_left.isOpened():
            self._autodetect_cameras()

        if not self.cap_right.isOpened() and not self.cap_left.isOpened():
            raise RuntimeError("Could not open any camera (RIGHT/LEFT)")

        print(
            f"[VISION] Model={self.model_path.name} conf={self.conf} iou={self.iou} "
            f"imgsz={self.imgsz} device={self.device}"
        )
        print(
            f"[VISION] RIGHT idx={self.cam_right_idx} open={self.cap_right.isOpened()} | "
            f"LEFT idx={self.cam_left_idx} open={self.cap_left.isOpened()}"
        )
        names = getattr(self.model, "names", None)
        if isinstance(names, dict):
            print(f"[VISION] model classes={names}")

        self._check_camera_reads()

    def _check_camera_reads(self) -> None:
        def test_cap(cap: cv2.VideoCapture, label: str) -> None:
            if cap is None or not cap.isOpened():
                print(f"[VISION] {label}: not opened")
                return
            ok_count = 0
            for _ in range(5):
                ok, frame = cap.read()
                if ok and frame is not None:
                    ok_count += 1
            print(f"[VISION] {label}: warmup_reads_ok={ok_count}/5")

        test_cap(self.cap_right, f"RIGHT(idx={self.cam_right_idx})")
        test_cap(self.cap_left, f"LEFT(idx={self.cam_left_idx})")

    def _autodetect_cameras(self) -> None:
        available = []
        for index in range(0, 6):
            cap = cv2.VideoCapture(index)
            if cap.isOpened():
                ok, frame = cap.read()
                if ok and frame is not None:
                    available.append(index)
            cap.release()

        if not available:
            return

        if not self.cap_right.isOpened():
            self.cap_right.release()
            self.cam_right_idx = available[0]
            self.cap_right = cv2.VideoCapture(self.cam_right_idx)
            self._configure_capture(self.cap_right)

        if not self.cap_left.isOpened():
            self.cap_left.release()
            if len(available) > 1:
                self.cam_left_idx = available[1]
            else:
                self.cam_left_idx = available[0]
            self.cap_left = cv2.VideoCapture(self.cam_left_idx)
            self._configure_capture(self.cap_left)

    def _configure_capture(self, cap: cv2.VideoCapture) -> None:
        if cap is None:
            return
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)

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
        print(f"[VISION] unmapped class='{class_name}' -> NONE")
        return VICTIM_NONE

    def detect_victim(self, camera_id: int) -> int:
        cap = self.cap_right if camera_id == CAM_RIGHT else self.cap_left
        if cap is None or not cap.isOpened():
            # Fallback to the other camera if requested one is unavailable
            fallback = self.cap_left if camera_id == CAM_RIGHT else self.cap_right
            if fallback is None or not fallback.isOpened():
                return VICTIM_NONE
            cap = fallback

        best_conf_global = -1.0
        best_class_name = None

        for _ in range(3):
            ok, frame = cap.read()
            if not ok or frame is None:
                continue

            results = self.model.predict(
                frame,
                conf=self.conf,
                iou=self.iou,
                imgsz=self.imgsz,
                device=self.device,
                verbose=False,
            )
            if not results:
                continue

            result = results[0]
            boxes = result.boxes
            names = result.names
            if boxes is None or len(boxes) == 0:
                continue

            best_idx = int(boxes.conf.argmax().item())
            best_conf = float(boxes.conf[best_idx].item())
            class_id = int(boxes.cls[best_idx].item())
            class_name = names.get(class_id, str(class_id)) if isinstance(names, dict) else str(class_id)

            if best_conf > best_conf_global:
                best_conf_global = best_conf
                best_class_name = class_name

        if best_class_name is None:
            print(f"[VISION] no detections cam={'RIGHT' if camera_id == CAM_RIGHT else 'LEFT'}")
            return VICTIM_NONE

        print(f"[VISION] top_class={best_class_name} conf={best_conf_global:.3f}")
        return self._class_to_victim_id(best_class_name)
