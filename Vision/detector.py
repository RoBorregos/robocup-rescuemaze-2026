"""YOLO-based victim detector for LEFT and RIGHT cameras.

Handles camera initialization (OpenCV/Picamera2), frame capture,
inference execution, and mapping classes to victim IDs.
"""

from __future__ import annotations

import os
import importlib
import time
from pathlib import Path
from typing import Optional

# Avoid noisy/buggy obsensor probing on Raspberry Pi OpenCV builds
os.environ.setdefault("OPENCV_VIDEOIO_PRIORITY_OBSENSOR", "0")

import cv2
from ultralytics import YOLO

import Constants
from protocol import (
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
    raise FileNotFoundError(
        "No model found in Vision/weights (expected best.onnx or best.pt)"
    )


class VisionDetector:
    def __init__(self) -> None:
        self.model_path = resolve_model_path()
        self.model = YOLO(str(self.model_path))
        self._target_detector = None
        self.conf = getattr(Constants, "vision_conf_threshold", 0.45)
        self.imgsz = getattr(Constants, "vision_imgsz", 640)
        self.iou = getattr(Constants, "vision_iou_threshold", 0.50)
        self.device = getattr(Constants, "vision_device", "cpu")
        self.inference_frames = max(
            1, int(getattr(Constants, "vision_inference_frames", 1))
        )
        self.inference_timeout_ms = int(
            getattr(Constants, "vision_inference_timeout_ms", 180)
        )
        self.force_frame_size = bool(
            getattr(Constants, "vision_force_frame_size", False)
        )
        self.frame_width = getattr(Constants, "vision_frame_width", 640)
        self.frame_height = getattr(Constants, "vision_frame_height", 480)
        self.picamera_width = int(getattr(Constants, "vision_picamera_width", 1640))
        self.picamera_height = int(getattr(Constants, "vision_picamera_height", 1232))
        self.picamera_prefer_full_fov = bool(
            getattr(Constants, "vision_picamera_prefer_full_fov", True)
        )
        self.picamera_right_idx = int(
            getattr(Constants, "vision_picamera_right_index", 0)
        )
        self.picamera_left_idx = int(
            getattr(Constants, "vision_picamera_left_index", 1)
        )
        self.prefer_picamera2 = bool(
            getattr(Constants, "vision_prefer_picamera2", True)
        )

        self.cam_right_idx = getattr(Constants, "camera_right_index", 0)
        self.cam_left_idx = getattr(Constants, "camera_left_index", 1)

        self.picam_right = None
        self.picam_left = None

        self.cap_right = cv2.VideoCapture()
        self.cap_left = cv2.VideoCapture()

        if self.prefer_picamera2:
            self.picam_right = self._init_picamera(self.picamera_right_idx)
            self.picam_left = self._init_picamera(self.picamera_left_idx)

        if self.picam_right is None or self.picam_left is None:
            self.cap_right = self._open_capture(self.cam_right_idx)
            self.cap_left = self._open_capture(self.cam_left_idx)
            self._configure_capture(self.cap_right)
            self._configure_capture(self.cap_left)

        if (self.picam_right is None or self.picam_left is None) and (
            not self.cap_right.isOpened() or not self.cap_left.isOpened()
        ):
            self._autodetect_cameras()

        if (
            self.picam_right is None
            and self.picam_left is None
            and not self.cap_right.isOpened()
            and not self.cap_left.isOpened()
        ):
            raise RuntimeError("Could not open any camera (RIGHT/LEFT)")

        print(
            f"[VISION] Model={self.model_path.name} conf={self.conf} iou={self.iou} "
            f"imgsz={self.imgsz} device={self.device} frames={self.inference_frames}"
        )
        print(
            f"[VISION] force_frame_size={self.force_frame_size} "
            f"target_size={self.frame_width}x{self.frame_height}"
        )
        print(
            f"[VISION] picamera_prefer_full_fov={self.picamera_prefer_full_fov} "
            f"picamera_size={self.picamera_width}x{self.picamera_height}"
        )
        print(
            f"[VISION] picamera_map RIGHT={self.picamera_right_idx} "
            f"LEFT={self.picamera_left_idx}"
        )
        print(f"[VISION] prefer_picamera2={self.prefer_picamera2}")
        print(
            f"[VISION] RIGHT idx={self.cam_right_idx} open={self.cap_right.isOpened()} picam={self.picam_right is not None} | "
            f"LEFT idx={self.cam_left_idx} open={self.cap_left.isOpened()} picam={self.picam_left is not None}"
        )
        names = getattr(self.model, "names", None)
        if isinstance(names, dict):
            print(f"[VISION] model classes={names}")

        self._check_camera_reads()
        self._activate_picamera2_if_needed()
        self._check_camera_reads()

    @staticmethod
    def _decode_fourcc(value: float) -> str:
        code = int(value)
        return "".join(chr((code >> (8 * i)) & 0xFF) for i in range(4)).strip("\x00")

    def _log_capture_info(self, cap: Optional[cv2.VideoCapture], label: str) -> None:
        if cap is None or not cap.isOpened():
            print(f"[VISION] {label}: capture not opened")
            return

        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = float(cap.get(cv2.CAP_PROP_FPS))
        fourcc = self._decode_fourcc(cap.get(cv2.CAP_PROP_FOURCC))

        backend_name = "UNKNOWN"
        get_backend_name = getattr(cap, "getBackendName", None)
        if callable(get_backend_name):
            try:
                backend_name = get_backend_name()
            except Exception:
                backend_name = "UNKNOWN"

        print(
            f"[VISION] {label}: backend={backend_name} "
            f"size={width}x{height} fps={fps:.2f} fourcc={fourcc or 'N/A'}"
        )

    def _init_picamera(self, camera_num: int):
        try:
            Picamera2 = importlib.import_module("picamera2").Picamera2
        except Exception:
            return None
        try:
            picam = Picamera2(camera_num)
            if self.force_frame_size:
                config = picam.create_preview_configuration(
                    main={"size": (self.frame_width, self.frame_height)}
                )
            elif self.picamera_prefer_full_fov:
                config = picam.create_preview_configuration(
                    main={"size": (self.picamera_width, self.picamera_height)}
                )
            else:
                config = picam.create_preview_configuration()
            picam.configure(config)
            self._apply_autofocus_controls(picam)
            picam.start()
            return picam
        except Exception as exc:
            print(f"[VISION] Picamera2 init failed for camera {camera_num}: {exc}")
            return None

    def _apply_autofocus_controls(self, picam) -> None:
        mode_name = (
            str(getattr(Constants, "camera_autofocus_mode", "continuous"))
            .strip()
            .lower()
        )
        lens_position = float(getattr(Constants, "camera_lens_position", 1.5))

        try:
            controls = {}
            if mode_name == "continuous":
                controls["AfMode"] = 2  # Continuous autofocus
            elif mode_name == "auto":
                controls["AfMode"] = 1  # Single autofocus
                controls["AfTrigger"] = 0
            elif mode_name == "manual":
                controls["AfMode"] = 0  # Manual focus
                controls["LensPosition"] = lens_position
            elif mode_name == "off":
                controls["AfMode"] = 0
            else:
                controls["AfMode"] = 2

            picam.set_controls(controls)
            print(f"[VISION] autofocus mode={mode_name} controls={controls}")
        except Exception as exc:
            print(f"[VISION] autofocus config skipped: {exc}")

    def _activate_picamera2_if_needed(self) -> None:
        if self.prefer_picamera2 and self.picam_right is not None and self.picam_left is not None:
            return

        right_ok = self._can_read_once(self.cap_right)
        left_ok = self._can_read_once(self.cap_left)

        if not right_ok or not left_ok:
            self._autodetect_readable_cameras()
            right_ok = self._can_read_once(self.cap_right)
            left_ok = self._can_read_once(self.cap_left)

        try:
            importlib.import_module("picamera2")
        except Exception:
            if right_ok and left_ok:
                return
            print(
                "[VISION] Picamera2 not available. Install with: sudo apt install -y python3-picamera2"
            )
            return

        print(
            "[VISION] Switching to Picamera2 backend for cameras that fail with OpenCV..."
        )

        camera_count = 0
        camera_info = []
        try:
            Picamera2 = importlib.import_module("picamera2").Picamera2
            camera_info = list(Picamera2.global_camera_info())
            camera_count = len(camera_info)
        except Exception:
            camera_count = 0
        print(f"[VISION] picamera2 available cameras={camera_count}")
        for idx, info in enumerate(camera_info):
            model = info.get("Model", "unknown") if isinstance(info, dict) else str(info)
            print(f"[VISION] picamera2[{idx}] model={model}")

        preferred_right = self.picamera_right_idx
        preferred_left = self.picamera_left_idx

        if preferred_right < 0:
            preferred_right = 0
        if preferred_left < 0:
            preferred_left = 1 if camera_count > 1 else 0

        if preferred_left == preferred_right:
            preferred_left = 1 if preferred_right == 0 else 0

        # Always try the configured Picamera2 indices first.
        # This is the reliable path for two Camera Module 3 devices, because
        # OpenCV/V4L2 can collapse both CSI cameras into the same index.
        if self.picam_right is None:
            self.picam_right = self._init_picamera(preferred_right)
        if self.picam_left is None:
            self.picam_left = self._init_picamera(preferred_left)

        if self.picam_right is None and right_ok:
            print("[VISION] RIGHT will keep using OpenCV fallback")
        if self.picam_left is None and left_ok:
            print("[VISION] LEFT will keep using OpenCV fallback")

    def _autodetect_readable_cameras(self) -> None:
        readable = []
        for index in range(0, 8):
            cap = self._open_capture(index)
            if cap.isOpened():
                ok, frame = cap.read()
                if ok and frame is not None:
                    readable.append(index)
            cap.release()

        if not readable:
            print("[VISION] No readable OpenCV cameras found during re-scan")
            return

        right_ok = self._can_read_once(self.cap_right)
        left_ok = self._can_read_once(self.cap_left)

        if not right_ok:
            new_right = readable[0]
            if self.cap_right is not None:
                self.cap_right.release()
            self.cam_right_idx = new_right
            self.cap_right = self._open_capture(self.cam_right_idx)
            self._configure_capture(self.cap_right)
            print(f"[VISION] RIGHT reassigned to readable OpenCV index={self.cam_right_idx}")

        used_idx = self.cam_right_idx if self._can_read_once(self.cap_right) else None
        left_candidates = [idx for idx in readable if idx != used_idx]

        if not left_ok:
            if left_candidates:
                new_left = left_candidates[0]
                if self.cap_left is not None:
                    self.cap_left.release()
                self.cam_left_idx = new_left
                self.cap_left = self._open_capture(self.cam_left_idx)
                self._configure_capture(self.cap_left)
                print(f"[VISION] LEFT reassigned to readable OpenCV index={self.cam_left_idx}")
            else:
                if self.cap_left is not None:
                    self.cap_left.release()
                self.cap_left = cv2.VideoCapture()
                print("[VISION] No second readable OpenCV camera for LEFT")

    def _can_read_once(self, cap: Optional[cv2.VideoCapture]) -> bool:
        if cap is None or not cap.isOpened():
            return False
        ok, frame = cap.read()
        return bool(ok and frame is not None)

    def _check_camera_reads(self) -> None:
        def test_cap(cap: cv2.VideoCapture, label: str) -> None:
            if cap is None or not cap.isOpened():
                print(f"[VISION] {label}: not opened")
                return
            self._log_capture_info(cap, label)
            ok_count = 0
            first_shape = None
            for _ in range(5):
                cv2.waitKey(1)
                ok, frame = cap.read()
                if ok and frame is not None:
                    ok_count += 1
                    if first_shape is None:
                        first_shape = frame.shape[:2]
            print(f"[VISION] {label}: warmup_reads_ok={ok_count}/5")
            if first_shape is not None:
                h, w = first_shape
                print(f"[VISION] {label}: first_frame_shape={w}x{h}")

        def test_picam(picam, label: str) -> None:
            if picam is None:
                print(f"[VISION] {label}: not opened")
                return
            ok_count = 0
            first_shape = None
            for _ in range(5):
                try:
                    frame = picam.capture_array()
                    if frame is not None:
                        ok_count += 1
                        if first_shape is None:
                            first_shape = frame.shape[:2]
                except Exception:
                    pass
            print(f"[VISION] {label}: warmup_reads_ok={ok_count}/5")
            if first_shape is not None:
                h, w = first_shape
                print(f"[VISION] {label}: first_frame_shape={w}x{h}")

        test_cap(self.cap_right, f"RIGHT(idx={self.cam_right_idx})")
        test_cap(self.cap_left, f"LEFT(idx={self.cam_left_idx})")
        test_picam(self.picam_right, "RIGHT(picamera2)")
        test_picam(self.picam_left, "LEFT(picamera2)")

    def _autodetect_cameras(self) -> None:
        available = []
        for index in range(0, 6):
            cap = self._open_capture(index)
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
            self.cap_right = self._open_capture(self.cam_right_idx)
            self._configure_capture(self.cap_right)

        if not self.cap_left.isOpened():
            self.cap_left.release()
            used_idx = self.cam_right_idx if self.cap_right.isOpened() else None
            left_candidates = [idx for idx in available if idx != used_idx]
            if left_candidates:
                self.cam_left_idx = left_candidates[0]
                self.cap_left = self._open_capture(self.cam_left_idx)
                self._configure_capture(self.cap_left)
            else:
                # Keep LEFT unopened so Picamera2 fallback can be assigned there.
                self.cap_left = cv2.VideoCapture()
                print(
                    "[VISION] No distinct OpenCV index left for LEFT; waiting for Picamera2 fallback"
                )

    def _open_capture(self, index: int) -> cv2.VideoCapture:
        cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            cap = cv2.VideoCapture(index)
        return cap

    def _configure_capture(self, cap: cv2.VideoCapture) -> None:
        if cap is None or not self.force_frame_size:
            return
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)

    def close(self) -> None:
        if self.cap_right is not None:
            self.cap_right.release()
        if self.cap_left is not None:
            self.cap_left.release()
        if self.picam_right is not None:
            try:
                self.picam_right.stop()
            except Exception:
                pass
        if self.picam_left is not None:
            try:
                self.picam_left.stop()
            except Exception:
                pass

    def read_frame(self, camera_id: int):
        if camera_id == CAM_RIGHT and self.picam_right is not None:
            try:
                frame = self.picam_right.capture_array()
                if frame is not None:
                    return True, cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            except Exception:
                pass
            if self.cap_right is not None:
                return self.cap_right.read()
            return False, None

        if self.picam_left is not None:
            try:
                frame = self.picam_left.capture_array()
                if frame is not None:
                    return True, cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            except Exception:
                pass
        if self.cap_left is not None:
            return self.cap_left.read()
        return False, None

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
        best_conf_global = -1.0
        best_class_name = None
        started = time.monotonic()

        for _ in range(self.inference_frames):
            elapsed_ms = (time.monotonic() - started) * 1000.0
            if elapsed_ms > self.inference_timeout_ms:
                break

            ok, frame = self.read_frame(camera_id)
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
            class_name = (
                names.get(class_id, str(class_id))
                if isinstance(names, dict)
                else str(class_id)
            )

            mapped = self._class_to_victim_id(class_name)
            if mapped != VICTIM_NONE:
                print(f"[VISION] top_class={class_name} conf={best_conf:.3f}")
                return mapped

            if best_conf > best_conf_global:
                best_conf_global = best_conf
                best_class_name = class_name

        if best_class_name is None:
            print(
                f"[VISION] no detections cam={'RIGHT' if camera_id == CAM_RIGHT else 'LEFT'}"
            )
            return VICTIM_NONE

        print(f"[VISION] top_class={best_class_name} conf={best_conf_global:.3f}")
        return self._class_to_victim_id(best_class_name)

    def detect_target(self, camera_id: int):
        """Detect a bullseye target using the same camera workflow."""
        if self._target_detector is None:
            from target_detector import TargetDetector

            self._target_detector = TargetDetector()

        ok, frame = self.read_frame(camera_id)
        if not ok or frame is None:
            cam_name = "RIGHT" if camera_id == CAM_RIGHT else "LEFT"
            print(f"[VISION] no frame available for {cam_name} target detection")
            return None

        return self._target_detector.detect_frame(frame)
