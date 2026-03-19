import importlib
import time
from pathlib import Path

from ultralytics import YOLO

from esp_protocol_link import (
    CAM_LEFT,
    CAM_RIGHT,
    CAM_LABEL,
    VICTIM_LABEL,
    VICTIM_NONE,
    VICTIM_OMEGA,
    VICTIM_PHI,
    VICTIM_PSI,

)


class DualCameraDetector:
    def __init__(
        self,
        model_path: Path,
        right_device_index: int,
        left_device_index: int,
        conf_threshold: float,
        img_size: int,
    ):
        self.model = YOLO(str(model_path))
        self.conf_threshold = conf_threshold
        self.img_size = img_size
        self.cameras = {
            CAM_RIGHT: self._init_camera(right_device_index),
            CAM_LEFT: self._init_camera(left_device_index),
        }

    @staticmethod
    def _init_camera(camera_num: int):
        picamera2_module = importlib.import_module("picamera2")
        libcamera_module = importlib.import_module("libcamera")

        Picamera2 = picamera2_module.Picamera2
        controls = libcamera_module.controls
        Transform = libcamera_module.Transform

        camera = Picamera2(camera_num=camera_num)
        camera.configure(
            camera.create_preview_configuration(
                main={"size": (320, 240), "format": "RGB888"},
                transform=Transform(hflip=1, vflip=1),
            )
        )
        camera.start()
        time.sleep(0.2)

        try:
            camera.set_controls({"AfMode": controls.AfModeEnum.Continuous})
        except Exception:
            pass

        return camera

    @staticmethod
    def _normalize_label(label: str) -> str:
        clean = label.strip().upper().replace("-", "_").replace(" ", "_")
        return clean

    def _label_to_victim_id(self, label: str) -> int:
        normalized = self._normalize_label(label)

        if normalized in {"PHI", "HEATED", "H"}:
            return VICTIM_PHI
        if normalized in {"PSI", "STABLE", "S"}:
            return VICTIM_PSI
        if normalized in {"OMEGA", "UNHEATED", "U"}:
            return VICTIM_OMEGA
        if normalized in {"N", "NONE", "NO_VICTIM"}:
            return VICTIM_NONE
        return VICTIM_NONE

    def detect_for_camera(self, cam_id: int) -> int:
        frame = self.cameras[cam_id].capture_array()
        if frame is None:
            return VICTIM_NONE

        results = self.model.predict(
            frame,
            conf=self.conf_threshold,
            imgsz=self.img_size,
            iou=0.45,
            agnostic_nms=True,
            verbose=False,
        )

        best_label = None
        best_conf = -1.0
        for result in results:
            for box in result.boxes:
                confidence = float(box.conf[0])
                if confidence > best_conf:
                    cls_id = int(box.cls[0])
                    best_label = self.model.names[cls_id]
                    best_conf = confidence

        if best_label is None:
            return VICTIM_NONE

        victim_id = self._label_to_victim_id(best_label)
        print(
            f"[DETECT] camera={CAM_LABEL[cam_id]} label={best_label} conf={best_conf:.2f} "
            f"-> victim={VICTIM_LABEL[victim_id]}"
        )
        return victim_id

    def close(self):
        for camera in self.cameras.values():
            try:
                camera.stop()
            except Exception:
                pass
