#!/usr/bin/env python3
import argparse
import glob
import importlib
import struct
import sys
import time
from pathlib import Path

from serial import Serial
from serial.serialutil import SerialException
from ultralytics import YOLO

import Constants

HEADER0 = 0xFF
HEADER1 = 0xAA

CMD_REQUEST_RIGHT = 0x02
CMD_REQUEST_LEFT = 0x03

CAM_RIGHT = 0
CAM_LEFT = 1
CAM_LABEL = {CAM_RIGHT: "RIGHT", CAM_LEFT: "LEFT"}

VICTIM_NONE = 0x00
VICTIM_PHI = 0x01
VICTIM_PSI = 0x02
VICTIM_OMEGA = 0x03

VICTIM_LABEL = {
    VICTIM_NONE: "N",
    VICTIM_PHI: "PHI",
    VICTIM_PSI: "PSI",
    VICTIM_OMEGA: "OMEGA",
}


class EspProtocolLink:
    def __init__(self, port: str, baudrate: int, timeout: float = 1.0):
        self.ser = Serial(port=port, baudrate=baudrate, timeout=timeout)
        self.timeout = timeout

        self.WAITING_FF = 0
        self.WAITING_AA = 1
        self.RECEIVE_LEN = 2
        self.RECEIVE_PKT = 3
        self.RECEIVE_CHECK = 4

        self.state_ = self.WAITING_FF
        self.msg_len_ = 0
        self.byte_cnt_ = 0
        self.cmd_byte_ = b""
        self.args_ = b""

    def _fsm(self, byte: bytes) -> bool:
        if self.state_ == self.WAITING_FF:
            if byte == b"\xff":
                self.state_ = self.WAITING_AA
                self.msg_len_ = 0
                self.byte_cnt_ = 0
                self.cmd_byte_ = b""
                self.args_ = b""

        elif self.state_ == self.WAITING_AA:
            self.state_ = self.RECEIVE_LEN if byte == b"\xaa" else self.WAITING_FF

        elif self.state_ == self.RECEIVE_LEN:
            self.msg_len_, = struct.unpack("B", byte)
            self.state_ = self.RECEIVE_PKT

        elif self.state_ == self.RECEIVE_PKT:
            if self.byte_cnt_ == 0:
                self.cmd_byte_ = byte
            else:
                self.args_ += byte
            self.byte_cnt_ += 1
            if self.byte_cnt_ >= self.msg_len_:
                self.state_ = self.RECEIVE_CHECK

        elif self.state_ == self.RECEIVE_CHECK:
            self.state_ = self.WAITING_FF
            return True

        return False

    def recv_packet(self, timeout: float = 1.5) -> bool:
        deadline = time.time() + timeout
        self.ser.timeout = timeout
        while time.time() < deadline:
            data = self.ser.read(1)
            if data and self._fsm(data):
                return True
        return False

    def send_detection(self, cam_id: int, victim_id: int):
        payload_len = 0x02
        checksum = (payload_len + cam_id + victim_id) & 0xFF
        pkt = struct.pack("6B", HEADER0, HEADER1, payload_len, cam_id, victim_id, checksum)
        self.ser.flushInput()
        self.ser.write(pkt)
        print(
            f"[Pi->ESP] {pkt.hex(' ').upper()} | "
            f"camera={CAM_LABEL.get(cam_id, cam_id)} victim={VICTIM_LABEL.get(victim_id, '?')}"
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


def resolve_port(cli_port: str | None) -> str:
    if cli_port:
        return cli_port

    if getattr(Constants, "serial_port", None):
        return Constants.serial_port

    preferred_patterns = [
        "/dev/cu.usbserial*",
        "/dev/cu.usbmodem*",
        "/dev/tty.usbserial*",
        "/dev/tty.usbmodem*",
        "/dev/ttyUSB*",
        "/dev/ttyACM*",
    ]

    for pattern in preferred_patterns:
        matches = sorted(glob.glob(pattern))
        if matches:
            return matches[0]

    raise RuntimeError("No serial port found. Pass it with --port.")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Automatic dual-camera detection responder for ESP requests"
    )
    parser.add_argument("--port", default=None, help="Serial port, e.g. /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=getattr(Constants, "baud_rate", 115200))
    parser.add_argument("--timeout", type=float, default=getattr(Constants, "timeout", 1.0))
    parser.add_argument("--right-device", type=int, default=1, help="Picamera2 index for RIGHT")
    parser.add_argument("--left-device", type=int, default=0, help="Picamera2 index for LEFT")
    parser.add_argument("--conf", type=float, default=0.6, help="YOLO confidence threshold")
    parser.add_argument("--imgsz", type=int, default=416, help="YOLO inference image size")
    return parser


def main():
    parser = build_arg_parser()
    args = parser.parse_args()

    base_dir = Path(__file__).resolve().parent
    model_path = base_dir / "Letters" / "weights" / "best.pt"
    if not model_path.exists():
        raise FileNotFoundError(f"Model not found: {model_path}")

    port = resolve_port(args.port)

    print(f"[INFO] Serial port: {port} @ {args.baud}")
    print(f"[INFO] RIGHT camera device={args.right_device} | LEFT camera device={args.left_device}")

    detector = None
    link = None
    try:
        detector = DualCameraDetector(
            model_path=model_path,
            right_device_index=args.right_device,
            left_device_index=args.left_device,
            conf_threshold=args.conf,
            img_size=args.imgsz,
        )
        link = EspProtocolLink(port=port, baudrate=args.baud, timeout=args.timeout)
        print("[INFO] Ready: waiting for ESP requests...")

        while True:
            if not link.recv_packet(timeout=1.5):
                continue

            cmd = link.cmd_byte_
            if cmd == bytes([CMD_REQUEST_RIGHT]):
                requested_cam = CAM_RIGHT
            elif cmd == bytes([CMD_REQUEST_LEFT]):
                requested_cam = CAM_LEFT
            else:
                continue

            victim_id = detector.detect_for_camera(requested_cam)
            link.send_detection(requested_cam, victim_id)

    except KeyboardInterrupt:
        print("\n[INFO] Stopping...")
    except SerialException as error:
        print(f"[ERROR] Serial error: {error}")
    finally:
        if detector is not None:
            detector.close()
        if link is not None:
            try:
                link.ser.close()
            except Exception:
                pass


if __name__ == "__main__":
    main()
