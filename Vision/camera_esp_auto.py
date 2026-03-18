#!/usr/bin/env python3
import argparse
import glob
import sys
import threading
import time
from pathlib import Path
from queue import Empty, Queue

from serial.serialutil import SerialException

import Constants
from dual_camera_detector import DualCameraDetector
from esp_protocol_link import CAM_LEFT, CAM_RIGHT, CMD_REQUEST_LEFT, CMD_REQUEST_RIGHT, CAM_LABEL, EspProtocolLink

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
    pending_requests = Queue()
    
    def detect_worker():
        """Background thread: process detection requests in parallel."""
        while True:
            try:
                requested_cam = pending_requests.get(timeout=0.5)
                if requested_cam is None:
                    break
                victim_id = detector.detect_for_camera(requested_cam)
                link.send_detection(requested_cam, victim_id)
            except Empty:
                continue
            except Exception as e:
                print(f"[WORKER] Error in detection: {e}", file=sys.stderr)
                continue
    
    try:
        detector = DualCameraDetector(
            model_path=model_path,
            right_device_index=args.right_device,
            left_device_index=args.left_device,
            conf_threshold=args.conf,
            img_size=args.imgsz,
        )
        link = EspProtocolLink(port=port, baudrate=args.baud, timeout=args.timeout)
        
        # Start detection worker thread
        worker_thread = threading.Thread(target=detect_worker, daemon=True)
        worker_thread.start()
        
        print("[INFO] Ready: waiting for ESP requests...")

        while True:
            try:
                if not link.recv_packet(timeout=1.5):
                    continue

                cmd = link.cmd_byte_
                if cmd == bytes([CMD_REQUEST_RIGHT]):
                    requested_cam = CAM_RIGHT
                elif cmd == bytes([CMD_REQUEST_LEFT]):
                    requested_cam = CAM_LEFT
                else:
                    continue

                # Queue detection request instead of blocking
                pending_requests.put(requested_cam)
            except SerialException as serial_error:
                print(f"[RECV] Serial error: {serial_error}", file=sys.stderr)
                time.sleep(0.5)
                continue
            except Exception as e:
                print(f"[RECV] Unexpected error: {e}", file=sys.stderr)
                time.sleep(0.5)
                continue

    except KeyboardInterrupt:
        print("\n[INFO] Stopping...")
        pending_requests.put(None)
    except SerialException as error:
        print(f"[ERROR] Failed to open serial port: {error}")
        sys.exit(1)
    except FileNotFoundError as error:
        print(f"[ERROR] {error}")
        sys.exit(1)
    except Exception as error:
        print(f"[ERROR] Initialization failed: {error}")
        sys.exit(1)
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
