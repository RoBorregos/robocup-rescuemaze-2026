#!/usr/bin/env python3

"""Interactive simulator for Raspberry detection requests.

Type `0` or `1` in terminal to simulate a detection request for RIGHT/LEFT camera.
This script does not require serial hardware; it reuses the same protocol parsing and
response packet building used by UART services.
"""

from __future__ import annotations

import cv2

from detector import VisionDetector
from protocol import (
    CAM_LEFT,
    CAM_RIGHT,
    PacketReader,
    build_victim_packet_for_name,
    camera_name,
    parse_detection_request,
    rebuild_packet_bytes,
    victim_name,
)


def build_detection_request(camera_id: int) -> bytes:
    payload = bytes([0x01, camera_id])
    payload_len = len(payload)
    checksum = (payload_len + sum(payload)) & 0xFF
    return bytes([0xFF, 0xAA, payload_len, *payload, checksum])


def detect_with_best_available(detector: VisionDetector, camera_id: int) -> int:
    combined_method = getattr(detector, "detect_combined", None)
    if callable(combined_method):
        return int(combined_method(camera_id))
    return int(detector.detect_victim(camera_id))


def draw_model_detections(detector: VisionDetector, frame):
    rendered = frame.copy()
    target_found = False
    best_target = None

    run_model = getattr(detector, "_run_model", None)
    if callable(run_model):
        result = run_model(frame)
    else:
        predictions = detector.model.predict(
            frame,
            conf=detector.conf,
            iou=detector.iou,
            imgsz=detector.imgsz,
            device=detector.device,
            verbose=False,
        )
        result = predictions[0] if predictions else None

    if result is None or result.boxes is None or len(result.boxes) == 0:
        return rendered, target_found

    names = result.names
    is_target_class = getattr(detector, "_is_target_class", None)

    for index in range(len(result.boxes)):
        conf = float(result.boxes.conf[index].item())
        class_id = int(result.boxes.cls[index].item())
        class_name = (
            names.get(class_id, str(class_id)) if isinstance(names, dict) else str(class_id)
        )
        x1, y1, x2, y2 = map(int, result.boxes.xyxy[index].tolist())

        if callable(is_target_class) and is_target_class(class_name):
            color = (0, 140, 255)
            target_found = True
            if best_target is None or conf > best_target["conf"]:
                best_target = {
                    "bbox": (x1, y1, x2, y2),
                    "class_name": class_name,
                    "conf": conf,
                }
        else:
            color = (255, 180, 0)

        cv2.rectangle(rendered, (x1, y1), (x2, y2), color, 2)
        text = f"{class_name} {conf:.2f}"
        text_y = y1 - 8 if y1 > 18 else y1 + 16
        cv2.putText(
            rendered,
            text,
            (x1, text_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 0),
            3,
        )
        cv2.putText(
            rendered,
            text,
            (x1, text_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            1,
        )

    return rendered, target_found, best_target


def get_target_ring_colors(detector: VisionDetector, frame, camera_id: int, target_bbox):
    crop_fn = getattr(detector, "_crop_bbox", None)
    pad_ratio = float(getattr(detector, "target_crop_pad_ratio", 0.12))

    if callable(crop_fn):
        roi, _ = crop_fn(frame, target_bbox, pad_ratio)
    else:
        h_img, w_img = frame.shape[:2]
        x1, y1, x2, y2 = target_bbox
        box_w = max(1, x2 - x1)
        box_h = max(1, y2 - y1)
        pad = int(min(box_w, box_h) * max(0.0, pad_ratio))
        rx1 = max(0, x1 - pad)
        ry1 = max(0, y1 - pad)
        rx2 = min(w_img, x2 + pad)
        ry2 = min(h_img, y2 + pad)
        roi = frame[ry1:ry2, rx1:rx2]

    if roi is None or roi.size == 0:
        return None

    try:
        from target_detector import detect_bullseye_frame

        analysis = detect_bullseye_frame(
            roi,
            model=None,
            yolo_conf=detector.conf,
            label_history=None,
            camera_id=camera_id,
        )
        if not analysis:
            return None

        detected = analysis.get("detected")
        if isinstance(detected, list) and detected:
            return detected

        rings = analysis.get("rings")
        if isinstance(rings, list):
            colors = [ring.get("color", "unknown") for ring in rings if isinstance(ring, dict)]
            return colors if colors else None
        return None
    except Exception as exc:
        print(f"[SIM PREVIEW] target ring analysis failed: {exc}")
        return None


def show_camera_detection_preview(detector: VisionDetector, camera_id: int, victim_id: int) -> None:
    ok, frame = detector.read_frame(camera_id)
    if not ok or frame is None:
        print(f"[SIM PREVIEW] No frame for {camera_name(camera_id)}")
        return

    raw_frame = frame.copy()
    frame, target_found, best_target = draw_model_detections(detector, frame)

    label = f"{camera_name(camera_id)} -> {victim_name(victim_id)}"
    cv2.putText(
        frame,
        label,
        (12, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 0, 0),
        3,
    )
    cv2.putText(
        frame,
        label,
        (12, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 255),
        2,
    )

    if target_found:
        cv2.putText(
            frame,
            "TARGET DETECTED",
            (12, 58),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 0),
            3,
        )
        cv2.putText(
            frame,
            "TARGET DETECTED",
            (12, 58),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 140, 255),
            2,
        )

        if best_target is not None:
            ring_colors = get_target_ring_colors(
                detector,
                raw_frame,
                camera_id,
                best_target["bbox"],
            )
            if ring_colors:
                ring_text = "rings: " + "-".join(ring_colors)
                cv2.putText(
                    frame,
                    ring_text,
                    (12, 84),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.58,
                    (0, 0, 0),
                    3,
                )
                cv2.putText(
                    frame,
                    ring_text,
                    (12, 84),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.58,
                    (0, 255, 255),
                    1,
                )
                print(f"[SIM TARGET] {camera_name(camera_id)} rings={ring_colors}")

    cv2.imshow(f"SIM {camera_name(camera_id)}", frame)
    cv2.waitKey(1)


def main() -> None:
    reader = PacketReader()
    detector = VisionDetector()

    print("\n=== Simulador Rasp Get Detection ===")
    print("Escribe 0 (RIGHT), 1 (LEFT), both, o q para salir.\n")

    try:
        while True:
            cmd = input("> request camera [0/1/both/q]: ").strip().lower()
            if cmd in {"q", "quit", "exit"}:
                break

            if cmd == "both":
                camera_ids = [CAM_RIGHT, CAM_LEFT]
            elif cmd == "0":
                camera_ids = [CAM_RIGHT]
            elif cmd == "1":
                camera_ids = [CAM_LEFT]
            else:
                print("Entrada inválida. Usa 0, 1, both o q.")
                continue

            for camera_id in camera_ids:
                request = build_detection_request(camera_id)
                print(f"[SIM RX RAW] {request.hex(' ').upper()}")

                parsed_packet = None
                for byte in request:
                    parsed_packet = reader.feed(byte)
                if parsed_packet is None:
                    print("[SIM] No se pudo parsear request")
                    continue

                rebuilt = rebuild_packet_bytes(parsed_packet)
                is_request, req_camera_id = parse_detection_request(parsed_packet)
                print(f"[SIM RX] {rebuilt.hex(' ').upper()}")

                if not is_request:
                    print("[SIM] Paquete no reconocido como request de detección")
                    continue

                victim_id = detect_with_best_available(detector, req_camera_id)
                tx_packet = build_victim_packet_for_name(req_camera_id, victim_id)

                print(
                    f"[SIM TX] {tx_packet.hex(' ').upper()} | CAM={camera_name(req_camera_id)} "
                    f"VICTIM={victim_name(victim_id)}"
                )
                show_camera_detection_preview(detector, req_camera_id, victim_id)

    finally:
        cv2.destroyAllWindows()
        detector.close()
        print("\n[SIM] Finalizado")


if __name__ == "__main__":
    main()
