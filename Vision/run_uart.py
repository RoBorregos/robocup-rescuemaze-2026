#!/usr/bin/env python3

"""UART entrypoint for victim, target, or combined detection modes."""

from __future__ import annotations

import argparse
import time
from typing import Optional

from serial import Serial
from serial.serialutil import SerialException

import Constants
from detector import VisionDetector
from protocol import (
    PacketReader,
    VICTIM_NONE,
    build_victim_packet_for_name,
    camera_name,
    parse_detection_request,
    rebuild_packet_bytes,
    victim_name,
)
from service import Esp32Service
from target_detector import TargetDetector
from target_service import TargetEsp32Service


class CombinedEsp32Service:
    """Single UART service that tries victim first, then target detection."""

    def __init__(
        self,
        port=Constants.serial_port,
        baudrate=Constants.baud_rate,
        timeout=Constants.timeout,
    ):
        self.port_name = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.port: Optional[Serial] = None
        self.packet_reader = PacketReader()
        self.detector = VisionDetector()

    def connect(self) -> None:
        while True:
            try:
                print(f"Connecting to Microcontroller on port {self.port_name} ...")
                self.port = Serial(
                    port=self.port_name,
                    baudrate=self.baudrate,
                    timeout=self.timeout,
                    writeTimeout=self.timeout,
                )
                time.sleep(1)
                print("conectado")
                return
            except SerialException as exc:
                print(f"Serial Exception: {exc}")
                time.sleep(1)

    def close(self) -> None:
        if self.port is not None:
            self.port.close()
        self.detector.close()

    def send_vision_packet(self, camera_id: int, victim_id: int) -> int:
        if self.port is None:
            return 0
        packet = build_victim_packet_for_name(camera_id, victim_id)
        written = self.port.write(packet)
        self.port.flush()
        print(f"[TX] {packet.hex(' ').upper()}")
        return written

    def _detect_combined(self, camera_id: int) -> int:
        victim_id = self.detector.detect_victim(camera_id)
        if victim_id != VICTIM_NONE:
            return victim_id

        target_result = self.detector.detect_target(camera_id)
        return TargetDetector.victim_id_from_result(target_result)

    def listen_and_respond(self) -> None:
        if self.port is None:
            raise RuntimeError("Serial port not connected")

        print("\n=== Vision System - COMBINED Listener Mode ===")
        print(f"Victim model: {self.detector.model_path.name}")
        print("Flow: victim first, target fallback when victim is NONE")
        print("Waiting for ESP detection requests... (Ctrl+C to exit)\n")

        try:
            while True:
                byte_data = self.port.read(1)
                if not byte_data:
                    continue

                packet = self.packet_reader.feed(byte_data[0])
                if packet is None:
                    continue

                raw_packet = rebuild_packet_bytes(packet)
                print(f"[RX] {raw_packet.hex(' ').upper()}")
                print(
                    f"[REQ] len={packet.payload_len} payload={[int(b) for b in packet.payload]}"
                )

                is_request, camera_id = parse_detection_request(packet)
                if not is_request:
                    print("[REQ] Ignored packet (not detection request)")
                    continue

                victim_id = self._detect_combined(camera_id)
                self.send_vision_packet(camera_id, victim_id)
                print(
                    f"[SENT] CAM={camera_name(camera_id)} VICTIM={victim_name(victim_id)}"
                )
        except KeyboardInterrupt:
            print("\n[LISTENER] Stopped")


def build_service(mode: str):
    normalized_mode = mode.strip().lower()
    if normalized_mode == "victim":
        return Esp32Service()
    if normalized_mode == "target":
        return TargetEsp32Service()
    if normalized_mode == "both":
        return CombinedEsp32Service()
    raise ValueError(f"Unsupported mode: {mode}")


def main() -> None:
    parser = argparse.ArgumentParser(description="UART vision service runner")
    parser.add_argument(
        "--mode",
        default="both",
        choices=["victim", "target", "both"],
        help="Service mode: victim letters, target classes, or both (default)",
    )
    args = parser.parse_args()

    service = build_service(args.mode)
    print(f"[RUN] mode={args.mode}")
    service.connect()
    try:
        service.listen_and_respond()
    finally:
        service.close()


if __name__ == "__main__":
    main()
