"""UART service layer between Raspberry Pi vision and ESP32.

Reads request packets, calls the detector, and sends response packets
using the protocol helpers.
"""

from __future__ import annotations

import time
from typing import Optional

from serial import Serial
from serial.serialutil import SerialException

import Constants
from detector import VisionDetector
from protocol import (
    PacketReader,
    build_vision_packet,
    camera_name,
    parse_detection_request,
    rebuild_packet_bytes,
    victim_name,
)


class Esp32Service:
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
        packet = build_vision_packet(camera_id, victim_id)
        written = self.port.write(packet)
        self.port.flush()
        print(f"[TX] {packet.hex(' ').upper()}")
        return written

    def listen_and_respond(self) -> None:
        if self.port is None:
            raise RuntimeError("Serial port not connected")

        print("\n=== Vision System - YOLO Listener Mode ===")
        print(f"Model: {self.detector.model_path.name}")
        print(
            f"Cameras: RIGHT={self.detector.cam_right_idx}, LEFT={self.detector.cam_left_idx}"
        )
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

                victim_id = self.detector.detect_victim(camera_id)
                self.send_vision_packet(camera_id, victim_id)
                print(
                    f"[SENT] CAM={camera_name(camera_id)} VICTIM={victim_name(victim_id)}"
                )
        except KeyboardInterrupt:
            print("\n[LISTENER] Stopped")
