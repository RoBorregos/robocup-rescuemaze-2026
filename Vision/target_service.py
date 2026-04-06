"""UART service for bullseye target detection.

The ESP keeps sending the same request packet; this service answers with
explicit target IDs: UNHARMED, STABLE, HARMED, FAKE_TARGET, or NONE.
"""

from __future__ import annotations

import time
from typing import Optional

from serial import Serial
from serial.serialutil import SerialException

import Constants
from detector import VisionDetector
from protocol import (
    VICTIM_FAKE_TARGET,
    VICTIM_HARMED,
    VICTIM_STABLE,
    VICTIM_UNHARMED,
    build_fake_target_packet,
    build_harmed_packet,
    build_none_packet,
    PacketReader,
    build_stable_packet,
    build_unharmed_packet,
    camera_name,
    parse_detection_request,
    rebuild_packet_bytes,
    victim_name,
)
from target_detector import TargetDetector


class TargetEsp32Service:
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
        self.camera_detector = VisionDetector()
        self.detector = None

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
        if self.camera_detector is not None:
            self.camera_detector.close()
        self.detector = None

    def send_vision_packet(self, camera_id: int, victim_id: int) -> int:
        if self.port is None:
            return 0
        packet_builders = {
            0: build_none_packet,
            VICTIM_UNHARMED: build_unharmed_packet,
            VICTIM_STABLE: build_stable_packet,
            VICTIM_HARMED: build_harmed_packet,
            VICTIM_FAKE_TARGET: build_fake_target_packet,
        }
        packet = packet_builders.get(victim_id, build_none_packet)(camera_id)
        written = self.port.write(packet)
        self.port.flush()
        print(f"[TX] {packet.hex(' ').upper()}")
        return written

    def listen_and_respond(self) -> None:
        if self.port is None:
            raise RuntimeError("Serial port not connected")

        print("\n=== Vision System - TARGET Listener Mode ===")
        print(f"Model: {self.camera_detector.model_path.name}")
        print("Sending UNHARMED/STABLE/HARMED/FAKE_TARGET (or NONE if unknown)")
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

                result = self.camera_detector.detect_target(camera_id)
                victim_id = TargetDetector.victim_id_from_result(result)
                self.send_vision_packet(camera_id, victim_id)
                print(
                    f"[SENT] CAM={camera_name(camera_id)} VICTIM={victim_name(victim_id)}"
                )
        except KeyboardInterrupt:
            print("\n[LISTENER] Stopped")


def main() -> None:
    service = TargetEsp32Service()
    service.connect()
    try:
        service.listen_and_respond()
    finally:
        service.close()


if __name__ == "__main__":
    main()
