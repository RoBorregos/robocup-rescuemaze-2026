from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Optional, Tuple

HEADER0 = 0xFF
HEADER1 = 0xAA

CAM_RIGHT = 0
CAM_LEFT = 1

VICTIM_NONE = 0
VICTIM_PHI = 1
VICTIM_PSI = 2
VICTIM_OMEGA = 3


@dataclass
class Packet:
    payload_len: int
    payload: bytes


class PacketReader:
    WAIT_FF = 0
    WAIT_AA = 1
    WAIT_LEN = 2
    WAIT_PAYLOAD = 3
    WAIT_CHECK = 4

    def __init__(self) -> None:
        self.state = self.WAIT_FF
        self.payload_len = 0
        self.payload = bytearray()
        self.checksum = 0

    def feed(self, b: int) -> Optional[Packet]:
        if self.state == self.WAIT_FF:
            if b == HEADER0:
                self.state = self.WAIT_AA
            return None

        if self.state == self.WAIT_AA:
            if b == HEADER1:
                self.state = self.WAIT_LEN
            else:
                self.state = self.WAIT_FF
            return None

        if self.state == self.WAIT_LEN:
            self.payload_len = b
            self.payload = bytearray()
            self.checksum = b
            if self.payload_len <= 0 or self.payload_len > 32:
                self.state = self.WAIT_FF
            else:
                self.state = self.WAIT_PAYLOAD
            return None

        if self.state == self.WAIT_PAYLOAD:
            self.payload.append(b)
            self.checksum = (self.checksum + b) & 0xFF
            if len(self.payload) >= self.payload_len:
                self.state = self.WAIT_CHECK
            return None

        if self.state == self.WAIT_CHECK:
            self.state = self.WAIT_FF
            if b == self.checksum:
                return Packet(payload_len=self.payload_len, payload=bytes(self.payload))
            return None

        self.state = self.WAIT_FF
        return None


def victim_name(victim_id: int) -> str:
    return {
        VICTIM_NONE: "NONE",
        VICTIM_PHI: "PHI",
        VICTIM_PSI: "PSI",
        VICTIM_OMEGA: "OMEGA",
    }.get(victim_id, "UNKNOWN")


def camera_name(camera_id: int) -> str:
    return "RIGHT" if camera_id == CAM_RIGHT else "LEFT"


def build_vision_packet(camera_id: int, victim_id: int) -> bytes:
    payload_len = 2
    checksum = (payload_len + camera_id + victim_id) & 0xFF
    return struct.pack("BBBBBB", HEADER0, HEADER1, payload_len, camera_id, victim_id, checksum)


def parse_detection_request(packet: Packet) -> Tuple[bool, int]:
    if packet.payload_len == 1 and packet.payload[0] == 0x01:
        return True, CAM_RIGHT
    if packet.payload_len == 2 and packet.payload[0] == 0x01 and packet.payload[1] in (CAM_RIGHT, CAM_LEFT):
        return True, int(packet.payload[1])
    return False, CAM_RIGHT
