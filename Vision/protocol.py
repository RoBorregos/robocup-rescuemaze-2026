"""Binary UART protocol definitions and helpers for Vision.

Contains packet constants, parser state machine, name utilities,
and packet build/parse helper functions.
"""

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
VICTIM_HARMED = 4
VICTIM_UNHARMED = 5
VICTIM_STABLE = 6
VICTIM_FAKE_TARGET = 7


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
        VICTIM_HARMED: "HARMED",
        VICTIM_UNHARMED: "UNHARMED",
        VICTIM_STABLE: "STABLE",
        VICTIM_FAKE_TARGET: "FAKE_TARGET",
    }.get(victim_id, "UNKNOWN")


def camera_name(camera_id: int) -> str:
    return "RIGHT" if camera_id == CAM_RIGHT else "LEFT"


def build_vision_packet(camera_id: int, victim_id: int) -> bytes:
    payload_len = 2
    checksum = (payload_len + camera_id + victim_id) & 0xFF
    return struct.pack(
        "BBBBBB", HEADER0, HEADER1, payload_len, camera_id, victim_id, checksum
    )


def build_victim_packet(camera_id: int, victim_id: int) -> bytes:
    return build_vision_packet(camera_id, victim_id)


def build_none_packet(camera_id: int) -> bytes:
    return build_victim_packet(camera_id, VICTIM_NONE)


def build_phi_packet(camera_id: int) -> bytes:
    return build_victim_packet(camera_id, VICTIM_PHI)


def build_psi_packet(camera_id: int) -> bytes:
    return build_victim_packet(camera_id, VICTIM_PSI)


def build_omega_packet(camera_id: int) -> bytes:
    return build_victim_packet(camera_id, VICTIM_OMEGA)


def build_harmed_packet(camera_id: int) -> bytes:
    return build_victim_packet(camera_id, VICTIM_HARMED)


def build_unharmed_packet(camera_id: int) -> bytes:
    return build_victim_packet(camera_id, VICTIM_UNHARMED)


def build_stable_packet(camera_id: int) -> bytes:
    return build_victim_packet(camera_id, VICTIM_STABLE)


def build_fake_target_packet(camera_id: int) -> bytes:
    return build_victim_packet(camera_id, VICTIM_FAKE_TARGET)


def build_victim_packet_for_name(camera_id: int, victim_id: int) -> bytes:
    builders = {
        VICTIM_NONE: build_none_packet,
        VICTIM_PHI: build_phi_packet,
        VICTIM_PSI: build_psi_packet,
        VICTIM_OMEGA: build_omega_packet,
        VICTIM_HARMED: build_harmed_packet,
        VICTIM_UNHARMED: build_unharmed_packet,
        VICTIM_STABLE: build_stable_packet,
        VICTIM_FAKE_TARGET: build_fake_target_packet,
    }
    return builders.get(victim_id, build_none_packet)(camera_id)


def rebuild_packet_bytes(packet: Packet) -> bytes:
    checksum = (packet.payload_len + sum(packet.payload)) & 0xFF
    return bytes([HEADER0, HEADER1, packet.payload_len, *packet.payload, checksum])


def parse_detection_request(packet: Packet) -> Tuple[bool, int]:
    if packet.payload_len == 1 and packet.payload[0] == 0x01:
        return True, CAM_RIGHT
    if (
        packet.payload_len == 2
        and packet.payload[0] == 0x01
        and packet.payload[1] in (CAM_RIGHT, CAM_LEFT)
    ):
        return True, int(packet.payload[1])
    return False, CAM_RIGHT
