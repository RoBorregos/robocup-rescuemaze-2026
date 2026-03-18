import struct
import time

from serial import Serial

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
