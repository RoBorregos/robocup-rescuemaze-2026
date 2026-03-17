#!/usr/bin/env python3
"""
Test de comunicación ESP32 ↔ Raspberry Pi / PC

Uso:
    python test_camera_esp.py [puerto]
    Ejemplo: python test_camera_esp.py /dev/ttyUSB0

Desde la terminal escribe:
    <camara> <victima>
    Ejemplos:
        0 PHI   →  cámara derecha,   víctima PHI   Φ (Heated)
        1 PSI   →  cámara izquierda, víctima PSI   Ψ (Stable)
        0 OMEGA →  cámara derecha,   víctima OMEGA Ω (Unheated)
        1 N     →  cámara izquierda, víctima N       (ninguna)

Cámaras:
    0 = derecha  (right)
    1 = izquierda (left)

Víctimas soportadas:
    PHI   → Heated   Φ (0x01)
    PSI   → Stable   Ψ (0x02)
    OMEGA → Unheated Ω (0x03)
    N     → None       (0x00)

Protocolo del paquete enviado a la ESP:
    [0xFF] [0xAA] [len=0x02] [cam_id] [victim_id] [checksum]
    checksum = (len + cam_id + victim_id) & 0xFF
"""

import struct
import time
import sys
import _thread
import glob
from serial import Serial
from serial.serialutil import SerialException

# ── Configuración ─────────────────────────────────────────────────────────────
BAUDRATE = 115200
TIMEOUT  = 1.0


def resolve_port() -> str:
    if len(sys.argv) > 1:
        return sys.argv[1]

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

    raise RuntimeError(
        "No serial port found automatically. "
        "Pass one manually, e.g. python Detection/test_camera_esp.py /dev/cu.usbserial-1140"
    )

# ── Protocolo ─────────────────────────────────────────────────────────────────
HEADER0 = 0xFF
HEADER1 = 0xAA

# Comandos que manda la ESP para pedir detección
CMD_REQUEST_RIGHT = 0x02
CMD_REQUEST_LEFT  = 0x03

# IDs de víctimas
VICTIM_MAP = {
    "PHI":   0x01,   # Heated   Φ
    "PSI":   0x02,   # Stable   Ψ
    "OMEGA": 0x03,   # Unheated Ω
    "N":     0x00,   # None
}

# IDs de cámara
CAM_MAP = {
    "0": 0,   # right 
    "1": 1,   # left 
}

CAM_LABEL = {0: "RIGHT", 1: "LEFT"}


def build_packet(cam_id: int, victim_id: int) -> bytes:
    """
    [0xFF][0xAA][len=0x02][cam_id][victim_id][checksum]
    """
    payload_len = 0x02
    checksum    = (payload_len + cam_id + victim_id) & 0xFF
    return struct.pack("6B", HEADER0, HEADER1, payload_len, cam_id, victim_id, checksum)


class EspTester:
    def __init__(self, port: str, baudrate: int = BAUDRATE):
        print(f"Conectando a {port} @ {baudrate} bauds...")
        self.ser = Serial(port=port, baudrate=baudrate, timeout=TIMEOUT)
        time.sleep(1)
        print("¡Conectado!\n")

        # FSM for recibe packs to ESP
        self.WAITING_FF    = 0
        self.WAITING_AA    = 1
        self.RECEIVE_LEN   = 2
        self.RECEIVE_PKT   = 3
        self.RECEIVE_CHECK = 4
        self.state_    = self.WAITING_FF
        self.msg_len_  = 0
        self.byte_cnt_ = 0
        self.cmd_byte_ = b''
        self.args_     = b''

        self.mutex = _thread.allocate_lock()

        # Valores pendientes seteados desde el teclado
        self.pending_cam_id    = None
        self.pending_victim_id = None
        self.last_waiting_notice = 0.0

    #  Reception FSM  
    def _fsm(self, byte: bytes) -> bool:
        if self.state_ == self.WAITING_FF:
            if byte == b'\xff':
                self.state_    = self.WAITING_AA
                self.msg_len_  = 0
                self.byte_cnt_ = 0
                self.cmd_byte_ = b''
                self.args_     = b''

        elif self.state_ == self.WAITING_AA:
            self.state_ = self.RECEIVE_LEN if byte == b'\xaa' else self.WAITING_FF

        elif self.state_ == self.RECEIVE_LEN:
            self.msg_len_, = struct.unpack("B", byte)
            self.state_    = self.RECEIVE_PKT

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
            b = self.ser.read(1)
            if b and self._fsm(b):
                return True
        return False

    def send_packet(self, cam_id: int, victim_id: int):
        pkt = build_packet(cam_id, victim_id)
        with self.mutex:
            self.ser.flushInput()
            self.ser.write(pkt)
        victim_label = next((k for k, v in VICTIM_MAP.items() if v == victim_id), "?")
        print(f"  [Pi->ESP] paquete: {pkt.hex(' ').upper()}")
        print(f"            camara={cam_id} ({CAM_LABEL[cam_id]}) | victima={victim_id} ({victim_label})\n")

    # ── Hilo de teclado ───────────────────────────────────────────────────────
    def _keyboard_thread(self):
        print("=" * 52)
        print("Escribe: <camara> <victima>  y presiona Enter")
        print("  Camaras : 0=derecha  1=izquierda")
        print("  Victimas: PHI=Heated(Φ)  PSI=Stable(Ψ)  OMEGA=Unheated(Ω)  N=None")
        print("  Ejemplo : '0 PHI'  o  '1 OMEGA'")
        print("=" * 52 + "\n")

        while True:
            try:
                line = input().strip().upper().split()
            except EOFError:
                break

            if len(line) != 2:
                print("  [!] Formato incorrecto. Ejemplo: '0 phi' o '1 omega'\n")
                continue

            cam_str, victim_str = line

            if cam_str not in CAM_MAP:
                print(f"  [!] Camara invalida '{cam_str}'. Usa 0 o 1\n")
                continue

            if victim_str not in VICTIM_MAP:
                print(f"  [!] Victima invalida '{victim_str}'. Opciones: {list(VICTIM_MAP.keys())}\n")
                continue

            cam_id    = CAM_MAP[cam_str]
            victim_id = VICTIM_MAP[victim_str]

            print(f"  [OK] Encolado -> camara={cam_id} ({CAM_LABEL[cam_id]}) | victima={victim_str} (0x{victim_id:02X})")
            print("       Se enviara cuando la ESP pida esa camara.")
            self.pending_cam_id    = cam_id
            self.pending_victim_id = victim_id

    # ── Bucle principal 
    def run(self):
        _thread.start_new_thread(self._keyboard_thread, ())
        print("Esperando peticiones de la ESP... (modo manual, sin autoenvio)\n")

        while True:
            try:
                # ── Esperar petición de la ESP ────────────────────────────────
                ok = self.recv_packet(timeout=1.5)
                if not ok:
                    continue

                cmd = self.cmd_byte_

                if cmd == bytes([CMD_REQUEST_RIGHT]):
                    requested_cam = 0
                elif cmd == bytes([CMD_REQUEST_LEFT]):
                    requested_cam = 1
                else:
                    continue

                if self.pending_cam_id is None or self.pending_victim_id is None:
                    now = time.time()
                    if now - self.last_waiting_notice >= 2.0:
                        # print("  [MANUAL] Esperando que escribas: <camara> <victima>")
                        self.last_waiting_notice = now
                    continue

                if self.pending_cam_id != requested_cam:
                    print(
                        f"  [MANUAL] Encolado para camara={self.pending_cam_id} "
                        f"({CAM_LABEL[self.pending_cam_id]}), esperando la peticion correcta.\n"
                    )
                    continue

                victim_id = self.pending_victim_id
                print(
                    f"[ESP->Pi] Peticion recibida para camara={requested_cam} "
                    f"({CAM_LABEL[requested_cam]})"
                )
                self.send_packet(requested_cam, victim_id)
                self.pending_cam_id = None
                self.pending_victim_id = None

            except KeyboardInterrupt:
                print("\nCerrando conexion...")
                self.ser.close()
                sys.exit(0)
            except SerialException as e:
                print(f"[error serial] {e} - reintentando en 2 s...")
                time.sleep(2)


# ── Entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    PORT = resolve_port()
    tester = EspTester(PORT, BAUDRATE)
    tester.run()
