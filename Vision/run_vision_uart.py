#!/usr/bin/env python3

"""Entrypoint script for running the UART vision service.

Creates the ESP32 service, opens serial communication, and keeps
the request/response loop alive.
"""

from vision_service import Esp32Service


def main() -> None:
    service = Esp32Service()
    service.connect()
    try:
        service.listen_and_respond()
    finally:
        service.close()


if __name__ == "__main__":
    main()
