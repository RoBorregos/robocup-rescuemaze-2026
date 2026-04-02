"""Entry point for the existing victim-detection UART service."""

from service import Esp32Service


def main() -> None:
    service = Esp32Service()
    service.connect()
    try:
        service.listen_and_respond()
    finally:
        service.close()


if __name__ == "__main__":
    main()