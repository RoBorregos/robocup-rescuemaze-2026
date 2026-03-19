#!/usr/bin/env python3

"""Backward-compatible wrapper for legacy entrypoint usage.

Keeps support for running the old script name while delegating execution
to the current UART runner.
"""

from run_uart import main
from service import Esp32Service as Esp32

if __name__ == "__main__":
    main()
