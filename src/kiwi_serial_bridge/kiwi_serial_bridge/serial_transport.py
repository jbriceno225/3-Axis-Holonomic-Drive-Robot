"""Thread-safe serial transport for newline-delimited protocol messages."""

import threading
import time
from typing import List

import serial


class SerialTransport:
    """Own the serial port and serialize all access to it."""

    def __init__(
        self,
        port: str,
        baud_rate: int,
        read_timeout: float = 0.02,
        write_timeout: float = 0.1,
        startup_delay: float = 2.0,
    ) -> None:
        self._lock = threading.Lock()
        self._serial = serial.Serial(
            port=port,
            baudrate=baud_rate,
            timeout=read_timeout,
            write_timeout=write_timeout,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
        time.sleep(startup_delay)
        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()

    def write_line(self, line: str) -> None:
        """Write and flush one ASCII protocol line."""
        data = f"{line}\n".encode("ascii")
        with self._lock:
            self._serial.write(data)
            self._serial.flush()

    def read_available(self) -> List[str]:
        """Read all currently buffered lines and decode them safely."""
        lines: List[str] = []
        while self._serial.in_waiting > 0:
            with self._lock:
                raw_line = self._serial.readline()
            line = raw_line.decode("utf-8", errors="replace").strip()
            if line:
                lines.append(line)
        return lines

    def close(self) -> None:
        """Close the serial port."""
        with self._lock:
            if self._serial.is_open:
                self._serial.close()
