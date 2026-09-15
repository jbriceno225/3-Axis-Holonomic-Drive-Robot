"""Thread-safe serial transport for newline-delimited protocol messages."""

import threading
import time
from typing import Any, List, Optional


class SerialTransport:
    """Own the serial port and serialize all access to it."""

    def __init__(
        self,
        port: str,
        baud_rate: int,
        read_timeout: float = 0.02,
        write_timeout: float = 0.1,
        startup_delay: float = 2.0,
        serial_device: Optional[Any] = None,
    ) -> None:
        self._lock = threading.Lock()
        self._rx_buffer = bytearray()
        if serial_device is not None:
            self._serial = serial_device
        else:
            import serial

            self._serial = serial.Serial(
                port=port,
                baudrate=baud_rate,
                timeout=read_timeout,
                write_timeout=write_timeout,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
            )

        # CP2102 modem-control defaults can reset or stall ESP32 boards.
        # This transport uses no modem flow control, so keep both lines low.
        self._serial.dtr = False
        self._serial.rts = False

        if serial_device is None:
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
        """Read buffered bytes and return every complete protocol line."""
        with self._lock:
            waiting = self._serial.in_waiting
            if waiting > 0:
                chunk = self._serial.read(waiting)
            else:
                # Some CP2102 drivers report in_waiting=0 while bytes are
                # pending. A timed read(1) still consumes them.
                chunk = self._serial.read(1)
                extra = self._serial.in_waiting
                if extra > 0:
                    chunk += self._serial.read(extra)
            return self._take_lines(chunk)

    def _take_lines(self, chunk: bytes) -> List[str]:
        """Append bytes and split out complete newline-terminated records."""
        if chunk:
            self._rx_buffer.extend(chunk)
        lines: List[str] = []
        while True:
            newline = self._rx_buffer.find(b"\n")
            if newline < 0:
                break
            raw = bytes(self._rx_buffer[:newline])
            del self._rx_buffer[: newline + 1]
            line = raw.decode("utf-8", errors="replace").strip("\r").strip()
            if line:
                lines.append(line)
        return lines

    def close(self) -> None:
        """Close the serial port."""
        with self._lock:
            if self._serial.is_open:
                self._serial.close()
