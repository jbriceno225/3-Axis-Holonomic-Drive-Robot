"""Unit tests for newline buffering in the serial transport."""

from kiwi_serial_bridge.serial_transport import SerialTransport


class FakeSerial:
    """Minimal serial stand-in for line-buffer tests."""

    def __init__(self, data: bytes):
        self._data = data
        self.in_waiting = 0
        self.is_open = True

    def read(self, size):
        chunk = self._data[:size]
        self._data = self._data[size:]
        self.in_waiting = len(self._data)
        return chunk

    def write(self, data):
        return len(data)

    def flush(self):
        return None

    def close(self):
        self.is_open = False


def test_splits_complete_lines_and_keeps_a_partial():
    """Incomplete records stay buffered until the newline arrives."""
    transport = SerialTransport(
        port="/dev/null",
        baud_rate=115200,
        serial_device=FakeSerial(b""),
    )
    first = transport._take_lines(b"ODOM,0.0000,0.0000,0.0000,0,0,0\nODOM,1")
    second = transport._take_lines(b".0000,0.0000,0.0000,1,2,3\n")
    assert first == ["ODOM,0.0000,0.0000,0.0000,0,0,0"]
    assert second == ["ODOM,1.0000,0.0000,0.0000,1,2,3"]


def test_reads_when_in_waiting_is_zero():
    """CP2102 adapters may report no pending bytes until a timed read."""
    fake = FakeSerial(b"ACK,MODE,AUTO\nODOM,0.0000,0.0000,0.0000,0,0,0\n")
    fake.in_waiting = 0
    transport = SerialTransport(
        port="/dev/null",
        baud_rate=115200,
        serial_device=fake,
    )
    assert transport.read_available() == [
        "ACK,MODE,AUTO",
        "ODOM,0.0000,0.0000,0.0000,0,0,0",
    ]


def test_deasserts_modem_control_lines_on_open():
    """CP2102 control lines must not reset or hold the ESP32 at boot."""
    fake = FakeSerial(b"")
    fake.dtr = True
    fake.rts = True
    SerialTransport(
        port="/dev/null",
        baud_rate=115200,
        serial_device=fake,
    )
    assert fake.dtr is False
    assert fake.rts is False
