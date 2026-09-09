"""Unit tests for the serial protocol."""

import pytest

from kiwi_serial_bridge.serial_protocol import (
    AckMessage,
    ErrMessage,
    ModeCommand,
    OdomMessage,
    ProtocolMessage,
    StopCommand,
    VelocityCommand,
    WarnMessage,
    format_message,
    parse_message,
)


@pytest.mark.parametrize(
    ("message", "wire"),
    [
        (VelocityCommand(0.5, -0.25, 1.0), "V,0.5000,-0.2500,1.0000"),
        (ModeCommand("AUTO"), "MODE,AUTO"),
        (StopCommand(), "STOP"),
        (AckMessage("MODE,AUTO"), "ACK,MODE,AUTO"),
        (WarnMessage("low voltage"), "WARN,low voltage"),
        (ErrMessage("bad command"), "ERR,bad command"),
        (
            OdomMessage(1.0, 2.0, 3.0, 4, 5, 6),
            "ODOM,1.0000,2.0000,3.0000,4,5,6",
        ),
    ],
)
def test_format_and_parse_round_trip(
    message: ProtocolMessage,
    wire: str,
) -> None:
    """Every supported message retains its values across formatting."""
    assert format_message(message) == wire
    assert parse_message(wire) == message


@pytest.mark.parametrize(
    "line",
    ["", "ODOM,1,2", "ODOM,a,2,3,4,5,6", "MODE", "UNKNOWN,1"],
)
def test_rejects_malformed_messages(line: str) -> None:
    """Malformed and unknown protocol lines are rejected."""
    with pytest.raises(ValueError):
        parse_message(line)
