"""Typed messages for the ESP32 line-oriented serial protocol."""

from dataclasses import dataclass
from typing import Union


@dataclass(frozen=True)
class VelocityCommand:
    """Normalized robot velocity command."""

    x: float
    y: float
    rotation: float


@dataclass(frozen=True)
class ModeCommand:
    """Controller operating-mode command or report."""

    mode: str


@dataclass(frozen=True)
class StopCommand:
    """Immediate stop command."""


@dataclass(frozen=True)
class AckMessage:
    """Controller acknowledgement."""

    detail: str


@dataclass(frozen=True)
class WarnMessage:
    """Controller warning."""

    detail: str


@dataclass(frozen=True)
class ErrMessage:
    """Controller error."""

    detail: str


@dataclass(frozen=True)
class OdomMessage:
    """Measured wheel speeds and encoder counts."""

    rpm1: float
    rpm2: float
    rpm3: float
    count1: int
    count2: int
    count3: int


ProtocolMessage = Union[
    VelocityCommand,
    ModeCommand,
    StopCommand,
    AckMessage,
    WarnMessage,
    ErrMessage,
    OdomMessage,
]


def format_message(message: ProtocolMessage) -> str:
    """Format a protocol message without its trailing newline."""
    if isinstance(message, VelocityCommand):
        return (
            f"V,{message.x:.4f},{message.y:.4f},"
            f"{message.rotation:.4f}"
        )
    if isinstance(message, ModeCommand):
        return f"MODE,{message.mode}"
    if isinstance(message, StopCommand):
        return "STOP"
    if isinstance(message, AckMessage):
        return _format_detail("ACK", message.detail)
    if isinstance(message, WarnMessage):
        return _format_detail("WARN", message.detail)
    if isinstance(message, ErrMessage):
        return _format_detail("ERR", message.detail)
    if isinstance(message, OdomMessage):
        return (
            f"ODOM,{message.rpm1:.4f},{message.rpm2:.4f},"
            f"{message.rpm3:.4f},{message.count1},"
            f"{message.count2},{message.count3}"
        )
    raise TypeError(f"Unsupported protocol message: {type(message)!r}")


def parse_message(line: str) -> ProtocolMessage:
    """Parse one protocol line, raising ValueError when malformed."""
    fields = line.strip().split(",")
    kind = fields[0] if fields else ""

    if kind == "V" and len(fields) == 4:
        return VelocityCommand(
            x=float(fields[1]),
            y=float(fields[2]),
            rotation=float(fields[3]),
        )
    if kind == "MODE" and len(fields) == 2 and fields[1]:
        return ModeCommand(fields[1])
    if kind == "STOP" and len(fields) == 1:
        return StopCommand()
    if kind == "ACK" and len(fields) >= 1:
        return AckMessage(",".join(fields[1:]))
    if kind == "WARN" and len(fields) >= 1:
        return WarnMessage(",".join(fields[1:]))
    if kind == "ERR" and len(fields) >= 1:
        return ErrMessage(",".join(fields[1:]))
    if kind == "ODOM" and len(fields) == 7:
        return OdomMessage(
            rpm1=float(fields[1]),
            rpm2=float(fields[2]),
            rpm3=float(fields[3]),
            count1=int(fields[4]),
            count2=int(fields[5]),
            count3=int(fields[6]),
        )
    raise ValueError(f"Unknown or malformed protocol line: {line!r}")


def _format_detail(kind: str, detail: str) -> str:
    return f"{kind},{detail}" if detail else kind
