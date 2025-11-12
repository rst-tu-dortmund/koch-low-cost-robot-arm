"""
@brief Types for Dynamixel control-table metadata and motor protocol.

Defines a lightweight `ControlField` to describe a single register entry and a `DynamixelMotor`
protocol that model-specific motor classes implement (exposing EEPROM/RAM control tables).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import (
    Optional,
    ClassVar,
    Protocol,
    Type,
)

@dataclass(frozen=True)
class ControlField:
    """
    @brief Descriptor of a Dynamixel control-table field.

    @param address int Register start address.
    @param size int Field length in bytes (1, 2, or 4).
    @param initial_value Optional[int] Initial value after reboot.
    """
    address: int
    size: int
    initial_value: Optional[int]


class DynamixelMotor(Protocol):
    """
    @brief Protocol for Dynamixel motor models exposing control-table classes.

    Implementations provide class attributes `EEPROM` and `RAM` (namespaces of `ControlField`s)
    and an `id` identifying the device on the bus.

    @note This is a typing protocol; it specifies the expected interface but does not implement it.
    """
    EEPROM: ClassVar[Type]
    RAM:    ClassVar[Type]

    id: int

    def __init__(self, id: int): ...