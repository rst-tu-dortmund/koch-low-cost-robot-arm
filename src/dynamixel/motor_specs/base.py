# Copyright 2025, Institute for Control Theory and Systems Engineering, 
# TU Dortmund University
#
# Redistribution and use in source and binary forms, with or without 
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, 
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, 
# this list of conditions and the following disclaimer in the documentation 
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors 
# may be used to endorse or promote products derived from this software without 
# specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” 
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
# POSSIBILITY OF SUCH DAMAGE.

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
    ### Register start address.
    address: int
    ### Field length in bytes (1, 2, or 4).
    size: int
    ### Initial value after reboot.
    initial_value: Optional[int]


class DynamixelMotor(Protocol):
    """
    @brief Protocol for Dynamixel motor models exposing control-table classes.

    Implementations provide class attributes `EEPROM` and `RAM` (namespaces of `ControlField`s)
    and an `id` identifying the device on the bus.

    @note This is a typing protocol; it specifies the expected interface but does not implement it.
    """
    ## EEPROM class type.
    EEPROM: ClassVar[Type]
    ## RAM class type.
    RAM:    ClassVar[Type]

    id: int

    def __init__(self, id: int): ...