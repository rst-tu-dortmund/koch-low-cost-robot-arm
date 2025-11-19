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
@brief Utilities for Dynamixel unit conversion and little-endian packing helpers.

Provide consistent conversions between logical units (rad, deg, rpm, rad/s) and Dynamixel "DXL" ticks,
plus small helpers for signed/unsigned 32-bit coercion and struct packing.
"""

import numpy as np
import struct

from enum import Enum

class Unit(Enum):
    """
    @brief Supported source/target units for conversion.

    Supported members:
      - DXL: Raw Dynamixel register ticks (position/velocity LSBs).
      - RAD: Radians (position).
      - DEG: Degrees (position).
      - RPM: Revolutions per minute (velocity).
      - RAD_S: Radians per second (velocity).
    @note DXL denotes raw Dynamixel position/velocity register units (ticks).
    """

    ## Unit: DXL register ticks.
    DXL     = 0  
    ## Unit: Radians.
    RAD     = 1
    ## Unit: Degrees.
    DEG     = 2
    ## Unit: Revolutions per minute.    
    RPM     = 3
    ## Unit: Radians per second.
    RAD_S   = 4



def to_dxl_units(value: float, from_unit: Unit) -> int:
    """
    @brief Convert a numeric value from a physical unit to Dynamixel ticks.

    @param value float Input value in the given unit.
    @param from_unit Unit Unit of the input value.
    @return int Value converted to DXL register ticks (truncated to int).
    @note Assumption: for velocities, DXL ticks map using vendor factors (0.229 rpm/LSB).
    """

    if from_unit == Unit.DXL:
        return int(value)
    
    elif from_unit == Unit.DEG:
        return int(value * (4096 / 360))
    
    elif from_unit == Unit.RAD:
        return int(value * (4096 / (2 * np.pi)))
    
    elif from_unit == Unit.RPM:
        return int(value / 0.229)

    elif from_unit == Unit.RAD_S:
        return int((value * (60 / (2*np.pi))) / 0.229)

    else:
        raise ValueError(f"Unsupported input unit: {from_unit}")

def from_dxl_units(value: float, to_unit: Unit) -> float:
    """
    @brief Convert a value from Dynamixel ticks to a physical unit.

    @param value float Input value in DXL register ticks.
    @param to_unit Unit Desired destination unit.
    @return float Value converted to the destination unit.
    @note Assumption: for velocities, DXL ticks map using vendor factors (0.229 rpm/LSB).
    """

    if to_unit == Unit.DXL:
        return value
    
    elif to_unit == Unit.DEG:
        return value / (4096 / 360)
    
    elif to_unit == Unit.RAD:
        return value / (4096 / (2 * np.pi))
    
    elif to_unit == Unit.RPM:
        return value * 0.229
    
    elif to_unit == Unit.RAD_S:
        return (value * 0.229) * (2*np.pi / 60)

    else:
        raise ValueError(f"Unsupported output unit: {to_unit}")


def to_u32(x):
    """
    @brief Convert a signed Python int to its unsigned 32-bit representation.

    @param x int Input value (any Python int).
    @return int Unsigned 32-bit representation (range 0..0xFFFFFFFF).
    """
    return x & 0xFFFFFFFF

def pack_i32_le(value: int) -> bytes:
    """
    @brief Pack a signed 32-bit integer in little-endian form.

    @param value int Value to pack.
    @return bytes 4-byte little-endian buffer.
    """
    return struct.pack('<i', value)

def unpack_i32_le(raw: int | bytes | bytearray) -> int:
    """
    @brief Unpack a signed 32-bit little-endian integer.

    Accepts either a 32-bit unsigned integer (as Python int) or a 4-byte buffer and returns the
    corresponding signed int32.

    @param raw int|bytes|bytearray Unsigned 32-bit integer or 4-byte little-endian buffer.
    @return int Decoded signed 32-bit integer.
    @note Assumption: byte order is little-endian; ints are coerced to 32 bits before unpacking.
    """

    if isinstance(raw, int):
        raw_bytes = struct.pack('<I', raw & 0xFFFFFFFF) # ensure 32-bit integer is incoming - eventually not needed, because SDK-outputs should be trusted values
    elif isinstance(raw, (bytes, bytearray)):
        if (len(raw) != 4):
            raise ValueError("Byte sequence must be exactly 4 bytes!") 
        raw_bytes = bytes(raw)

    return struct.unpack('<i', raw_bytes)[0]   


def check_limits(number, limits) -> bool:
    """
    @brief Check if a number lies within an inclusive interval.

    @param number float|int Value to test.
    @param limits tuple[float|int, float|int] Inclusive lower and upper bounds.
    @return bool True if limits[0] <= number <= limits[1], else False.
    """

    return limits[0] <= number <= limits[1]