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
@brief Control-table models and dataclass for the Dynamixel XL430-W250.

Provides lightweight Enum types for Operating modes, plain classes that map the EEPROM/RAM
control tables to `ControlField` descriptors, and a small dataclass bundling per-motor
calibration such as home position and limits.

@see XL330_M288EEPROMControlTable
@see XL330_M288RAMControlTable
"""


import numpy as np

from enum import IntEnum
from dataclasses import dataclass, field

from .base import ControlField

from typing import (
    ClassVar,
    Tuple
)


class XL430_W250OperatingModeType(IntEnum):
    """@brief Operating modes supported by the Dynamixel XL430-W250."""

    VELOCITY_CONTROL_MODE           = 1
    POSITION_CONTROL_MODE           = 3
    EXTENDED_POSITION_CONTROL_MODE  = 4
    PWM_CONTROL_MODE                = 16


class XL430_W250EEPROMControlTable:
    """
    @brief EEPROM control-table fields for the Dynamixel XL430-W250.

    Each attribute is a `ControlField` with (address, size and initial value).
    These fields reside in non-volatile memory and typically require torque-off to
    modify.

    @note Assumption: addresses and defaults follow the Robotis XL430-W250 e-Manual.
    """

    MODEL_NUMBER            = ControlField(address=0,  size=2, initial_value=1060)
    MODEL_INFORMATION       = ControlField(address=2,  size=4, initial_value=None)
    FIRMWARE_VERSION        = ControlField(address=6,  size=1, initial_value=None)
    ID                      = ControlField(address=7,  size=1, initial_value=1)
    BAUD_RATE               = ControlField(address=8,  size=1, initial_value=1)
    RETURN_DELAY_TIME       = ControlField(address=9,  size=1, initial_value=250)
    DRIVE_MODE              = ControlField(address=10, size=1, initial_value=0)
    OPERATING_MODE          = ControlField(address=11, size=1, initial_value=3)
    SECONDARY_ID            = ControlField(address=12, size=1, initial_value=255)
    PROTOCOL_TYPE           = ControlField(address=13, size=1, initial_value=2)
    HOMING_OFFSET           = ControlField(address=20, size=4, initial_value=0)
    MOVING_THRESHOLD        = ControlField(address=24, size=4, initial_value=10)
    TEMPERATURE_LIMIT       = ControlField(address=31, size=1, initial_value=72)
    MAX_VOLTAGE_LIMIT       = ControlField(address=32, size=2, initial_value=140)
    MIN_VOLTAGE_LIMIT       = ControlField(address=34, size=2, initial_value=60)
    PWM_LIMIT               = ControlField(address=36, size=2, initial_value=885)
    VELOCITY_LIMIT          = ControlField(address=44, size=4, initial_value=265)
    MAX_POSITION_LIMIT      = ControlField(address=48, size=4, initial_value=4095)
    MIN_POSITION_LIMIT      = ControlField(address=52, size=4, initial_value=0)
    STARTUP_CONFIGURATION   = ControlField(address=60, size=1, initial_value=0)
    SHUTDOWN                = ControlField(address=63, size=1, initial_value=52)


class XL430_W250RAMControlTable:
    """
    @brief RAM control-table fields for the Dynamixel XL430-W250.

    Each attribute is a `ControlField` with (address, size, initial value at power-on).
    These fields reside in volatile memory and can be changed while torque is enabled,
    subject to device rules.

    @note Assumption: addresses and defaults follow the Robotis XL430-W250 e-Manual.
    """
    
    TORQUE_ENABLE           = ControlField(address=64,  size=1, initial_value=0)
    LED                     = ControlField(address=65,  size=1, initial_value=0)
    STATUS_RETURN_LEVEL     = ControlField(address=68,  size=1, initial_value=2)
    REGISTERED_INSTRUCTION  = ControlField(address=69,  size=1, initial_value=0)
    HARDWARE_ERROR_STATUS   = ControlField(address=70,  size=1, initial_value=0)
    VELOCITY_I_GAIN         = ControlField(address=76,  size=2, initial_value=1000)
    VELOCITY_P_GAIN         = ControlField(address=78,  size=2, initial_value=100)
    POSITION_D_GAIN         = ControlField(address=80,  size=2, initial_value=4000)
    POSITION_I_GAIN         = ControlField(address=82,  size=2, initial_value=0)
    POSITION_P_GAIN         = ControlField(address=84,  size=2, initial_value=640)
    FEEDFORWARD_2ND_GAIN    = ControlField(address=88,  size=2, initial_value=0)
    FEEDFORWARD_1ST_GAIN    = ControlField(address=90,  size=2, initial_value=0)
    BUS_WATCHDOG            = ControlField(address=98,  size=1, initial_value=0)
    GOAL_PWM                = ControlField(address=100, size=2, initial_value=None)
    GOAL_VELOCITY           = ControlField(address=104, size=4, initial_value=None)
    PROFILE_ACCELERATION    = ControlField(address=108, size=4, initial_value=0)
    PROFILE_VELOCITY        = ControlField(address=112, size=4, initial_value=0)
    GOAL_POSITION           = ControlField(address=116, size=4, initial_value=None)
    REALTIME_TICK           = ControlField(address=120, size=2, initial_value=None)
    MOVING                  = ControlField(address=122, size=1, initial_value=0)
    MOVING_STATUS           = ControlField(address=123, size=1, initial_value=0)
    PRESENT_PWM             = ControlField(address=124, size=2, initial_value=None)
    PRESENT_LOAD            = ControlField(address=126, size=2, initial_value=None)
    PRESENT_VELOCITY        = ControlField(address=128, size=4, initial_value=None)
    PRESENT_POSITION        = ControlField(address=132, size=4, initial_value=None)
    VELOCITY_TRAJECTORY     = ControlField(address=136, size=4, initial_value=None)
    POSITION_TRAJECTORY     = ControlField(address=140, size=4, initial_value=None)
    PRESENT_INPUT_VOLTAGE   = ControlField(address=144, size=2, initial_value=None)
    PRESENT_TEMPERATURE     = ControlField(address=146, size=1, initial_value=None)
    BACKUP_READY            = ControlField(address=147, size=1, initial_value=None)


@dataclass
class DynamixelXL430_W250:
    """
    @brief Per-motor data and shared specs for the model XL430-W250.

    Stores the device ID, assembly-specific home position, and physical position limits (in
    DXL ticks). Also exposes class attributes for the model's EEPROM/RAM control tables,
    tick-per-revolution constant, and operating-mode enum.

    @note `logical_position_limits` are derived as `physical_position_limits - physical_home_position`.
    """

    id: int
    physical_home_position: int
    physical_position_limits: np.ndarray[int, int]

    logical_position_limits: Tuple[int] = field(init=False) # (MIN, MAX) in DXL-units

    # class attributes
    EEPROM:                 ClassVar[type[XL430_W250EEPROMControlTable]]    = XL430_W250EEPROMControlTable
    RAM:                    ClassVar[type[XL430_W250RAMControlTable]]       = XL430_W250RAMControlTable

    FULL_ROTATION_TICKS:    ClassVar[type[int]]                             = 4096
    
    OperatingModeType:      ClassVar[type[XL430_W250OperatingModeType]]     = XL430_W250OperatingModeType

    
    def __post_init__(self):
        self.logical_position_limits = self.physical_position_limits - self.physical_home_position
