"""
@brief Control-table models and dataclass for the Dynamixel XL330-M288.

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

class XL330_M288OperatingModeType(IntEnum):
    """@brief Operating modes supported by the Dynamixel XL330-M288."""
    
    VELOCITY_CONTROL_MODE               = 1
    POSITION_CONTROL_MODE               = 3
    EXTENDED_POSITION_CONTROL_MODE      = 4
    CURRENT_BASED_POSITION_CONTROL_MODE = 5
    PWM_CONTROL_MODE                    = 16


class XL330_M288EEPROMControlTable:
    """
    @brief EEPROM control-table fields for the Dynamixel XL330-M288.

    Each attribute is a `ControlField` with (address, size and initial value).
    These fields reside in non-volatile memory and typically require torque-off to
    modify.

    @note Assumption: addresses and defaults follow the Robotis XL330-M288 e-Manual.
    """
    
    MODEL_NUMBER            = ControlField(address=0,  size=2, initial_value=1200)
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
    TEMPERATURE_LIMIT       = ControlField(address=31, size=1, initial_value=70)
    MAX_VOLTAGE_LIMIT       = ControlField(address=32, size=2, initial_value=70)
    MIN_VOLTAGE_LIMIT       = ControlField(address=34, size=2, initial_value=35)
    PWM_LIMIT               = ControlField(address=36, size=2, initial_value=885)
    CURRENT_LIMIT           = ControlField(address=38, size=2, initial_value=1750)
    VELOCITY_LIMIT          = ControlField(address=44, size=4, initial_value=445)
    MAX_POSITION_LIMIT      = ControlField(address=48, size=4, initial_value=4095)
    MIN_POSITION_LIMIT      = ControlField(address=52, size=4, initial_value=0)
    STARTUP_CONFIGURATION   = ControlField(address=60, size=1, initial_value=0)
    PWM_SLOPE               = ControlField(address=62, size=1, initial_value=140)
    SHUTDOWN                = ControlField(address=63, size=1, initial_value=53)


class XL330_M288RAMControlTable:
    """
    @brief RAM control-table fields for the Dynamixel XL330-M288.

    Each attribute is a `ControlField` with (address, size, initial value at power-on).
    These fields reside in volatile memory and can be changed while torque is enabled,
    subject to device rules.

    @note Assumption: addresses and defaults follow the Robotis XL330-M288 e-Manual.
    """

    TORQUE_ENABLE           = ControlField(address=64,  size=1, initial_value=0)
    LED                     = ControlField(address=65,  size=1, initial_value=0)
    STATUS_RETURN_LEVEL     = ControlField(address=68,  size=1, initial_value=2)
    REGISTERED_INSTRUCTION  = ControlField(address=69,  size=1, initial_value=0)
    HARDWARE_ERROR_STATUS   = ControlField(address=70,  size=1, initial_value=0)
    VELOCITY_I_GAIN         = ControlField(address=76,  size=2, initial_value=1600)
    VELOCITY_P_GAIN         = ControlField(address=78,  size=2, initial_value=180)
    POSITION_D_GAIN         = ControlField(address=80,  size=2, initial_value=0)
    POSITION_I_GAIN         = ControlField(address=82,  size=2, initial_value=0)
    POSITION_P_GAIN         = ControlField(address=84,  size=2, initial_value=400)
    FEEDFORWARD_2ND_GAIN    = ControlField(address=88,  size=2, initial_value=0)
    FEEDFORWARD_1ST_GAIN    = ControlField(address=90,  size=2, initial_value=0)
    BUS_WATCHDOG            = ControlField(address=98,  size=1, initial_value=0)
    GOAL_PWM                = ControlField(address=100, size=2, initial_value=None)
    GOAL_CURRENT            = ControlField(address=102, size=2, initial_value=None)
    GOAL_VELOCITY           = ControlField(address=104, size=4, initial_value=None)
    PROFILE_ACCELERATION    = ControlField(address=108, size=4, initial_value=0)
    PROFILE_VELOCITY        = ControlField(address=112, size=4, initial_value=0)
    GOAL_POSITION           = ControlField(address=116, size=4, initial_value=None)
    REALTIME_TICK           = ControlField(address=120, size=2, initial_value=None)
    MOVING                  = ControlField(address=122, size=1, initial_value=0)
    MOVING_STATUS           = ControlField(address=123, size=1, initial_value=0)
    PRESENT_PWM             = ControlField(address=124, size=2, initial_value=None)
    PRESENT_CURRENT         = ControlField(address=126, size=2, initial_value=None)
    PRESENT_VELOCITY        = ControlField(address=128, size=4, initial_value=None)
    PRESENT_POSITION        = ControlField(address=132, size=4, initial_value=None)
    VELOCITY_TRAJECTORY     = ControlField(address=136, size=4, initial_value=None)
    POSITION_TRAJECTORY     = ControlField(address=140, size=4, initial_value=None)
    PRESENT_INPUT_VOLTAGE   = ControlField(address=144, size=2, initial_value=None)
    PRESENT_TEMPERATURE     = ControlField(address=146, size=1, initial_value=None)
    BACKUP_READY            = ControlField(address=147, size=1, initial_value=None)


@dataclass
class DynamixelXL330_M288:
    """
    @brief Per-motor data and shared specs for the model XL330-M288.

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
    EEPROM:                 ClassVar[type[XL330_M288EEPROMControlTable]]    = XL330_M288EEPROMControlTable
    RAM:                    ClassVar[type[XL330_M288RAMControlTable]]       = XL330_M288RAMControlTable

    FULL_ROTATION_TICKS:    ClassVar[type[int]]                             = 4096

    OperatingModeType:      ClassVar[type[XL330_M288OperatingModeType]]     = XL330_M288OperatingModeType

    def __post_init__(self):
        self.logical_position_limits = self.physical_position_limits - self.physical_home_position