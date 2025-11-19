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
@brief High-level Dynamixel bus for the KochV1 robot.

Provides convenience methods to configure XL330/XL430 motors and control joints and gripper
via the lower-level `DxlBus` interface.
"""

import logging

import numpy as np

from dynamixel import (
    DxlBus,
)

from dynamixel.motor_specs import(
    DynamixelXL330_M288, XL330_M288OperatingModeType,
    DynamixelXL430_W250, XL430_W250OperatingModeType
)

from utils import (
    Unit,
    to_dxl_units, from_dxl_units,
    unpack_i32_le, to_u32,
)

from typing import (
    List,
)

class KochV1_DxlBus(DxlBus):
    """
    @brief Dynamixel bus specialization for the robot arm Koch V1.1.

    Wraps a `DxlBus` with model-specific motor initialization, default sync groups,
    and helpers to command joints and the gripper in physical units.
    """

    def __init__(self, motor_physical_home_positions: List[int], 
                 port_name: str = "", baudrate: int = 1_000_000, protocol_version: float = 2.0):
        """
        @brief Construct a KochV1_DxlBus, initialize motor metadata and create default sync groups.

        @param motor_physical_home_positions list[int] Physical home positions for each motor ID 1-6.
        @param port_name str Serial port name; auto-detected if empty.
        @param baudrate int Bus baudrate in bits per second.
        @param protocol_version float Dynamixel protocol version used by the SDK.
        """

        super().__init__(port_name, baudrate, protocol_version)

        self.motors: List[DynamixelXL430_W250 | DynamixelXL330_M288] = self._init_motors(motor_physical_home_positions)

        self.make_default_sync_groups()

    def __enter__(self):
        """
        @brief Open the bus and prepare motors for motion.

        Connects the port, sets extended position control mode, initializes physical home positions,
        configures velocity/acceleration, and enables torque on all motors.

        @return KochV1_DxlBus Active bus instance for use as a context manager.
        """

        self.connect()

        #if not self._validate_motors_operating_mode():
        #    self.disconnect()
        #    raise RuntimeError("Invalid motor configuration: incorrect Operating Mode, please check Motors using Wizard 2.0")
        
        for motor in self.motors:
            self.set_epcm_control_mode()
            self._init_physical_home_position(motor)
            
            self._set_motor_velocity_and_acceleration(motor, velocity=150, acceleration=10)

            # it is possible to configure EEPROM so torque is enabled by default 
            # see https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#startup-configuration
            self._enable_torque(motor)  

        return self
    
    def __exit__(self, exc_type, exc_value, traceback):
        """
        @brief Disable torque and close the bus on context exit.

        Disables torque on all motors and disconnects the port.
        """

        for motor in self.motors:
            self._disable_torque(motor)
        
        self.disconnect()
        
        if exc_type is not None:
            raise exc_value

    def make_default_sync_groups(self):
        """
        @brief Create default sync groups for joint positions and velocities.

        Initializes sync groups for joint angle write/read and joint velocity write/read
        for motors 1-5 (motor 6 is the gripper).
        """

        # Addresses for Goal Position register are the same for both types of motors
        # https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/
        # https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/
        self.make_sync_group("joint_angles_write", DynamixelXL430_W250.RAM.GOAL_POSITION, [1, 2, 3, 4, 5])  # motor 6 is gripper 
        self.make_sync_group("joint_angles_read", DynamixelXL430_W250.RAM.PRESENT_POSITION, [1, 2, 3, 4, 5])  # motor 6 is gripper 

        self.make_sync_group("goal_velocities", DynamixelXL430_W250.RAM.GOAL_VELOCITY, [1, 2, 3, 4, 5])  # motor 6 is gripper
        self.make_sync_group("joint_velocities_read", DynamixelXL430_W250.RAM.PRESENT_VELOCITY, [1, 2, 3, 4, 5])  # motor 6 is gripper 

    def set_joints(self, joint_angles: List[float], unit: Unit = Unit.RAD):
        """
        @brief Command joint positions for motors 1-5.

        @param joint_angles list[float] Desired joint angles for joints 1-5.
        @param unit Unit Unit of the input angles.
        """

        if len(joint_angles) != 5:
            raise ValueError("Expected 5 joint angles for the robot arm.")
        
        data_write = {}

        for angle, motor in zip(joint_angles, self.motors):
            logical_desired_pos = to_dxl_units(angle, from_unit=unit)
            physical_desired_pos = logical_desired_pos + motor.physical_home_position
            
            data_write[motor.id] = physical_desired_pos

        logging.debug(f"Data Write: {[val for val in data_write.values()]}")
        self._sync_groups["joint_angles_write"].write(data=data_write)

    def read_joints(self, unit: Unit = Unit.RAD) -> List[float]:
        """
        @brief Read current joint positions for motors 1-5.

        @param unit Unit Target unit for returned joint angles.
        @return list[float] Joint angles in the requested unit.
        """

        data = self._sync_groups["joint_angles_read"].read()

        joint_angles = []
        joint_angles_dxl = []

        for value, motor in zip(data.values(), self.motors[:5]):
            joint_dxl = unpack_i32_le(value) - motor.physical_home_position
            joint_unit = from_dxl_units(joint_dxl, unit)

            joint_angles.append(joint_unit)

            joint_angles_dxl.append(unpack_i32_le(value))
        
        logging.debug(f"Data read: {joint_angles_dxl}")
        return joint_angles
    
    def set_gripper_percentage(self, percentage: float):
        """
        @brief Set the gripper opening as a percentage.

        The percentage is mapped from the gripper's physical position limits, with a safety
        margin to avoid hitting mechanical stops.

        @param percentage float Gripper position in [0.0, 1.0], where 1.0 is fully closed.
        """

        gripper_motor = self.motors[5]
        motor_limits = gripper_motor.physical_position_limits
        
        # cause of assembly min lim is full closed gripper
        # Add 100 DXL Units to avoid hitting the limit
        full_closed = motor_limits[0] + 100
        full_opened = motor_limits[1] - 100

        gripper_percentage_dxl_unit = int(np.abs(full_closed - full_opened) * percentage)
        
        gripper_position = full_opened - gripper_percentage_dxl_unit # minus sign because of assembly

        logging.debug(f"Gripper Data Write: {gripper_position}")
        self.write_reg(6, DynamixelXL330_M288.RAM.GOAL_POSITION, gripper_position)

    def set_goal_velocities(self, joint_velocities: List[float], unit=Unit.RAD_S):
        """
        @brief Set target velocities for joints 1-5.

        @param joint_velocities list[float] Desired joint velocities for joints 1-5.
        @param unit Unit Unit of the input velocities.
        """

        if len(joint_velocities) != 5:
            raise ValueError("Expected 5 joint velocities for the robot arm.")
        
        logging.debug(f"Velocities Write: {joint_velocities}")

        for motor, joint_velocity in zip(self.motors[:5], joint_velocities):
            joint_velocity_dxl = to_dxl_units(joint_velocity, unit)
            joint_velocity_verified = int(np.clip(joint_velocity_dxl, -150, 150))   # limit to max velocity of the motor
            # see dynamixel profile velocity and profile acceleration
            # https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/
            # set max possible acceleration based on velocity
            acceleration_value = int(np.abs(joint_velocity_verified) / 2)
            self._set_motor_velocity_and_acceleration(motor, joint_velocity_verified, acceleration_value)

    def read_joint_velocities(self, unit=Unit.RAD_S):
        """
        @brief Read current joint velocities for motors 1-5.

        @param unit Unit Target unit for returned velocities.
        @return list[float] Joint velocities in the requested unit.
        """

        data = self._sync_groups["joint_velocities_read"].read()
        
        velocities = []
        velocities_dxl = []

        for value in data.values():
            vel_dxl = unpack_i32_le(value)
            vel_unit = from_dxl_units(vel_dxl, unit)

            velocities.append(vel_unit)

            velocities_dxl.append(unpack_i32_le(value))
        
        logging.debug(f"Data read: {velocities_dxl}")
        return velocities

    def set_velocity_control_mode(self):
        """
        @brief Switch joints 1-5 to velocity control mode.

        @note Torque is briefly disabled and re-enabled, which can cause a small positional shift.
        """

        for motor in self.motors[:5]:
            # For both motor types same register value for velocity control mode
            self.set_operating_mode(motor, operating_mode = XL430_W250OperatingModeType.VELOCITY_CONTROL_MODE)

    def set_epcm_control_mode(self):
        """
        @brief Switch joints 1-5 to extended position control mode.

        @note Torque is briefly disabled and re-enabled, which can cause a small positional shift.
              After changing the operating mode, Profile Velocity and Profile Acceleration are
              reset to 0 and are reconfigured in this method.
        """

        # After setting new operating mode, the Profile Velocity and Profile Acceleration 
        # will be reset to default values (0) ->  need to set them again
        for motor in self.motors[:5]:
            # For both motor types same register value for velocity control mode
            self.set_operating_mode(motor, operating_mode = XL430_W250OperatingModeType.EXTENDED_POSITION_CONTROL_MODE)
            self._set_motor_velocity_and_acceleration(motor, velocity=150, acceleration=10)

    def set_operating_mode(self, motor: DynamixelXL330_M288 | DynamixelXL430_W250, 
                           operating_mode: XL330_M288OperatingModeType | XL430_W250OperatingModeType):
        """
        @brief Set the operating mode for a specific motor.

        Disables torque before writing the mode and re-enables torque afterwards.

        @param motor DynamixelXL330_M288|DynamixelXL430_W250 Target motor instance.
        @param operating_mode XL330_M288OperatingModeType|XL430_W250OperatingModeType Desired mode value.
        @note This method temporarily disables motor torque.
        """

        self._disable_torque(motor)

        self.write_reg(motor.id, motor.EEPROM.OPERATING_MODE, operating_mode)

        self._enable_torque(motor)

    def _init_motors(self, p_home: List[int]):
        """
        @brief Build motor objects with physical home positions and limits.

        @param p_home list[int] Physical home positions read from the assembly for motors 1-6.
        @return list[DynamixelXL430_W250|DynamixelXL330_M288] Configured motor objects.
        """

        # Motor physical home positions are depending on the assembly of the robot
        # But the movement limits are fixed for the robot design for chosen convenience
        logical_limits = [
            (1024, 1024),
            (180, 1450),
            (1950, 160),
            (1024, 1024),
            (2048, 2048),
            (200, 800),
        ]
        return [
            DynamixelXL430_W250(1, p_home[0],        np.array([p_home[0] - logical_limits[0][0], p_home[0] + logical_limits[0][1]])),
            DynamixelXL430_W250(2, p_home[1] + 88,   np.array([p_home[1] - logical_limits[1][0], p_home[1] + logical_limits[1][1]])), # motor zero -> dh-zero 
            DynamixelXL330_M288(3, p_home[2] - 106,  np.array([p_home[2] - logical_limits[2][0], p_home[2] + logical_limits[2][1]])), # motor zero -> dh-zero 
            DynamixelXL330_M288(4, p_home[3] - 1006, np.array([p_home[3] - logical_limits[3][0], p_home[3] + logical_limits[3][1]])), # motor zero -> dh-zero 
            DynamixelXL330_M288(5, p_home[4],        np.array([p_home[4] - logical_limits[4][0], p_home[4] + logical_limits[4][1]])),    
            DynamixelXL330_M288(6, p_home[5],        np.array([p_home[5] - logical_limits[5][0], p_home[5] + logical_limits[5][1]])),
        ]

    def _validate_motor_operating_mode(self, motor: DynamixelXL330_M288 | DynamixelXL430_W250,
                                       operating_mode: XL330_M288OperatingModeType | XL430_W250OperatingModeType):
        """
        @brief Check whether a motor is currently set to a given operating mode.

        @param motor DynamixelXL330_M288|DynamixelXL430_W250 Motor to query.
        @param operating_mode XL330_M288OperatingModeType|XL430_W250OperatingModeType Expected mode value.
        @return bool True if the EEPROM OPERATING_MODE matches the expected value.
        """

        return self.read_reg(motor.id, motor.EEPROM.OPERATING_MODE) == operating_mode
    
    def _init_physical_home_position(self, motor: DynamixelXL330_M288 | DynamixelXL430_W250):
        """
        @brief Adjust physical home position and limits based on current reading.

        Reads the present position and, if necessary, shifts the stored physical home position and
        limits by one full rotation to keep the current reading inside the allowed range.

        @param motor DynamixelXL330_M288|DynamixelXL430_W250 Motor to initialize.
        """

        # check if at the turn on the start position was correctly read
        current_pos_raw = self.read_reg(motor.id, motor.RAM.PRESENT_POSITION) 
        
        # NOTE may be unpack here not needed because start position may be only in range [0 .. 4095]?
        # NOTE probably still needs to be, because it can fluctuate around zero at the start
        #       either (5...-5) or (4100...4090) 
        physical_pos = unpack_i32_le(current_pos_raw)
        
        if physical_pos < motor.physical_position_limits[0]:
            motor.physical_home_position -= motor.FULL_ROTATION_TICKS
            motor.physical_position_limits -= motor.FULL_ROTATION_TICKS

        elif physical_pos > motor.physical_position_limits[1]:
            motor.physical_home_position += motor.FULL_ROTATION_TICKS
            motor.physical_position_limits += motor.FULL_ROTATION_TICKS


    def _set_motor_velocity_and_acceleration(
            self, motor: DynamixelXL330_M288 | DynamixelXL430_W250, 
            velocity: int = 150, acceleration: int = 10
            ):
        """
        @brief Configure profile velocity and acceleration for a motor.

        @param motor DynamixelXL330_M288|DynamixelXL430_W250 Motor to configure.
        @param velocity int Profile velocity in raw Dynamixel units.
        @param acceleration int Profile acceleration in raw Dynamixel units.
        """
        
        self.write_reg(motor.id, motor.RAM.PROFILE_VELOCITY, to_u32(velocity))
        self.write_reg(motor.id, motor.RAM.PROFILE_ACCELERATION, acceleration)

    def _enable_torque(self, motor: DynamixelXL330_M288 | DynamixelXL430_W250):
        """
        @brief Enable torque on a motor.

        @param motor DynamixelXL330_M288|DynamixelXL430_W250 Motor to enable.
        """
        self.write_reg(motor.id, motor.RAM.TORQUE_ENABLE, 1)

    def _disable_torque(self, motor: DynamixelXL330_M288 | DynamixelXL430_W250):
        """
        @brief Disable torque on a motor.

        @param motor DynamixelXL330_M288|DynamixelXL430_W250 Motor to disable.
        """
        self.write_reg(motor.id, motor.RAM.TORQUE_ENABLE, 0)