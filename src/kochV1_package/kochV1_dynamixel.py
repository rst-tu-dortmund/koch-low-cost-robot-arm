import logging

import numpy as np

from dynamixel import (
    DxlBus,
    SyncGroup,
)

from dynamixel.motor_specs import(
    DynamixelXL330_M288, XL330_M288OperatingModeType,
    DynamixelXL430_W250, XL430_W250OperatingModeType
)

from utils import (
    Unit,
    to_dxl_units, from_dxl_units,
    pack_i32_le, unpack_i32_le, to_u32,
)

from typing import (
    List,
)

class KochV1_DxlBus(DxlBus):
    def __init__(self, motor_physical_home_positions: List[int], 
                 port_name: str = "", baudrate: int = 1_000_000, protocol_version: float = 2.0):
        super().__init__(port_name, baudrate, protocol_version)

        self.motors: List[DynamixelXL430_W250 | DynamixelXL330_M288] = self._init_motors(motor_physical_home_positions)

        self.make_default_sync_groups()

    def __enter__(self):
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
        for motor in self.motors:
            self._disable_torque(motor)
        
        self.disconnect()
        
        if exc_type is not None:
            raise exc_value

    def make_default_sync_groups(self):        
        # Addresses for Goal Position register are the same for both types of motors
        # https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/
        # https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/
        self.make_sync_group("joint_angles_write", DynamixelXL430_W250.RAM.GOAL_POSITION, [1, 2, 3, 4, 5])  # motor 6 is gripper 
        self.make_sync_group("joint_angles_read", DynamixelXL430_W250.RAM.PRESENT_POSITION, [1, 2, 3, 4, 5])  # motor 6 is gripper 

        self.make_sync_group("goal_velocities", DynamixelXL430_W250.RAM.GOAL_VELOCITY, [1, 2, 3, 4, 5])  # motor 6 is gripper
        self.make_sync_group("joint_velocities_read", DynamixelXL430_W250.RAM.PRESENT_VELOCITY, [1, 2, 3, 4, 5])  # motor 6 is gripper 

    def set_joints(self, joint_angles: List[float], unit: Unit = Unit.RAD):
        """
        Set the joint angles for the robot arm.
        :param joint_angles: List of joint angles (in radians by default).
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
        Read the current joint angles from the robot arm.
        :return: List of joint angles (in radians by default).
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
        Set the gripper position based on a percentage (0 to 1).
        :param percentage: Gripper position as a percentage.
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

    def set_goal_velocities(self, joint_velocities: List[int]):
        """
        Set the joint velocities for the robot arm.
        :param joint_velocities: List of joint velocities (in DXL units).
        """

        if len(joint_velocities) != 5:
            raise ValueError("Expected 5 joint velocities for the robot arm.")
        
        logging.debug(f"Velocities Write: {joint_velocities}")

        for velocity, motor in zip(joint_velocities, self.motors[:5]):
            self.write_reg(motor.id, motor.RAM.GOAL_VELOCITY, to_u32(velocity))

    def read_joint_velocities(self, unit=Unit.RAD_S):
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
        for motor in self.motors[:5]:
            # For both motor types same register value for velocity control mode
            self.set_operating_mode(motor, operating_mode = XL430_W250OperatingModeType.VELOCITY_CONTROL_MODE)

    def set_epcm_control_mode(self):
        # After setting new operating mode, the Profile Velocity and Profile Acceleration 
        # will be reset to default values (0) ->  need to set them again
        for motor in self.motors[:5]:
            # For both motor types same register value for velocity control mode
            self.set_operating_mode(motor, operating_mode = XL430_W250OperatingModeType.EXTENDED_POSITION_CONTROL_MODE)
            self._set_motor_velocity_and_acceleration(motor, velocity=150, acceleration=10)

    def set_operating_mode(self, motor: DynamixelXL330_M288 | DynamixelXL430_W250, 
                           operating_mode: XL330_M288OperatingModeType | XL430_W250OperatingModeType):
        """
        Set the operating mode for a specific motor.

        ATTENTION: This method causes the motor torque to be disabled and enabled again.

        :param motor: The motor to set the operating mode for.
        :param operating_mode: The desired operating mode.
        """

        self._disable_torque(motor)

        self.write_reg(motor.id, motor.EEPROM.OPERATING_MODE, operating_mode)

        self._enable_torque(motor)

    def _init_motors(self, p_home: List[int]):
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
        return self.read_reg(motor.id, motor.EEPROM.OPERATING_MODE) == operating_mode
    
    def _init_physical_home_position(self, motor: DynamixelXL330_M288 | DynamixelXL430_W250):
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
        
        self.write_reg(motor.id, motor.RAM.PROFILE_VELOCITY, to_u32(velocity))
        self.write_reg(motor.id, motor.RAM.PROFILE_ACCELERATION, acceleration)

    def _enable_torque(self, motor: DynamixelXL330_M288 | DynamixelXL430_W250):
        self.write_reg(motor.id, motor.RAM.TORQUE_ENABLE, 1)

    def _disable_torque(self, motor: DynamixelXL330_M288 | DynamixelXL430_W250):
        self.write_reg(motor.id, motor.RAM.TORQUE_ENABLE, 0)