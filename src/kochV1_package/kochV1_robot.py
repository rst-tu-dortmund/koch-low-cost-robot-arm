import logging

import numpy as np
import sympy as sp

import time

from spatialmath import SE3, UnitQuaternion

from .kochV1_kinematics import KochV1_KinematicsModel
from .kochV1_dynamixel import KochV1_DxlBus

from utils import (
    Unit,
    to_dxl_units,
)

from typing import (
    List,
)

class KochV1_Robot:
    def __init__(self, dxl_bus: KochV1_DxlBus):
        self.kinematics_model = KochV1_KinematicsModel()

        self._dxl_bus = dxl_bus

        self._apply_dh_q_offsets()
        logging.info("Robot is ready")

    def set_joints(self, joint_angles: List[float], unit: Unit = Unit.RAD):
        self._dxl_bus.set_joints(joint_angles, unit=unit)

    def read_joints(self, unit: Unit = Unit.RAD) -> List[float]:
        joint_angles = self._dxl_bus.read_joints(unit)
        return joint_angles

    def set_gripper_position_from_transform(self, transform: SE3):
        desired_joint_angles = self.kinematics_model.compute_inverse_kinematics(transform)
        logging.debug(f"\n\tDesired angles: {np.round(np.rad2deg(desired_joint_angles), 2)}")
        
        self.set_joints(desired_joint_angles, unit=Unit.RAD)

    def set_gripper_position_from_xyz_psi_phi(self, x: float, y: float, z: float, psi: float, phi: float):
        transform = self._create_transform_from_xyz_psi_phi(x, y, z, psi, phi)

        self.set_gripper_position_from_transform(transform)

    def get_gripper_transform(self) -> SE3:
        joint_angles = self.read_joints(unit=Unit.RAD)
        T_ee = self.kinematics_model.compute_forward_kinematics(joint_angles)
        return T_ee

    def set_gripper_percentage(self, percentage: float):
        """
        The percentage is between 0.0 and 1.0, where 1.0 is fully closed.
        """
        self._dxl_bus.set_gripper_percentage(percentage)

    def set_goal_velocities(self, joint_velocities: List[float]):
        """
        Set the joint velocities of the robot.
        :param joint_velocities: List of joint velocities in RAD/S.
        """

        joint_velocities_verified = []
        for motor, joint_velocity in zip(self._dxl_bus.motors[:5], joint_velocities):
            joint_velocity_verified = int(np.clip(joint_velocity, -150, 150))   # limit to max velocity of the motor
            joint_velocities_verified.append(joint_velocity_verified)

            # see dynamixel profile velocity and profile acceleration
            # https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/
            # set max possible acceleration based on velocity
            acceleration_value = int(np.abs(joint_velocity_verified) / 2)
            self._dxl_bus.write_reg(motor.id, motor.RAM.PROFILE_ACCELERATION, acceleration_value)
        
        self._dxl_bus.set_goal_velocities(joint_velocities_verified)

    def read_joint_velocities(self, unit=Unit.RAD_S):
        return self._dxl_bus.read_joint_velocities(unit)        

    def set_velocity_control_mode(self):
        self._dxl_bus.set_velocity_control_mode()
    
    def set_epcm_control_mode(self):
        self._dxl_bus.set_epcm_control_mode()

    def _apply_dh_q_offsets(self):
        for i, dh_joint in enumerate(self.kinematics_model.robot_cfg.dh_joints):
            self._dxl_bus.motors[i].physical_home_position += to_dxl_units(dh_joint.q_offset, from_unit=Unit.RAD)
        
    def _create_transform_from_xyz_psi_phi(self, x: float, y: float, z: float, psi: float, phi: float) -> SE3:
        """
        Create a transformation matrix from the given parameters.
        :param x: X-coordinate.
        :param y: Y-coordinate.
        :param z: Z-coordinate.
        :param psi: Rotation around the Z-axis (in radians).
        :param phi: Rotation around the Y-axis (in radians).
        :return: SE3 transformation matrix.
        """
    
        # Step 1: Calculate theta1 - rotation around the global Z-axis
        theta1 = np.arctan2(y, x) if x != 0 else 0


        z_hat = np.array([np.cos(psi)*np.cos(theta1),
                          np.cos(psi)*np.sin(theta1),
                          np.sin(psi)])

        y_ref  = np.array([-np.sin(theta1), np.cos(theta1), 0.0])
        x_hat0 = np.cross(z_hat, y_ref)
        x_hat0 /= np.linalg.norm(x_hat0)
        y_hat0 = np.cross(z_hat, x_hat0)

        cos_phi = np.cos(phi)
        sin_phi = np.sin(phi)
        x_hat =  cos_phi * x_hat0 + sin_phi * y_hat0
        y_hat = -sin_phi * x_hat0 + cos_phi * y_hat0

        R = np.column_stack((x_hat, y_hat, z_hat))   # [X Y Z] as columns
        return SE3.Rt(R, [x, y, z])