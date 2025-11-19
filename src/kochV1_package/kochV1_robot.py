"""
@brief High-level robot control interface for robot arm Koch V1.1.

Provides thin orchestration over a `KochV1_DxlBus` and the kinematics model to set/read joint states,
move the end-effector by pose or task-space parameters, and configure control modes.

@see KochV1_DxlBus for low-level motor/Bus I/O.
@see KochV1_KinematicsModel for forward/inverse kinematics.
"""

import logging

import numpy as np

from spatialmath import SE3

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
    """
    @brief High-level interface for commanding the Koch V1.1 robot arm.

    Exposes convenience methods to set joint angles/velocities, compute and execute IK targets, and
    read back the joint configuration and end-effector transform.
    """

    def __init__(self, dxl_bus: KochV1_DxlBus):
        """
        @brief Construct the robot interface.

        @param dxl_bus KochV1_DxlBus Pre-initialized Dynamixel bus wrapper.
        """

        self.kinematics_model = KochV1_KinematicsModel()

        self._dxl_bus = dxl_bus

        self._apply_dh_q_offsets()
        logging.info("Robot is ready")

    def set_joints(self, joint_angles: List[float], unit: Unit = Unit.RAD):
        """
        @brief Set the 5 joint angles.

        @param joint_angles List[float] Target joint angles ordered as the kinematic chain expects.
        @param unit Unit Unit of the provided angles (default: radians).
        """
        self._dxl_bus.set_joints(joint_angles, unit=unit)

    def read_joints(self, unit: Unit = Unit.RAD) -> List[float]:
        """
        @brief Read the current joint configuration.

        @param unit Unit Desired output unit (default: radians).
        @return List[float] Current joint angles in the requested unit ordered as the kinematic chain expects.
        """
        joint_angles = self._dxl_bus.read_joints(unit)
        return joint_angles

    def set_gripper_position_from_transform(self, transform: SE3):
        """
        @brief Move the end-effector to a target pose via inverse kinematics.

        @param transform SE3 Desired end-effector pose in the base frame.
        @note Uses the kinematics model to compute joint angles; no path planning is performed.
        """

        desired_joint_angles = self.kinematics_model.compute_inverse_kinematics(transform)
        logging.debug(f"\n\tDesired angles: {np.round(np.rad2deg(desired_joint_angles), 2)}")
        
        self.set_joints(desired_joint_angles, unit=Unit.RAD)

    def set_gripper_position_from_xyz_psi_phi(self, x: float, y: float, z: float, psi: float, phi: float):
        """
        @brief Move the end-effector using Cartesian position and two orientation angles.

        Builds an SE3 transform from (x, y, z, psi, phi) and executes IK.

        @param x float Position X in meters.
        @param y float Position Y in meters.
        @param z float Position Z in meters.
        @param psi float Elevation of the end-effector Z-axis above the horizontal plane (radians).
        @param phi float Rotation about the end-effector Z-axis (radians).
        """

        transform = self._create_transform_from_xyz_psi_phi(x, y, z, psi, phi)

        self.set_gripper_position_from_transform(transform)

    def get_gripper_transform(self) -> SE3:
        """
        @brief Compute the current end-effector pose from measured joints.

        @return SE3 Current transform of the end-effector in the base frame.
        """

        joint_angles = self.read_joints(unit=Unit.RAD)
        T_ee = self.kinematics_model.compute_forward_kinematics(joint_angles)
        return T_ee

    def set_gripper_percentage(self, percentage: float):
        """
        @brief Set gripper closure as a fraction of the span.

        @param percentage float Value in [0.0, 1.0]; 1.0 means fully closed.
        """
        self._dxl_bus.set_gripper_percentage(percentage)

    def set_goal_velocities(self, joint_velocities: List[float], unit=Unit.RAD_S):
        """
        @brief Set joint velocities for the first 5 joints (velocity control).

        @param joint_velocities List[float] Target joint velocities in DXL ordered as the kinematic chain expects; 
                                values are clipped to the motor's supported range before sending.
        @param Unit Unit of the provided velocities (default: rad/s).
        @note Also updates each motor's `PROFILE_ACCELERATION` based on the requested velocity.
        """
        self._dxl_bus.set_goal_velocities(joint_velocities, unit)

    def read_joint_velocities(self, unit=Unit.RAD_S):
        """
        @brief Read current joint velocities ordered as the kinematic chain expects.

        @param unit Unit Desired output unit (default: rad/s).
        @return List[float] Joint velocities in the requested unit.
        """
        return self._dxl_bus.read_joint_velocities(unit)        

    def set_velocity_control_mode(self):
        """
        @brief Switch the first 5 joints to velocity control mode.

        @note Torque is briefly disabled and re-enabled, which can cause a small positional shift.
        """
        self._dxl_bus.set_velocity_control_mode()
    
    def set_epcm_control_mode(self):
        """
        @brief Switch the first 5 joints to extended position control mode.

        @note Torque is briefly disabled and re-enabled, which can cause a small positional shift;
              Profile velocity/acceleration are re-applied to default after mode change.
        """
        self._dxl_bus.set_epcm_control_mode()

    # TODO move logic to the dxl_bus, here only API-Call
    def _apply_dh_q_offsets(self):
        """
        @brief Add DH joint offsets to the motors' physical home positions.

        @note This aligns the encoder's logical zero with the DH model's q-offsets.
        """
        for i, dh_joint in enumerate(self.kinematics_model.robot_cfg.dh_joints):
            self._dxl_bus.motors[i].physical_home_position += to_dxl_units(dh_joint.q_offset, from_unit=Unit.RAD)

    # TODO move to Kinematics-Class, here only API-Call    
    def _create_transform_from_xyz_psi_phi(self, x: float, y: float, z: float, psi: float, phi: float) -> SE3:
        """
        @brief Build an SE3 transform from Cartesian coordinates and two orientation angles.

        Constructs orthonormal axes with Z aligned by `psi` about the global Z, then rotates the
        local XY frame by `phi` about the intermediate Y to form the final rotation.

        @param x float Position X in meters.
        @param y float Position Y in meters.
        @param z float Position Z in meters.
        @param psi float Elevation of the end-effector Z-axis above the horizontal plane (radians).
        @param phi float Rotation about the end-effector Z-axis (radians).
        @return SE3 Homogeneous transform in the base frame.
        @note Assumption: `x == 0` implies `theta1 = 0` for the base yaw computation.
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