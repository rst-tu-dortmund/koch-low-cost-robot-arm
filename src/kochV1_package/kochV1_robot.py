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

        ## Kinematics model for the Koch V1.1 robot
        self.kinematics_model = KochV1_KinematicsModel()

        ## Dynamixel bus interface
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

        transform = self.kinematics_model.create_transform_from_xyz_psi_phi(x, y, z, psi, phi)

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
        @param unit Unit of the provided velocities (default: rad/s).
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

    def _apply_dh_q_offsets(self):
        """
        @brief Add DH joint offsets to the motors' physical home positions.

        @note This aligns the encoder's logical zero with the DH model's q-offsets.
        """
        q_offsets = [joint.q_offset for joint in self.kinematics_model.robot_cfg.dh_joints]
        self._dxl_bus.apply_dh_q_offsets(q_offsets)