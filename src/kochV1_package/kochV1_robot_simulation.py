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

import os
import mujoco
import numpy as np

import logging
import mujoco

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
"""
@brief High-level robot control interface for Mujoco simulation of the robot arm Koch V1.1. We rely on the simulation model of the previous version https://github.com/AlexanderKoch-Koch/low_cost_robot. The control is adapted to the structure of the KochV1_Robot class.

Provides thin orchestration over a `Mujoco Data` and the kinematics model to set/read joint states,
move the end-effector by pose or task-space parameters, and configure control modes.

@see KochV1_KinematicsModel for forward/inverse kinematics.
"""

class KochV1_Robot_Simulation:
    """
    @brief High-level interface for commanding the simulated Koch V1.1 robot arm.

    Exposes convenience methods to set joint angles/velocities, compute and execute IK targets, and
    read back the joint configuration and end-effector transform.
    """

    def __init__(self, path: str = "kochV1_package/simulation/low_cost_robot/scene.xml"):
        """
        @brief Construct the robot interface.

        @param path str Path to the Mujoco XML model file.
        """
        
        ## Mujoco model for simulation
        self.mjModel = mujoco.MjModel.from_xml_path(os.path.join(os.getcwd(), "src", path))
        ## Mujoco data for simulation
        self.mjData = mujoco.MjData(self.mjModel)
        ## Kinematics model for the Koch V1.1 robot
        self.kinematics_model = KochV1_KinematicsModel()

        # self._apply_dh_q_offsets()
        logging.info("Robot is ready")

    def set_joints(self, joint_angles: List[float], unit: Unit = Unit.RAD):
        """
        @brief Set the 5 joint angles.

        @param joint_angles List[float] Target joint angles ordered as the kinematic chain expects.
        @param unit Unit Unit of the provided angles (default: radians).
        """
        if unit == Unit.DEG:
            joint_angles = np.deg2rad(joint_angles)
        elif unit == Unit.RAD:
            joint_angles = np.array(joint_angles)
        else:
            raise ValueError(f"Unsupported unit for joint angles: {unit}")     
         
        self.mjData.ctrl[:5] = joint_angles 

    def read_joints(self, unit: Unit = Unit.RAD) -> List[float]:
        """
        @brief Read the current joint configuration.

        @param unit Unit Desired output unit (default: radians).
        @return List[float] Current joint angles in the requested unit ordered as the kinematic chain expects.
        """
        joint_angles = self.mjData.qpos[:5].tolist()
        
        if unit == Unit.DEG:
            joint_angles = np.rad2deg(joint_angles).tolist()    
        elif unit == Unit.RAD:
            pass  # already in radians
        else:
            raise ValueError(f"Unsupported unit for joint angles: {unit}")
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
        
        # Get grippper limits from Mujoco model
        gripper_joint_id = self.mjModel.joint("joint5").id
        lower_limit = self.mjModel.jnt_range[gripper_joint_id][0]
        upper_limit = self.mjModel.jnt_range[gripper_joint_id][1]
        
        target_position = lower_limit + percentage * (upper_limit - lower_limit)
        
        self.mjData.ctrl[4] = target_position

    def set_goal_velocities(self, joint_velocities: List[float]):
        """
        @brief Set joint velocities for the first 5 joints (velocity control). Currently in a development state. It works with high velocities but not with low velocities since the simulation errors accumulate fast.

        @param joint_velocities List[float] Target joint velocities in DXL ordered as the kinematic chain expects; 
                                values are clipped to the motor's supported range before sending.
        @note Also updates each motor's `PROFILE_ACCELERATION` based on the requested velocity.
        """
        self.mjData.ctrl[:5] = joint_velocities  # Zero position control to avoid conflict
        vel = np.array(joint_velocities)
        
        # Integrate velocity commands (previous state is rounded to avoid accumulation of small errors)
        new_positions = np.round(np.array(self.read_joints(unit=Unit.RAD)),2) + vel * self.mjModel.opt.timestep
        
        
        print(new_positions)
        self.set_joints(new_positions, unit=Unit.RAD)
            
    def read_joint_velocities(self, unit=Unit.RAD_S):
        """
        @brief Read current joint velocities ordered as the kinematic chain expects.

        @param unit Unit Desired output unit (default: rad/s).
        @return List[float] Joint velocities in the requested unit.
        """
    
        if unit == Unit.RAD_S:
            vel = self.mjData.qvel[:5].tolist()
        else:
            raise ValueError(f"Unsupported unit for joint velocities: {unit}")
        
        return vel