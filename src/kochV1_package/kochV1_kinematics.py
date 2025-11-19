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

from __future__ import annotations

"""
@brief Kinematics utilities for the robot arm Koch V1.1.

Provides DH-based forward/inverse kinematics and Jacobian computation for a simple serial manipulator.
@see KochV1_KinematicsModel for the implementation of forward and inverse kinematics.
"""

import numpy as np
import sympy as sp

from dataclasses import dataclass, field
from enum import Enum

from spatialmath import SE3

from typing import (
    Tuple,
    List,
    Callable
)

from utils import (
    check_limits
)

class JointType(Enum):
    """
    @brief Joint actuation type.

    Distinguishes between revolute and prismatic joints.
    """
    REVOLUTE = 1
    PRISMATIC = 2


# d will be ignored for prismatic joint and 
# theta will be ignored for revolute joint
@dataclass(frozen=True, slots=True)
class DHJoint:
    """
    @brief Denavit-Hartenberg joint/link parameters.

    Stores standard (a, alpha, d, theta) along with joint limits and offsets. For revolute joints `theta` is the
    variable; for prismatic joints `d` is the variable.

    @note Angles are in radians; lengths are in meters.
    """
    type: JointType                 = JointType.REVOLUTE
    name: str                       = ""
    a: float                        = 0.0                   # link length (m)
    alpha: float                    = 0.0                   # link twist (rad)
    d: float                        = 0.0                   # link offset (m)
    theta: float                    = 0.0                   # joint angle (rad)
    q_limits: Tuple[float, float]   = (-np.inf, np.inf)     # joint limits (rad or m)
    q_offset: float                 = 0.0                   # joint offset (rad or m)

    def get_transform(self, q: float = 0.0) -> SE3:
        """
        @brief Compute the homogeneous transform of this joint for a given configuration.

        Uses the standard DH convention. For revolute joints `q` adds to `theta`; for prismatic joints `q` adds to `d`.

        @param q float Joint variable (rad for revolute, m for prismatic).
        @return SE3 Transform from the previous link frame to this link frame.
        """

        # Select DH parameters depending on joint type
        # REVOLUTE
        if self.type is JointType.REVOLUTE:
            theta = self.q_offset + q
            d     = self.d
        
        # PRISMATIC
        else:
            theta = self.theta
            d     = self.q_offset + q

        # see https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters
        cos_alpha, sin_alpha = np.cos(self.alpha),  np.sin(self.alpha)
        cos_theta, sin_theta = np.cos(theta),       np.sin(theta)

        T = np.array([
            [ cos_theta, -sin_theta*cos_alpha,  sin_theta*sin_alpha, self.a*cos_theta],
            [ sin_theta,  cos_theta*cos_alpha, -cos_theta*sin_alpha, self.a*sin_theta],
            [     0,           sin_alpha,            cos_alpha,             d        ],
            [     0,               0,                    0,                 1        ],
        ])

        return SE3(T)


@dataclass(frozen=True, slots=True)
class RobotConfig:
    """
    @brief Immutable container for a robot's DH chain.

    @param dh_joints Tuple[DHJoint] Ordered DH joints from base to end-effector.
    """
    dh_joints: Tuple[DHJoint]

    @staticmethod
    def from_DH_parameters(dh_parameters: List[DHJoint]) -> 'RobotConfig':
        """
        @brief Build a configuration from a list of DH joints.

        @param dh_parameters list[DHJoint] Joints in base-to-tip order.
        @return RobotConfig Frozen configuration.
        """
        return RobotConfig(dh_joints=tuple(dh_parameters))

    @staticmethod
    def from_URDF(urdf_path: str) -> 'RobotConfig':
        pass


class KochV1_KinematicsModel:
    """
    @brief Kinematics model (FK, IK, Jacobian) for the robot arm Koch V1.1.

    Defines a 5-DOF DH chain;
    Provides analytical FK/IK and symbolic Jacobian compiled to NumPy.
    """
    def __init__(self):
        """
        @brief Initialize the model with fixed DH parameters and compile the Jacobian function.
        """
        joints = [
            DHJoint(type=JointType.REVOLUTE, a=0,       alpha=np.pi/2, d=0.0563,   q_offset=0,                   q_limits=(-np.pi / 2, np.pi / 2)),
            DHJoint(type=JointType.REVOLUTE, a=0.10931, alpha=0,       d=0,        q_offset=-7.78 * (np.pi/180), q_limits=(-8*np.pi/180, 125*np.pi/180)),
            DHJoint(type=JointType.REVOLUTE, a=0.10051, alpha=0,       d=0,        q_offset= 9.32 * (np.pi/180), q_limits=(-170*np.pi/180, 15*np.pi/180)),
            DHJoint(type=JointType.REVOLUTE, a=7e-6,    alpha=np.pi/2, d=0.953e-3, q_offset=88.46 * (np.pi/180), q_limits=(-np.pi/2, np.pi/2)),
            DHJoint(type=JointType.REVOLUTE, a=0,       alpha=0,       d=0.0681,   q_offset=0,                   q_limits=(-np.pi, np.pi)),
        ]

        self._robot_cfg = RobotConfig.from_DH_parameters(joints)

        self._compute_jacobian_func = self._init_compute_jacobian_func()

    @property
    def robot_cfg(self) -> RobotConfig:
        return self._robot_cfg

    def compute_forward_kinematics(self, joint_angles: List[float]) -> SE3:
        """
        @brief Compute end-effector pose from joint angles.

        Multiplies per-link DH transforms in order.

        @param joint_angles list[float] Joint values (rad) for the 5 DOF.
        @return SE3 End-effector pose in the base frame.
        """

        T_ee = SE3()

        for dh_joint, q in zip(self.robot_cfg.dh_joints, joint_angles):
            T_ee = T_ee * dh_joint.get_transform(q)

        return T_ee
    
    def compute_inverse_kinematics(self, T_ee: SE3) -> List[float] | None:
        """
        @brief Compute joint angles that realize the desired end-effector pose.

        Solves an analytical (geometric) IK, evaluates elbow-up/down branches and joint limits. 
        Returns the feasible solution closest in orientation to the current transform.

        @param T_ee SE3 Desired end-effector pose in the base frame.
        @return list[float] Joint angles (rad) if solvable; otherwise None.
        """
        l_0 = self.robot_cfg.dh_joints[0].d
        l_1 = self.robot_cfg.dh_joints[1].a
        l_2 = self.robot_cfg.dh_joints[2].a
        l_3 = self.robot_cfg.dh_joints[4].d

        # Desired Position:
        (x_des, y_des, z_des) = T_ee.t
        psi = np.asin(T_ee.A[2, 2])       

        # Theta 1 ---------------------------------------------------------------------
        theta_1 = np.atan2(y_des, x_des) if x_des != 0 else 0
        if not check_limits(theta_1, self.robot_cfg.dh_joints[0].q_limits):
            return None
        
        # Transform to y'-O_1-x'
        x_5 = np.sqrt(x_des**2 + y_des**2)
        y_5 = z_des - l_0

        # Transform to 2D-Planar:
        x_3 = x_5 - (l_3*np.cos(psi)) 
        y_3 = y_5 - (l_3*np.sin(psi)) 

        d = np.sqrt(x_3**2 + y_3**2)

        # Theta 2, Theta 3 ------------------------------------------------------------
        alpha = np.acos((l_1**2 + l_2**2 - d**2) / (2*l_1*l_2))
        beta = np.acos((l_1**2 + d**2 - l_2**2) / (2*l_1*d))

        # elbow down solution ---------------------------------------------------------
        theta_2_elbow_down = np.atan2(y_3, x_3) - beta
        theta_3_elbow_down = np.pi - alpha

        theta_4_elbow_down = psi - (theta_2_elbow_down + theta_3_elbow_down)

        theta_5_elbow_down = self._calculate_theta_5(
            theta_1, theta_2_elbow_down, theta_3_elbow_down, theta_4_elbow_down, T_ee
        )

        success_elbow_down = all([
            check_limits(theta_2_elbow_down, self.robot_cfg.dh_joints[1].q_limits),
            check_limits(theta_3_elbow_down, self.robot_cfg.dh_joints[2].q_limits),
            check_limits(theta_4_elbow_down, self.robot_cfg.dh_joints[3].q_limits),
            check_limits(theta_5_elbow_down, self.robot_cfg.dh_joints[4].q_limits),
        ])

        # elbow up solution -----------------------------------------------------------
        theta_2_elbow_up = np.atan2(y_3, x_3) + beta
        theta_3_elbow_up = -(np.pi - alpha)

        theta_4_elbow_up = psi - (theta_2_elbow_up + theta_3_elbow_up)

        theta_5_elbow_up = self._calculate_theta_5(
            theta_1, theta_2_elbow_up, theta_3_elbow_up, theta_4_elbow_up, T_ee
        )

        success_elbow_up = all([
            check_limits(theta_2_elbow_up, self.robot_cfg.dh_joints[1].q_limits),
            check_limits(theta_3_elbow_up, self.robot_cfg.dh_joints[2].q_limits),
            check_limits(theta_4_elbow_up, self.robot_cfg.dh_joints[3].q_limits),
            check_limits(theta_5_elbow_up, self.robot_cfg.dh_joints[4].q_limits),
        ])

        # Candidate Selection ---------------------------------------------------------
        # select candidate closer to current joint state
        if not success_elbow_down and not success_elbow_up:
            return None
        
        elif not success_elbow_down and success_elbow_up:
            return [theta_1, theta_2_elbow_up, theta_3_elbow_up, theta_4_elbow_up, theta_5_elbow_up]
        
        elif success_elbow_down and not success_elbow_up:
            return [theta_1, theta_2_elbow_down, theta_3_elbow_down, theta_4_elbow_down, theta_5_elbow_down]
        
        # else
        elif success_elbow_down and success_elbow_up:
            # compare quaternion distances to goal and select closer one

            fwd_elbow_down = self.compute_forward_kinematics(
                [theta_1, theta_2_elbow_down, theta_3_elbow_down, theta_4_elbow_down, theta_5_elbow_down]
            )
            fwd_elbow_up = self.compute_forward_kinematics(
                [theta_1, theta_2_elbow_up, theta_3_elbow_up, theta_4_elbow_up, theta_5_elbow_up]
            )

            goal_quat = T_ee.UnitQuaternion()
            elbow_down_quat = fwd_elbow_down.UnitQuaternion()
            elbow_up_quat = fwd_elbow_up.UnitQuaternion()

            # see http://math.stackexchange.com/questions/90081/quaternion-distance
            dist_elbow_down = 1 - float(np.dot(goal_quat.vec, elbow_down_quat.vec))**2
            dist_elbow_up   = 1 - float(np.dot(goal_quat.vec, elbow_up_quat.vec))**2

            if abs(dist_elbow_down) < abs(dist_elbow_up):
                return [theta_1, theta_2_elbow_down, theta_3_elbow_down, theta_4_elbow_down, theta_5_elbow_down]
            else:
                return [theta_1, theta_2_elbow_up, theta_3_elbow_up, theta_4_elbow_up, theta_5_elbow_up]


    
    def _calculate_theta_5(self, theta_1, theta_2, theta_3, theta_4, T_ee) -> float:
        """
        @brief Compute the final wrist rotation to align end-effector X-axes.

        Uses the angle between current and desired X-axis with the desired Z-axis as rotation axis.

        @param theta_1 float Joint 1 angle (rad).
        @param theta_2 float Joint 2 angle (rad).
        @param theta_3 float Joint 3 angle (rad).
        @param theta_4 float Joint 4 angle (rad).
        @param T_ee SE3 Desired end-effector pose.
        @return float Angle for joint 5 in radians.
        """
        T_before = self.compute_forward_kinematics([theta_1, theta_2, theta_3, theta_4, 0])
        T_after = T_ee

        x_before = T_before.R[:, 0]
        x_after = T_after.R[:, 0]

        z_local = T_after.R[:, 2]  # end-effector's local Z-axis, same for both poses

        cos_theta_5 = np.dot(x_before, x_after)
        sin_theta_5 = np.dot(z_local, np.cross(x_before, x_after))   # vectors are normalized
        
        theta_5 = np.arctan2(sin_theta_5, cos_theta_5)
        return theta_5
    
    def compute_jacobian(self, joint_angles: List[float]) -> np.ndarray:
        """
        @brief Compute the 6×n geometric Jacobian for given joint configuration.

        @param joint_angles list[float] Joint angles (rad).
        @return np.ndarray Geometric Jacobian matrix for the current joint state.
        """
        return np.asarray(self._compute_jacobian_func(*joint_angles), dtype=float)
    
    def _init_compute_jacobian_func(self) -> Callable:
        """
        @brief Build a fast NumPy Jacobian function from a symbolic expression.

        @return Callable Function mapping (q1, …, qn) -> 6×n Jacobian as ndarray-like.
        """
        J_sp, q_sp = self._build_sympy_jacobian()

        J_func = sp.lambdify(q_sp, J_sp, modules="numpy")

        return J_func
    
    def _build_sympy_jacobian(self):
        """
        @brief Build the symbolic geometric Jacobian J(q) using SymPy.

        Constructs forward kinematics using DH matrices, collects frame origins and z-axes, and assembles the
        translational and rotational Jacobian blocks.

        @return tuple[sp.Matrix, tuple] Symbolic Jacobian and the q-symbols.
        """

        # helper function to create a symbolic DH transformation matrix
        def A_sym(a, alpha, d, theta) -> sp.Matrix:
            ca, sa = sp.cos(alpha), sp.sin(alpha)
            ct, st = sp.cos(theta), sp.sin(theta)
            
            return sp.Matrix([
                [ ct, -st*ca,  st*sa, a*ct],
                [ st,  ct*ca, -ct*sa, a*st],
                [  0,     sa,     ca,   d ],
                [  0,      0,      0,   1 ]
            ])
        
        # build J(q)
        n = len(self.robot_cfg.dh_joints)
        q  = sp.symbols(f'q1:{n+1}')          # (q1, q2, …, qn)

        T   = sp.eye(4)
        o   = [sp.Matrix([0, 0, 0])]          # origin of frame 0
        z   = [sp.Matrix([0, 0, 1])]          # z-axis of frame 0

        # Forward kinematics:
        for i, joint in enumerate(self.robot_cfg.dh_joints):
            if joint.type is JointType.REVOLUTE:
                theta = q[i] + joint.q_offset
                d     = joint.d
            else:  # PRISMATIC
                theta = joint.theta
                d     = q[i] + joint.q_offset

            T  = T * A_sym(joint.a, joint.alpha, d, theta)
            o.append(T[:3, 3])
            z.append(T[:3, 2])

        o_n = o[-1]                            # end-effector origin

        # Assemble Jacobian column by column
        Jv, Jw = [], []
        for i, joint in enumerate(self.robot_cfg.dh_joints):
            if joint.type is JointType.REVOLUTE:
                Jv.append(z[i].cross(o_n - o[i]))
                Jw.append(z[i])
            else:  # PRISMATIC
                Jv.append(z[i])
                Jw.append(sp.zeros(3, 1))

        J = sp.Matrix.hstack(*Jv).col_join(sp.Matrix.hstack(*Jw))
        return J, q