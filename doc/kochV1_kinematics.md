# `kochV1_kinematics.py` — Documentation

A lightweight, mostly stateless kinematics layer that:

* computes forward kinematics (FK) from joint angles,
* solves **analytical** inverse kinematics (IK) (elbow-down / elbow-up branches),
* builds a symbolic 6×n geometric Jacobian (SymPy) and exposes a fast NumPy evaluator.

---

## Contents

* [Types](#types)

  * [`JointType`](#jointtype)
  * [`DHJoint`](#dhjoint)
  * [`RobotConfig`](#robotconfig)
* [`KochV1_KinematicsModel`](#kochv1_kinematicsmodel)
  * [Methods](#methods)

* [Usage Example](#usage-example)

---

## Types

### `JointType`

```python
class JointType(Enum):
    REVOLUTE = 1
    PRISMATIC = 2
```

Joint classification used throughout the DH chain.

---

### `DHJoint`

```python
@dataclass(frozen=True, slots=True)
class DHJoint:
    type: JointType = JointType.REVOLUTE
    name: str = ""
    a: float = 0.0                                    # link length (m)
    alpha: float = 0.0                                # link twist (rad)
    d: float = 0.0                                    # link offset (m)
    theta: float = 0.0                                # joint angle (rad)
    q_limits: Tuple[float, float] = (-np.inf, np.inf) # joint limits (rad or m)
    q_offset: float = 0.0                             # joint offset (rad or m)

    def get_transform(self, q: float = 0.0) -> SE3: ...
```

#### `get_transform` 
```python
get_transform(q: float = 0) -> SE3
```
Compute homogeneous transform using **Denavit-Hartenberg** convention.

* **Args:** `q` - joint value (rotation angle for revolute joints)
* **Notes:** For **REVOLUTE** joints `theta = q_offset + q`, `d = constant`.

---

### `RobotConfig`

```python
@dataclass(frozen=True, slots=True)
class RobotConfig:
    dh_joints: Tuple[DHJoint]

    @staticmethod
    def from_DH_parameters(dh_parameters: List[DHJoint]) -> 'RobotConfig': ...
```

Immutable wrapper for the ordered DH chain.

---

## `KochV1_KinematicsModel`

```python
model = KochV1_KinematicsModel(robot_cfg: RobotConfig)
```

`robot_cfg.dh_joints` must be an ordered list of `DHJoint`.

---

### Methods

#### `compute_forward_kinematics`

```python
compute_forward_kinematics(joint_angles: list[float]) -> SE3
```

Multiply per-joint DH transforms (respecting offsets and joint type) **left-to-right** to obtain the EE pose.

* **Args:** `joint_angles` — ordered list of joint values **in radians**.
* **Returns:** `SE3` pose of the end-effector.

---

#### `compute_inverse_kinematics`

```python
compute_inverse_kinematics(T_ee: SE3) -> list[float] | None
```

Anlytical IK-solver for, that targets the end-effector pose `T_ee`.

**Limit checking & return:**

Every solved angle is validated. If any angle violates limits (or an intermediate computation is invalid), the function returns `None`.

* **Args:** `T_ee` - `SE3` target end-effector pose.
* **Returns:** 5-element list of desired joint angles in **radians**, or `None` if solution not found.

---

#### `compute_jacobian`

```python
compute_jacobian(joint_angles: list[float]) -> np.ndarray
```

Return the **6×5** geometric Jacobian `J(q)` as a `NumPy`-matrix.

* **Args:** `joint_angles` — ordered list of joint values **in radians**.
* **Returns:** `np.ndarray` of shape `(6, 5)`.

---

## Assumptions & Conventions

* **Units:** radians for revolute joints; meters for DH lengths.
* **Frames:** standard DH; FK returns base→EE transform.
* **Offsets/Limits:** `q_offset` applied per joint; `q_limits` enforced via `check_limits`.
* **Model DOF:** the IK solves for a 5-joint chain as laid out in `robot_cfg.dh_joints`.

---

## Usage Example

```python
import numpy as np
from spatialmath import SE3
from kochV1_pkg.kinematics import KochV1_KinematicsModel, RobotConfig, DHJoint, JointType

# Define DH chain
joints = [
    DHJoint(type=JointType.REVOLUTE, a=0,   alpha=np.pi/2, d=0.05, q_offset=0),
    DHJoint(type=JointType.REVOLUTE, a=0.1, alpha=0,       d=0,    q_offset=0),
    DHJoint(type=JointType.REVOLUTE, a=0.1, alpha=0,       d=0,    q_offset=0),
    DHJoint(type=JointType.REVOLUTE, a=0,   alpha=np.pi/2, d=0,    q_offset=np.pi/2),
    DHJoint(type=JointType.REVOLUTE, a=0,   alpha=0,       d=0.06, q_offset=0),
]
cfg = RobotConfig.from_DH_parameters(joints)

# Build model
model = KochV1_KinematicsModel(cfg)

# FK
q = [0.0, 0.6, -0.4, 0.2, 0.0]
T = model.compute_forward_kinematics(q)

# IK
q_sol = model.compute_inverse_kinematics(T, elbow_down=True)
if q_sol is None:
    print("IK infeasible or violates limits")

# Jacobian
J = model.compute_jacobian(q)
```
---
[Back to Overview](README.md)