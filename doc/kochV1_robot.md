# KochV1 Robot — Module Documentation

A high-level interface to the KochV1 arm built on a [Dynamixel Bus](dxl_bus.md). Kinematic details and math background are covered in [KochV1-Kinematics](kochV1_kinematics.md).

---

## Contents

* [1) Quick Start](#1-quick-start)
* [2) `KochV1_Robot` API](#2-kochv1_robot-api)
* [3) Units & Conversions](#3-units--conversions)
* [4) Usage Examples](#4-usage-examples)
* [5) Notes & Gotchas](#5-notes--gotchas)

---

## 1) Quick Start

```python
import time
from dynamixel.utils import Unit
from kochV1_pkg import KochV1_Robot, KochV1_DxlBus

# Physical home position of the motors in the assembly
motor_physical_home_positions = [0, 0, 0, 0, 0, 0]

# Recommended: use the bus as a context manager
with KochV1_DxlBus(motor_physical_home_positions) as dxl_bus:
    robot = KochV1_Robot(dxl_bus)

    # Move to a joint configuration (degrees shown for readability)
    robot.set_joints([0, 90, 0, 0, 0], unit=Unit.DEG)
    time.sleep(3)  # allow time to reach goal position

    # Read back the current configuration
    print(robot.read_joints(unit=Unit.DEG))
```

---

## 2) `KochV1_Robot` API

A convenience façade that bridges the kinematics model and the Dynamixel bus. Methods raise standard Python exceptions on invalid input, unreachable IK targets, or bus/SDK errors.

### Constructor

```python
KochV1_Robot(dxl_bus: KochV1_DxlBus)
```

* **dxl_bus**: initialized `KochV1_DxlBus` instance.

---

### `set_joints`

```python
set_joints(joint_angles: list[float], unit: Unit = Unit.RAD) -> None
```

Set the goal joint configuration. Angles are converted to bus units and sent to the drivers.

* **Args:** `joint_angles` (length = DOF), `unit` (see Units).
* **Notes:** list length must match the arm’s DOF.

---

### `read_joints`

```python
read_joints(unit: Unit = Unit.RAD) -> list[float]
```

Read the latest measured joint angles.

* **Args:** `unit` (see Units).
* **Returns:** list of angles in the requested unit.

---

### `set_gripper_position_from_transform`

```python
set_gripper_position_from_transform(transform: SE3) -> None
```

Solve IK for the given pose and apply the resulting joint configuration.

* **Args:** `transform` — `SE3` pose (`R`, `t`).
* **Errors:** may raise if target is unreachable or violates limits.

---

### `set_gripper_position_from_xyz_psi_phi`

```python
set_gripper_position_from_xyz_psi_phi(
    x: float, y: float, z: float, psi: float, phi: float
) -> None
```

Construct a target pose from Cartesian position and orientation `(ψ, φ)` in **radians**, then solve IK and move.

* **Args:** `x, y, z` (meters), `psi, phi` (radians).
* **Errors:** may raise if target is unreachable or violates limits.

---

### `get_gripper_transform`

```python
get_gripper_transform() -> SE3
```

Compute FK from the latest measured joints and return the current end-effector pose.

* **Returns:** `SE3` pose of the EE.

---

### `set_gripper_percentage`

```python
set_gripper_percentage(percentage: float) -> None
```

Set the gripper closure percentage.

* **Args:** `percentage` — `0.0` = fully open; `1.0` = fully closed.

---

### `set_goal_velocities`
> Commands require velocity control mode. Call `set_velocity_control_mode()` first to avoid command rejection or undefined behavior.

```python
set_goal_velocities(joint_velocities: list[float]) -> None
```

Command joint velocities (rad/s) for joints 1–5 in **Velocity Control** mode.

* **Args:** `joint_velocities` — rad/s for the first five joints.
* **Notes:** The Dynamixel SDK requires non-zero profile acceleration; internally:
  `profile_acceleration = int(goal_velocity / 2) + 1` (DXL units).

---
### `set_velocity_control_mode`

```python
set_velocity_control_mode() -> None
```

Switches motors to velocity mode; torque is briefly disabled and re-enabled, which can cause a small positional shift. 

---

### `set_epcm_control_mode`

```python
set_velocity_control_mode() -> None
```

Switches motors to position control mode; torque is briefly disabled and re-enabled, which can cause a small positional shift.
> Position control mode is enabled by default immediately after robot initialization. If the control mode is changed at runtime, you must re-enable it before issuing position-based commands

## 3) Units & Conversions
To specify any desired unit use [utils](utils.md)-module, that provides:
- Convertion between available angle units
- Convertion between available angular velocity units
- Packing and unpacking raw integer values
---

## 4) Usage Examples

> Minimal snippets; adapt motor home positions to your setup.

### 4.1 Build the robot and query FK/IK

```python
from time import sleep
from spatialmath import SE3
from dynamixel.utils import Unit
from kochV1_pkg import KochV1_Robot, KochV1_DxlBus

with KochV1_DxlBus() as dxl_bus:
    robot = KochV1_Robot(dxl_bus)

    robot.set_joints([0, 90, 0, 0, 0], unit=Unit.DEG)
    sleep(3)

    # Current joint state (deg)
    print(robot.read_joints(unit=Unit.DEG))

    # Current EE pose
    T_current: SE3 = robot.get_gripper_transform()

    # Move to a target pose specified by xyz + (ψ, φ) orientation (radians)
    robot.set_gripper_position_from_xyz_psi_phi(
        x=0.18, y=0.05, z=0.12, psi=0.3, phi=-0.2
    )
```

### 4.2 Gripper control

```python
# open
robot.set_gripper_percentage(0.3)
sleep(0.5)

# close
robot.set_gripper_percentage(0.8)
sleep(0.5)
```

### 4.3 Velocity control (rad/s)

> Invoking this switches joints 1–5 to **Velocity Control** mode.

```python
robot.set_goal_velocities([0.5, 0.2, -0.1, 0.0, 0.0])
```

---

## 5) Notes & Gotchas

* **Units** — Positions default to radians; velocities are in rad/s. Be explicit with `unit=...` when setting or reading angles.
* **IK reachability** — `set_gripper_position_*` can fail if the target is outside the workspace or violates joint limits.
* **Timing** — Motions are non-blocking. If a settled state is required, add a delay (`sleep`) or poll until within tolerance.
* **Velocity mode behavior** — Switching to velocity mode briefly toggles torque; ensure clearance for small pose changes.

---
[Back to Overview](README.md)
