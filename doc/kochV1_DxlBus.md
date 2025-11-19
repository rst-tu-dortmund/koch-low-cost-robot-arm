# `kochV1_dxl_bus.py` — `KochV1_DxlBus` Documentation

A thin, ergonomic layer over the base `DxlBus` tailored to the KochV1 Robotarm. It:

* manages six Dynamixel XL-series motors (IDs 1–6),
* creates the default sync groups for joint controls,
* exposes high-level helpers for joints, gripper, temperatures, and operating modes,
* handles torque and physical home offseting.

Uses the project’s motor spec types (`ControlField`, `DynamixelMotor`) and unit helpers from [`utils`](utils.md)-module.

---

## Contents

* [Types & Motor Layout](#types--motor-layout)
* [`KochV1_DxlBus(DxlBus)` API](#kochv1_dxlbusdxlbus-api)
* [Methods](#methods)
* [Usage Example](#usage-example)

---

## Types & Motor Layout

* **Motors** (created in `__init__`):

  * IDs **1–2**: `DynamixelXL430_W250` (base/shoulder)
  * IDs **3–5**: `DynamixelXL330_M288` (elbow/wrist)
  * ID **6**: `DynamixelXL330_M288` (gripper)

Each motor is initialized with:

* `id`,
* `physical_home_position` (DXL ticks),

---

## `KochV1_DxlBus(DxlBus)` API

### Constructor
```python
dxl_bus = KochV1_DxlBus(
    physical_home_positions: List[int], 
    port_name: str = "",
    baudrate: int = 1_000_000, 
    protocol_version: float = 2.0
    )
```
* **Args** `physical_home_positions` - unique for assembly physical home positions in **DXL-unit** for each motor. Remaining args are derived from the `DxlBus`


### Context management
`KochV1_DxlBus` extends [`DxlBus`](dxl_bus.md), so it is recommended to use a context **context-manager**

```python
with KochV1_DxlBus(...) as bus:
    ...
```

* **Enter:** connects the port, initializes each motor’s **physical home** (see below), then **enables torque** on all motors.
* **Exit:** **disables torque** on all motors and disconnects the port. If an exception occurred inside the `with` block, it is re-raised after clean-up.

---

### Sync Groups (defaults)

Created by `make_default_sync_groups()`:

* `"joint_angles_write"` -> `GOAL_POSITION` for IDs `[1,2,3,4,5]`
* `"joint_angles_read"`  -> `PRESENT_POSITION` for IDs `[1,2,3,4,5]`
* `"goal_velocities"`    -> `GOAL_VELOCITY` for IDs `[1,2,3,4,5]`

> Motor 6 (gripper) is not part of the joint groups. Gripper position is handled individually.

---

### Methods:

#### `set_joints`

```python
set_joints(joint_angles: list[float], unit: Unit = Unit.RAD) -> None
```

Write **goal positions** for joints 1–5 via the sync group.

* Converts each `angle` from `unit` → DXL ticks, then offsets by the motor’s `physical_home_position`.
* Sends a single sync-write to `"joint_angles_write"`.

**Args:**
`joint_angles` — length **5**; `unit` — e.g., `Unit.RAD` or `Unit.DEG`.

**Errors:**
`ValueError` if length ≠ 5. Underlying bus may raise on SDK/device errors.

---

#### `read_joints`

```python
read_joints(unit: Unit = Unit.RAD) -> list[float]
```

Read **present positions** for joints 1–5 via the sync group.

* Values are unpacked, offset **back** by `physical_home_position`, then converted from DXL ticks → `unit`.

**Args:**
`unit` — requested output unit.

**Returns:**
list of 5 joint angles.

---

#### `set_gripper_percentage`

```python
set_gripper_percentage(percentage: float) -> None
```

Open/close the gripper (motor ID 6) using a normalized **closure** value.

* Computes a safe range from the gripper’s `physical_position_limits` with a ±100 tick margin.
* Maps `percentage ∈ [0,1]` to that range; assembly requires an inverted direction (handled internally).
* Writes the resulting `GOAL_POSITION` in **DXL ticks**.

**Args:**
`percentage` — `0.0` = fully open, `1.0` = fully closed (per assembly; see margin note).

---

#### `set_goal_velocities`

```python
set_goal_velocities(joint_velocities: list[float]) -> None
```

Write **goal velocities** for joints 1–5.

* Iterates over IDs 1–5 and writes `GOAL_VELOCITY` per motor.
* Values are expected in raw **DXL units**; they are passed through `to_u32(...)`.

> TODO in code: replace per-motor writes with a sync group.

**Args:**
`joint_velocities` — length **5**, raw DXL units.

**Errors:**
`ValueError` if length ≠ 5.

---

#### `set_velocity_control_mode`

```python
set_velocity_control_mode() -> None
```

Set IDs 1–5 to **Velocity Control** mode (XL-series). Internally calls `set_operating_mode(...)` per motor.

---

#### `set_epcm_control_mode`

```python
set_epcm_control_mode() -> None
```

Set IDs 1–5 to **Extended Position Control** mode (EPCM). Internally calls `set_operating_mode(...)` per motor.

---

#### `set_operating_mode`

```python
set_operating_mode(
    motor: DynamixelXL330_M288 | DynamixelXL430_W250,
    operating_mode: XL330_M288OperatingModeType | XL430_W250OperatingModeType
) -> None
```

Set a motor’s operating mode safely.

* **Disables torque**, writes `EEPROM.OPERATING_MODE`, then **enables torque**.

**Safety:**
Torque toggling can cause small pose shifts; ensure clearance.

---

## Notes & Safety

* **Torque toggling**: mode changes disable/enable torque; expect small pose shift.

---

## Usage Example

```python
import time
import numpy as np
from dynamixel.utils import Unit
from kochV1_pkg.dxl import KochV1_DxlBus

physical_home_positions = [
    2048,
    2048,
    2048,
    2048,
    2048,
    2048,
]

with KochV1_DxlBus(physical_home_positions) as bus:
    # Move joints (degrees for readability)
    bus.set_joints([0, 45, -20, 15, 0], unit=Unit.DEG)
    time.sleep(3.0)

    # Read back joints (radians)
    q = bus.read_joints(unit=Unit.RAD)
    print("joints:", np.round(q, 3))

    # Gripper: 0.0 open … 1.0 closed
    bus.set_gripper_percentage(0.7)

    # Switch to velocity control, then command velocities (DXL units)
    bus.set_velocity_control_mode()
    bus.set_goal_velocities([50, 30, 20, 0, 0])
```
---
[Back to Overview](README.md)