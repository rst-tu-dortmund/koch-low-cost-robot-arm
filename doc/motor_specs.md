# Dynamixel XL‑Series Specifications — Module Documentation

This document explains the structure and usage of predefined specifications for Dynamixel motors.
It’s intended to be read alongside the official [ROBOTIS e‑manuals](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/)

---

## 1) High‑Level Overview

This package models **Dynamixel XL‑series** servos as lightweight Python types that describe their **Control Tables** and **Operating Modes**. Typical usage is to import these classes into the bus/driver layer and use their metadata (addresses, sizes, enums) to generate read/write operations.

Key ideas:

* **ControlField** – a small, immutable descriptor for a register in the control table (`address`, `size`, and an optional `initial_value`).

* **Model namespaces** – each model defines two namespaces (`EEPROM` and `RAM`) composed of `ControlField`s. These classes are never instantiated; they act as readable structures.

* **Model dataclasses** – `DynamixelXL330_M288` and `DynamixelXL430_W250` hold per‑device info (e.g., id, position limits) and reference their control‑table namespaces.

* **OperatingMode enums** – enumerate the values expected by the device’s `OPERATING_MODE` register.

---

## 2) Modules & Public API

### `base.py`

Core typing protocols and control-table data structures for Dynamixel models:

* `ControlField` – immutable metadata for a **single** control‑table entry.
* `DynamixelMotor` – a **typing protocol** that captures motor metadata and per-instance unique values.

#### `ControlField`

A frozen dataclass describing one control‑table entry.

| Attribute       | Type            | Meaning                                   |
| --------------- | --------------- | ----------------------------------------- |
| `address`       | `int`           | Register address within the control table |
| `size`          | `int`           | Field size in bytes                       |
| `initial_value` | `Optional[int]` | Default value if applicable               |

> **Note**: `ControlField` is metadata only; it performs no I/O.

#### `DynamixelMotor` (Protocol)

Concrete motor classes expose:

* `EEPROM` and `RAM`: classes containing `ControlField` attributes
* `OperatingModeType`: `IntEnum` enumerating supported operating modes (values are **register values**)
* `id`: motor ID (instance attribute)

---

### `XL330_M288.py`

#### `XL330_M288OperatingModeType` (`IntEnum`)

Supported modes and their **register** values:

* `VELOCITY_CONTROL_MODE = 1`
* `POSITION_CONTROL_MODE = 3`
* `EXTENDED_POSITION_CONTROL_MODE = 4`
* `CURRENT_BASED_POSITION_CONTROL_MODE = 5`
* `PWM_CONTROL_MODE = 16`

#### `XL330_M288EEPROMControlTable`

Namespace of EEPROM registers. Examples:

* `OPERATING_MODE = ControlField(11, 1, 3)`
* `VELOCITY_LIMIT = ControlField(44, 4, 445)`
* `MAX_POSITION_LIMIT = ControlField(48, 4, 4095)`

#### `XL330_M288RAMControlTable`

Namespace of RAM registers. Examples:

* `TORQUE_ENABLE = ControlField(64, 1, 0)`
* `GOAL_POSITION = ControlField(116, 4, None)`
* `PRESENT_POSITION = ControlField(132, 4, None)`

#### `DynamixelXL330_M288` (dataclass)

Per‑instance class for a Dynamixel **XL330‑M288** motor.

* `id: int`
* `physical_home_position: int`
* `physical_position_limits: np.ndarray[int, int]` → absolute `[min, max]` encoder-ticks

**Class attributes**: `EEPROM`, `RAM`, `FULL_ROTATION_TICKS`, `OperatingModeType`.

**Example (instantiation & usage):**

```py
from XL330_M288 import DynamixelXL330_M288
import numpy as np

motor = DynamixelXL330_M288(
    1,
    2048,
    np.array([1024, 3072], dtype=int),
)

addr = motor.RAM.GOAL_POSITION.address  # 116
size = motor.RAM.GOAL_POSITION.size     # 4
```

---

### `XL430_W250.py`

Same structure and behavior as `XL330_M288.py`; only the register defaults and available modes differ.

#### `XL430_W250OperatingModeType` (`IntEnum`)

* `VELOCITY_CONTROL_MODE = 1`
* `POSITION_CONTROL_MODE = 3`
* `EXTENDED_POSITION_CONTROL_MODE = 4`
* `PWM_CONTROL_MODE = 16`

**Example (instantiation):**

```py
from XL430_W250 import DynamixelXL430_W250
import numpy as np

motor = DynamixelXL430_W250(
    1,
    2048,
    np.array([1024, 3072], dtype=int),
)
```
---
[Back to Overview](README.md)