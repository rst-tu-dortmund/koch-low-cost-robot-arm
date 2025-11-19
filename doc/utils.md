# `utils.md` —  Unit Handling and Data Conversion Helpers

Utility functions for converting between **Dynamixel internal units** and common physical units, plus a few helpers for 32-bit packing/unpacking.

> **Assumptions**
>
> * **Position**: 4096 DXL units per full revolution.
> * **Velocity**: 1 DXL unit = **0.229 rpm**.
>
> These constants match common ROBOTIS models. If your model differs, adjust the factors in the code.

---

## Contents

* [Enum: `Unit`](#enum-unit)
* [Conversions](#conversions)

  * [`to_dxl_units(value, from_unit)`](#to_dxl_unitsvalue-from_unit--int)
  * [`from_dxl_units(value, to_unit)`](#from_dxl_unitsvalue-to_unit--float)
  * [`convert_angle(value, from_unit, to_unit)`](#convert_anglevalue-from_unit-to_unit--float)
* [Integer / Byte Helpers](#integer--byte-helpers)

  * [`to_u32(x)`](#to_u32x--int)
  * [`pack_i32_le(value)`](#pack_i32_levalue--bytes)
  * [`unpack_i32_le(raw)`](#unpack_i32_leraw--int)

---

## Enum: `Unit`

```python
from enum import Enum

class Unit(Enum):
    DXL   = 0  # raw Dynamixel ticks or velocity units
    RAD   = 1
    DEG   = 2
    RPM   = 3
    RAD_S = 4
```

**Meaning**

* `DXL` — raw register units used by the servo (encoder-ticks or velocity units).
* `RAD` — angle value in radians
* `DEG` — angle value in degrees.
* `RPM` — angular velocity value in RPM
* `RAD_S` — angular velocity value in radian per second.

---

## Conversions

- `to_dxl_units(value: float, from_unit: Unit) -> int`

    Converts a physical value to **DXL encoder values**.

- `from_dxl_units(value: int, to_unit: Unit) -> float`

    Converts **DXL encoder values** to a physical value.

- `convert_angle(value: float, from_unit: Unit, to_unit: Unit) -> float`

    Two-step angle converter via DXL ticks:

    1. `dxl = to_dxl_units(value, from_unit)`
    2. `return from_dxl_units(dxl, to_unit)`

---

## Integer / Byte Helpers

- `to_u32(x: int) -> int`
    
    Returns the **unsigned 32-bit** representation of `x`


- `pack_i32_le(value: int) -> bytes`

    Packs a **signed 32-bit integer** to **little-endian** bytes:


- `unpack_i32_le(raw: int | bytes | bytearray) -> int`

    Unpacks **little-endian** 4 bytes (or a 32-bit unsigned int) into a **signed 32-bit** Python int:
---
[Back to Overview](README.md)