# Dynamixel Python Wrapper – **Koch V1.1**

*A high-level facade around the Robotis **Dynamixel-SDK** tailored to the Koch V1.1 manipulator (5 joints + gripper).*

---

## Overview

`dynamixel`-package wraps the low-level C-based **Dynamixel-SDK** in a pythonic facade [`DxlBus`](dxl_bus.md). 
`KochV1_DxlBus` - a `DxlBus` designed specifically for Koch V1.1 robot-arm.


The main Features are:
* **Discover, configure and drive** all servos through a single serial port
* Predefined model-specific control tables for **XL-430-W250** and **XL-330-M288**
* Optimized command sending and reading using `SyncGroup` objects.
* Expose convenience helpers
  * `set_joints(List[float], unit=Unit.RAD|DEG|DXL)`
  * `read_joints(unit=…)`
  * `set_gripper_percentage(float)`
  * ...

---


## Further Reading

* Robotis e-Manual – [XL-430-W250](https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/) · [XL-330-M288](https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/)
* [Dynamixel-SDK Python examples](https://github.com/ROBOTIS-GIT/DynamixelSDK/tree/master/python)
* [Understanding Motor Control Modes](understanding_motor_control_modes.md)
* [Motor calibration](motor_calibration.md) for **Koch V1.1**
---

## API Reference
* [`motor_specs`](motor_specs.md)
* [`DxlBus`](dxl_bus.md)
* [`KochV1_DxlBus`](kochV1_DxlBus.md)
---
[Back to Overview](README.md)