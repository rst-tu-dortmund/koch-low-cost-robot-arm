# Dynamixel Control Modes

## Overview

Dynamixel servos support multiple control modes, each optimized for different robotic applications. These modes primarily differ in their approach to interpreting internal encoder data and managing motor positions.

The following document covers two specific positioning modes in detail:

| Mode                          | Purpose                                         | Position Range                                     | Typical Use‑cases                                 |
| ----------------------------- | ----------------------------------------------- | -------------------------------------------------- | ------------------------------------------------- |
| **Position Control**          | Single-turn servo positioning                   | 0 – 360° (0 – 4095 encoder counts, one revolution) | Humanoid joints, grippers, pan‑tilt units         |
| **Extended Position Control** | Multi-turn positioning without mechanical stops | ±256 revolutions (±2,147,483,648 encoder counts)   | Wheel drives, rotary actuators, infinite pan axes |

Additional control modes (e.g., Velocity, Current, PWM, and Torque) will be added soon.

---

## Encoder Scaling & Unit Conversion

Understanding how encoder counts map to angular displacement is fundamental when configuring and controlling Dynamixel motors. Encoder counts are directly convertible to mechanical angles according to the following equation:

$\theta\,[\text{rad}] = 2\pi \frac{p}{\text{res}}$

Where:

* $p$ is the encoder count.

* $\text{res}$ is the encoder resolution (4096 counts/revolution for all X-series models).

* In **Position Control Mode**, $p \in [0, \text{res}-1]$.

* In **Extended Position Control Mode**, $p$ can span the entire 32-bit signed integer range.

> **Tip:** Always convert desired angles into integer encoder counts and cache them to avoid cumulative floating-point drift in closed-loop position control algorithms.

---

## Key Concepts: Physical, Home, and Logical Positions

### Physical Position

The **physical position** is the actual encoder count directly read from the Dynamixel motor’s encoder. This position is independent of higher-level software abstractions and relies solely on the internal motor firmware. It is initialized based on the motor's position at startup and is represented in native Dynamixel encoder counts.

### Home Position

The **home position** is a predefined physical encoder count set as the default or reference position for the motor. This serves as a baseline for all subsequent position calculations.

### Logical Position

The logical position is defined relative to the home position. By definition, the logical home is always 0 and directly corresponds to any physical position. For example, if the encoder has a resolution of 4096 counts per revolution, a rotation of +90 degrees from the logical home position corresponds to an encoder increment of 1024 counts.

The diagram below illustrates logical positions relative to a defined physical home position. The **logical home (0)** is physically located at an encoder count of **2560**.

* Encoder counts around the circle hich has a resolution of 4096 counts per revolution.

* A **-90°** rotation (clockwise) from the **physical** home position **(2560)** corresponds **physically** to encoder count **1536**, shown highlighted in red.

* A **logical** rotation of **-90°** corresponds to a decrease of 1024 encoder counts from the logical home. Thus, the resulting logical position is **-1024** counts relative to the logical home

![Key_Concepts](doc_images/key_concepts_dynamixel.png)
---

## Position Control Mode (Single-Turn)

Position control mode limits the servo rotation strictly within one full revolution (0 to 4095 encoder counts). The servo **cannot cross the physical zero boundary** in either direction. This limitation arises due to firmware constraints, irrespective of any logical offsets or adjustments.

### Practical Considerations

Incorrect assembly relative to physical zero can cause significant issues, illustrated in the figure (to be added). For example, consider two linked joints:

* The joint configuration prevents rotation past certain mechanical stops (blue area).
* The servo physically cannot cross zero due to firmware limitations, restricting reachable positions (red area).

In such scenarios, choosing **Extended Position Control Mode** may resolve these positioning constraints without mechanical redesign.

---

## Extended Position Control Mode (Multi-Turn)

Extended position control allows rotation beyond one revolution, overcoming the single-turn limitations by enabling continuous rotation and crossing of the physical zero boundary. Positions are continuously cumulative, with each revolution adding or subtracting 4096 encoder counts.

### Practical Considerations

While multi-turn mode provides greater flexibility, it requires careful consideration:

* After powering on, encoder values reset and are limited to a 0–4095 range, leading to potential confusion if a joint’s logical range spans negative values.
* For instance, calibrating a motor with a logical range of \[-1024, 1024] can lead to unexpected behavior after restarting if the motor is physically located within a negative logical region. The encoder will report values as if they were within the positive (3072–4095) range, leading to possible full rotations if commanded naively.

Careful initialization and calibration routines must be implemented to manage these scenarios safely.

This behavior is illustrated in the figure below (to be added).

---
[Back to Overview](README.md)