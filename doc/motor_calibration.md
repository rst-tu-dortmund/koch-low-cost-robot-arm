# Calibration Procedure for Dynamixel Motors (Robot Arm Koch V1.1)

## Overview

This document outlines the precise calibration steps for Dynamixel motors used in the Robot Arm Koch V1.1. Proper calibration ensures accurate positioning, effective motion control, and reliable long-term operation.

## Prerequisites

* Dynamixel motors (models compatible with Robot Arm Koch V1.1)
* Dynamixel Wizard 2.0 ([Download Link](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/))
* Dynamixel U2D2 USB Communication Converter ([ROBOTIS](https://emanual.robotis.com/docs/en/parts/interface/u2d2/))
* Power Hub Board (if required)
* Koch V1.1 Robot Arm
* Adequate power supply (per motor specification)

## Motor Configuration in Dynamixel Wizard 2.0

### Step 1: Install Dynamixel Wizard 2.0

* Download and install Dynamixel Wizard 2.0 from [this link](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/).
* Verify successful installation by launching the application.

### Step 2: Set Unique Motor IDs

* Connect each Dynamixel motor individually to the U2D2 USB Communication Converter. **Only one motor should be connected at a time.**
* Use Dynamixel Wizard to assign a unique ID to each motor:

  * Navigate to the ID configuration tab.
  * Enter and apply a unique ID number for each motor.
* Disconnect the motor and repeat the process for each additional motor.

> **Important Note:** If using the U2D2 with a Power Hub Board (ROBOTIS), ensure the power supply matches the requirements of the specific Dynamixel motor model. Incorrect power supply can damage the motors or the converter.

### Step 3: Reconnect Motors to the Waveshare Board

* After assigning unique IDs, reconnect the motors in series to the Waveshare board (installed on Koch V1.1).
* Power on the system.
* Run a scan in Dynamixel Wizard 2.0 again:

  * Pay close attention to scan settings, as USB connection ports can vary.

**Test Results:**

* If motors appear in the scan, the Waveshare board is confirmed operational.
* If motors do not appear, replace the Waveshare board and repeat this step.

### Step 4: Disable Torque

* Ensure torque is **disabled** for all motors via Dynamixel Wizard to allow manual manipulation.
  * In the control table this corresponds to `Torque Enable` (e.g. address 64) = `0`.

### Step 5: Set Extended Position Mode

* Configure each motor to **Extended Position Control Mode**:
  * Set the `Operating Mode` register (e.g. address 11) to the value for **Extended Position** (typically `4` for XL-series Dynamixels).
* Apply the change to each motor.

### Step 6: Set Baudrate

* Set the baud rate to **1,000,000 bps (1 Mbps)** for all motors.

### Step 7: Set Motor Direction

* Open the `Drive Mode` register (e.g. address 10).
* Set **Bit 0 (Reverse Mode)** according to the DH-axis direction used in the Koch V1.1 kinematic model (see image with directions).
  * You can manully rotate joints and record Present Position changes (register `132`) to determine actual motor direction.

![dh_directions](dh_directions.png)

> Bit 0 = 0 → normal direction  
> Bit 0 = 1 → reversed direction

### Step 8: Restart Motors

* After changing to Extended Position Mode, completely power off the robot.
* Reconnect power and restart the robot to apply the new settings (it may be needed to scan the motors again).

## Manual Calibration

> Make sure **torque is disabled** for all motors before performing any manual movements (see Step 4).

### Step 9: Set and Record Home Positions

* Manually rotate each joint to its **home** position (see image with default position).
* Read and record the home positions using Dynamixel Wizard 2.0:
  * Read the **Present Position** register (`132`) for each motor.
* Self-check for home position:
  * The **home** position of each motor should be a multiple of `1024`.
  * This corresponds to exact quarter-turn steps (for 4096 ticks per revolution).
  * Example: if Dynamixel Wizard shows something like `3000`, set the home position to the nearest multiple of 1024, e.g. `3072`.
* Store the recorded home-position values (per motor ID).  
  These values will be used later in your control code (Koch V1.1 software) to define the reference zero of each joint.


---
[Back to Overview](README.md)