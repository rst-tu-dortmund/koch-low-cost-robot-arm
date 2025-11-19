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

## Installation / Setup

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

### Step 4: Set Extended Position Mode

* Configure each motor to **Extended Position Mode** using Dynamixel Wizard.

### Step 5: Restart Motors

* After changing to Extended Position Mode, completely power off the robot.
* Reconnect power and restart the robot to apply the new settings (it may be needed to scan the motors again).

### Step 6: Disable Torque

* Ensure torque is **disabled** for all motors via Dynamixel Wizard to allow manual manipulation.

## Manual Calibration

* Manually rotate each joint to its **home** (default) position.
* Record the home positions using the Dynamixel Wizard 2.0 application by reading from the **Present Position** register (`132`).
* Carefully move each joint to its physical limits.
* Record these limit positions similarly using Dynamixel Wizard 2.0 (Present Position, register `132`).
---
[Back to Overview](README.md)