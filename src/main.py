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

import logging
import time

import numpy as np

from kochV1_package import KochV1_Robot, KochV1_DxlBus
from utils import Unit

def main():
    logging.basicConfig(level=logging.INFO)
    # Physical home position of the motors in the assembly
    motor_physical_home_positions = [2048, 1024, 2048, 2048, 2048, 2048]

    # Recommended: use the bus as a context manager
    with KochV1_DxlBus(motor_physical_home_positions) as dxl_bus:
        robot = KochV1_Robot(dxl_bus)
                
        # Move to a joint configuration (degrees shown for readability)
        robot.set_joints([0, 90, -90, -90, 0], unit=Unit.DEG)
        time.sleep(3)  # allow time to reach goal position

        robot.set_gripper_position_from_xyz_psi_phi(0.13, 0.0, 0.09, -np.pi/2, 0)
        time.sleep(3)  # allow time to reach goal position


        robot.set_joints([-45, 90, -90, -90, 0], unit=Unit.DEG)
        time.sleep(3)  # allow time to reach goal position

        robot.set_gripper_percentage(0.3)
        time.sleep(0.75)
        robot.set_gripper_percentage(0.8)
        time.sleep(0.75)

        robot.set_joints([0, 90, -90, -90, 0], unit=Unit.DEG)
        time.sleep(3)  # allow time to reach goal position

        robot.set_joints([0, 0, -90, 0, 0], unit=Unit.DEG)
        time.sleep(3)  # allow time to reach goal position

        # Read back the current configuration
        print(robot.read_joints(unit=Unit.DEG))
        time.sleep(1)

if __name__ == "__main__":
    main()
