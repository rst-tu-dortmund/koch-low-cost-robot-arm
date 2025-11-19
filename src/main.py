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
