# MIT License

# Copyright (c) 2024 Alexander Koch

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# Adapted by Heiko Renz, 2025


import os
import time
import mujoco
import mujoco.viewer
import numpy as np
import utils
from kochV1_package.kochV1_robot_simulation import KochV1_Robot_Simulation

r = KochV1_Robot_Simulation()

with mujoco.viewer.launch_passive(r.mjModel, r.mjData) as viewer:
  start = time.time()
  time.sleep(1.0)  # Allow viewer to initialize
  while viewer.is_running():
    step_start = time.time()
    mujoco.mj_step(r.mjModel, r.mjData)
    viewer.sync()   
    
    # Example commands:
    
    # r.set_goal_velocities([0, 0, -50, 0, 0])
    
    # r.set_joints([0, 0, -45, 0, 0], unit=utils.Unit.DEG)
    
    # r.set_gripper_percentage(0.2)
    
    
    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = r.mjModel.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)