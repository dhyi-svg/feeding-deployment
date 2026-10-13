"""Drive RammpArm.press_impedance against the mock (domain 77)."""
import os, sys, json
assert os.environ.get("ROS_DOMAIN_ID") == "77"
import numpy as np, rclpy
from rammp_button_press import Abort
from rammp_button_press.rammp_arm import RammpArm
depth = float(sys.argv[1])
N_OUT = np.array([-0.9, -0.435, 0.025]); N_OUT /= np.linalg.norm(N_OUT)
rclpy.init(); a = RammpArm(execute=True)
p, _ = a.ee_pose()
try:
    r = a.press_impedance(p, N_OUT, depth, "mock press", lambda rec: None)
    print("RESULT", r)
except Abort as e:
    print("ABORT", e)
a.close(); rclpy.shutdown()
