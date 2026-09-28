# Plan-only: retreat-over from the REAL current arm state, door_z/door_mid estimated (door file untouched).
import json, sys, types
from pathlib import Path
import numpy as np
sys.path.insert(0, "microwave")
from feeding_deployment.control.robot_controller.arm_client import ArmInterfaceClient
from door_push import DoorFrame, _plan_retreat_over
from microwave_common import make_sim
st = ArmInterfaceClient().get_state()
st = {"ee_pos": np.array(st["ee_pos"], float), "position": np.array(st["position"], float)}
door = json.loads((Path.home() / ".microwave_door.json").read_text())
print("door angle from held handle:", round(DoorFrame(door, 1.0).angle_of(st["ee_pos"]), 1))
mid_y = float(door["closed_grasp_pos"][1] + door["hinge"][1]) / 2 - 0.03   # rough: halfway hinge->free edge
scene, rb = make_sim()
for top in (0.42,):
    for out in (0.05,):
        d = dict(door, door_z=[0.14, top], door_mid=[0.65, mid_y, 0.28])
        print(f"\n===== door top {top}, out {out}, mid y {mid_y:.3f} =====")
        plan = _plan_retreat_over(scene, rb, d, st, types.SimpleNamespace(retreat_out=out, retreat_above=0.08))
        print("PLAN", "OK" if plan else "FAILED")
