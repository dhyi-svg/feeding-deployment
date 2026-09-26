"""SIM ONLY -- never commands the arm. Dry-run `microwave_common`'s release / re-grasp
planners from hypothetical arm states via a fake arm, using the recorded
`~/.microwave_last_grasp.json` + `~/.microwave_door.json`. This is how the 09-23 findings were
made (the joint move back-off -> park clips the open door; paths to park cross J3 +-180; the
re-grasp from park clips the door).

    python3 microwave/tools/what_if_release_and_regrasp.py release-open    # at the recorded open-door grasp
    python3 microwave/tools/what_if_release_and_regrasp.py release-closed  # at the closed-door grasp (IK'd)
    python3 microwave/tools/what_if_release_and_regrasp.py regrasp         # from a few door-facing starts
"""
import json
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import microwave_common as mc  # noqa: E402


class FakeArm:
    def __init__(self, q, pos, quat, gripper):
        self.q, self.ee, self.g = np.asarray(q, float), list(pos) + list(quat), gripper

    def get_state(self):
        return {"position": self.q, "ee_pos": self.ee, "gripper_pos": self.g}


rec = json.loads(mc.LAST_GRASP_FILE.read_text())
door = json.loads(mc.DOOR_FILE.read_text())
what = sys.argv[1] if len(sys.argv) > 1 else "release-open"

if what == "release-open":
    for bo, x in [(0.20, 0.216), (0.16, 0.216)]:
        print(f"\n######## back-off {bo * 100:.0f} cm, toward base to x={x}, then park")
        mc.release_and_back_off(FakeArm(rec["grasp_joints"], rec["grasp_pos"], rec["quat"], 0.917),
                                bo, execute=False, pre_park_x=x)
elif what == "release-closed":
    open_deg = abs(mc.open_door_state(door, rec)[3])
    quat_c = (R.from_euler("z", np.radians(open_deg)) * R.from_quat(rec["quat"])).as_quat()
    scene, rb = mc.make_sim()
    q_c, err = mc.solve_ik(scene, rb, door["closed_grasp_pos"], quat_c, rec["grasp_joints"])
    print(f"(closed-door grasp joints from IK, err {err * 100:.2f} cm)")
    mc.release_and_back_off(FakeArm(q_c, door["closed_grasp_pos"], quat_c, 0.917), 0.20, execute=False)
elif what == "regrasp":
    bo = np.array(rec["back_off_pos"])
    scene, rb = mc.make_sim()
    for name, pos, quat in [
            ("15 cm above the back-off point", bo + [0, 0, 0.15], rec["quat"]),
            ("10 cm toward the base + 10 cm up", bo + [-0.10, 0, 0.10], rec["quat"]),
            ("off to the side (+y 12 cm)", bo + [0, 0.12, 0.0], rec["quat"]),
            ("gripper yawed 30 deg, 10 cm up", bo + [0, 0, 0.10],
             (R.from_euler("z", np.radians(30)) * R.from_quat(rec["quat"])).as_quat())]:
        q, err = mc.solve_ik(scene, rb, list(pos), quat, rec["back_off_joints"])
        print(f"\n######## start: {name} {np.round(pos, 3)} (IK err {err * 100:.2f} cm)")
        mc.regrasp(FakeArm(q, list(pos), list(quat), 0.004), execute=False)
else:
    sys.exit(f"unknown: {what}")
