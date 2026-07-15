"""Open the microwave door on the REAL Gen3, ONE arc waypoint per call, using
the repo's own arc geometry driven by joint-space sim-IK.

This is the exact script that ran the 2026-07-14 door-opening arc (rung 5 of
the real-arm test ladder -- see TESTING_LOG.md's "2026-07-14 -- autonomous
detect -> grasp -> open" entry and arm_commands_log.txt for the corroborating
hardware log). Recovered from that session's scratchpad (it was never
committed) and checked back in here verbatim so it doesn't get lost again --
see feedback_live_test_log: reconstructing a live-tested script after the fact
is exactly what we don't want to repeat.

Uses perception_interface.PerceptionInterface._generate_door_arc_waypoints (no
`self` used internally, so invoked unbound here) with the repo's own microwave
params: arc_length_m=0.55, waypoint_spacing_m=0.05, direction=-1 (microwave is
left-hinged), rotate_orientation=True -- same params open_door.py's
open_microwave() uses. Start pose = the real current EE pose (assumed to
already be at the grasp pose -- run the detect+grasp script first); hinge
position is *estimated* one door-width (0.32 m) to +y, since the wrist-mounted
camera can't see the hinge while grasping the handle.

Joint-space, not cartesian: set_ee_pose/plan_to_ee_pose-style cartesian control
was unreliable at these extended arm configs on this rig; set_joint_position
was reliable. Each waypoint's joint target comes from PyBullet's native
calculateInverseKinematics (this custom URDF's IKFast build hangs on this box,
see CLAUDE.md's "Known wall" note) -- deliberately UNSEEDED. Seeding
calculateInverseKinematics from the arm's current real joints (via
p.resetJointState) is the right fix for big single-shot jumps (approach,
retract, back-off -- prevents a wrist-flip to a far-away IK solution) but is
TOO CONSERVATIVE for these small arc waypoints: seeding stalled on arc step 2
in this same session (sim ik_err 3.5 cm) where unseeded IK reached it exactly.
So: seed for large jumps elsewhere, leave this arc runner unseeded.

Run from the repo root, with the arm server + bulldog bypass already up (see
TESTING_LOG.md's "Setup -- talking to the arm" section for the env vars and
bring-up order):

  PYTHONPATH=$HOME/.local/lib/python3.10/site-packages ARM_RPC_HOST=127.0.0.1 \\
      .venv/bin/python scripts/real_gen3_open_microwave.py

Run again to advance one waypoint at a time; state (waypoints + progress index)
persists in /tmp/door_arc.json. Delete that file to re-plan from the arm's
current pose (e.g. after a fresh grasp).
"""
import json
import sys
import time
from pathlib import Path

import numpy as np
import pybullet as p
from pybullet_helpers.geometry import Pose, multiply_poses

from feeding_deployment.interfaces.perception_interface import PerceptionInterface
from feeding_deployment.simulation.scene_description import create_scene_description_from_config
from feeding_deployment.simulation.simulator import FeedingDeploymentPyBulletSimulator
from feeding_deployment.control.robot_controller.arm_interface import (
    ArmManager, NUC_HOSTNAME, ARM_RPC_PORT, RPC_AUTHKEY,
)

# Estimated hinge offset (door width) and per-waypoint tracking-error abort
# threshold -- both empirical, from the 2026-07-14 session.
DOOR_W, ABORT = 0.32, 0.02
S = Path("/tmp/door_arc.json")

ArmManager.register("ArmInterface")
mg = ArmManager(address=(NUC_HOSTNAME, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
mg.connect()
ai = mg.ArmInterface()
ai.set_speed("low")

if not S.exists():
    ee = list(ai.get_state()["ee_pos"])
    grasp = Pose(position=tuple(ee[:3]), orientation=tuple(ee[3:7]))
    hinge = (ee[0], ee[1] + DOOR_W, ee[2])  # door-width to +y (left), same depth/height
    # REPO's arc geometry, exact microwave params (matches open_door.py's open_microwave()):
    wps_pose = PerceptionInterface._generate_door_arc_waypoints(
        None, start_pose=grasp, hinge_position=hinge,
        arc_length_m=0.55, waypoint_spacing_m=0.05, direction=-1, rotate_orientation=True)
    wps = [list(w.position) + list(w.orientation) for w in wps_pose]
    S.write_text(json.dumps({"wps": wps, "i": 0, "hinge": list(hinge)}))
    print(f"repo _generate_door_arc_waypoints: {len(wps)} waypoints (arc 0.55m, dir=-1), hinge {np.round(hinge, 3)}")

st = json.loads(S.read_text())
i, wps = st["i"], st["wps"]
if i >= len(wps):
    print("arc COMPLETE.")
    sys.exit(0)
wp = wps[i]
pos, quat = wp[:3], wp[3:]

sd = create_scene_description_from_config("src/feeding_deployment/simulation/configs/vention.yaml", "skewer")
sim = FeedingDeploymentPyBulletSimulator(sd, use_gui=False)
rb = sim.robot
wpose = multiply_poses(sd.robot_base_pose, Pose(tuple(pos), tuple(quat)))
sol = p.calculateInverseKinematics(
    rb.robot_id, rb.end_effector_id, list(wpose.position), list(wpose.orientation),
    physicsClientId=rb.physics_client_id, maxNumIterations=200)
for j, jj in enumerate([1, 2, 3, 4, 5, 6, 7]):
    p.resetJointState(rb.robot_id, jj, sol[j], physicsClientId=rb.physics_client_id)
ikerr = np.linalg.norm(np.array(rb.get_end_effector_pose().position) - np.array(wpose.position))
d = float(np.linalg.norm(pos))
print(f"step {i + 1}/{len(wps)} -> target {np.round(pos, 3)} ({d * 100:.0f}cm from base, sim ik_err {ikerr * 100:.1f}cm)")
if ikerr > 0.02 or d > 0.90:
    print("  target unreachable / past limit -- stopping (door only opens so far before over-extend).")
    sys.exit(1)

ai.set_joint_position([sol[k] for k in range(7)])
for _ in range(60):
    stt = ai.get_state()
    if (np.linalg.norm(np.array(stt["ee_pos"][:3]) - np.array(pos)) < 0.02
            and max(abs(x) for x in stt["velocity"]) < 0.01):
        break
    time.sleep(0.2)
got = np.array(ai.get_state()["ee_pos"][:3])
err = np.linalg.norm(got - np.array(pos))
print(f"  reached {np.round(got, 3)} | tracking err {err * 100:.1f}cm | gripper {ai.get_state()['gripper_pos']:.2f}")
if err > ABORT:
    print(f"  ABORT: err {err * 100:.1f}cm > {ABORT * 100:.0f}cm -- door binding / latch / limit. Not advancing.")
    sys.exit(1)

st["i"] = i + 1
S.write_text(json.dumps(st))
print(f"  step {i + 1} done. {len(wps) - st['i']} steps left. Re-run to continue.")
