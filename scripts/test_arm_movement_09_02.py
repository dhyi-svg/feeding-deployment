"""Slow, gated straight-line test move: translate the gripper by a constant x offset.

The smallest motion test on this rig -- no perception, no planner, no behavior
tree. It takes the arm's current pose, adds a constant offset along +x in
arm_base_link (+x is out/forward, away from the arm), holds the orientation
fixed, and walks there in small pre-validated sub-steps. Nothing is detected and
nothing is grasped.

Straight-line EE paths are the branch-flip case this rig has actually been bitten
by, so this follows the proven pattern rather than commanding a Cartesian pose
and hoping. Every sub-step is solved with PyBullet IK SEEDED from the previous
step (the approach script's --interp cartesian mode), the whole chain is gated in
sim BEFORE anything is commanded, and each step is then sent as a JointCommand.
The two reasons, both already paid for on this hardware:

  - A Cartesian command hands the branch choice to the arm. `goto_preset.py`
    avoids IK entirely for exactly this reason, and CLAUDE.md (2026-07-31)
    records the real incident: a Cartesian-correct waypoint chain whose IK was
    not seeded from the arm's true posture swept ~150 deg on J1/J3/J7.
  - Per-step IK drifts through the 7-DOF null space even when the EE path is
    perfect: measured 2026-08-05 on the approach script, a 12-step chain hit its
    target to 0.001 cm while ending 83.8 deg away in JOINT space. Sub-stepping
    alone does NOT bound that -- the explicit --max-step-deg check does.

An EE tracking check cannot catch either one: the gripper arrives on target with
a completely different arm behind it. So the joint guard runs in sim first, and
the tracking check is only the second line.

Prerequisites (runbook rung 1, in this order): arm_server.py, stub_base_server.py,
bulldog_bypass.py. Set the speed FIRST -- this script refuses to move unless the
arm is on the "low" preset, since moving slowly is the whole point:

    $PY scripts/session/arm_set_speed.py low

There is NO software e-stop on this rig -- keep a hand on the physical one for
any --execute run. Run from the repo root (the sim scene path is relative).

    E="PYTHONPATH=$HOME/.local/lib/python3.10/site-packages ARM_RPC_HOST=127.0.0.1"
    PY=$HOME/feeding-deployment/.venv/bin/python

    $PY scripts/test_arm_movement_09_02.py                        # dry run, +5 cm
    $PY scripts/test_arm_movement_09_02.py --execute              # move +5 cm
    $PY scripts/test_arm_movement_09_02.py --dx -0.05 --execute   # and back
"""
import argparse
import sys
import time

import numpy as np
import pybullet as p
from pybullet_helpers.geometry import Pose, multiply_poses

from feeding_deployment.control.robot_controller.arm_client import ArmInterfaceClient
from feeding_deployment.control.robot_controller.command_interface import JointCommand
from feeding_deployment.simulation.scene_description import (
    create_scene_description_from_config,
)
from feeding_deployment.simulation.simulator import FeedingDeploymentPyBulletSimulator

# Offset along +x in arm_base_link (out/forward, away from the arm).
DEFAULT_DX_M = 0.05
MAX_DX_M = 0.15           # a motion smoke test, not a transit: cap what can be asked

# Sub-step sizing when --steps is not given. 2.5 cm keeps each increment small
# enough that seeded IK stays on the current branch.
DEFAULT_STEP_M = 0.025

# Safety envelope, same arm and same rig as the real_gen3_ros2_* scripts.
# 0.88 rather than the approach script's 0.85: the arm demonstrably reached
# 0.858 under teleop and 0.85 was found over-tight, blocking a feasible pose
# (real_gen3_ros2_grasp_microwave.py).
MAX_REACH, MIN_Z, MAX_Z = 0.88, 0.25, 0.75
MAX_IK_ERR = 0.02         # reject an IK solution that misses the waypoint
TRACK_ABORT = 0.03        # abort if a commanded sub-step lands this far off plan
REQUIRED_SPEED = "low"
PAUSE_S = 0.5
ARM = [1, 2, 3, 4, 5, 6, 7]

parser = argparse.ArgumentParser(
    description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--dx", type=float, default=DEFAULT_DX_M,
                    help=f"x offset in metres, +forward (default {DEFAULT_DX_M}); "
                         f"negative retraces the move")
parser.add_argument("--steps", type=int, default=0,
                    help=f"split the move into N sub-steps (default 0 = auto, one per "
                         f"{DEFAULT_STEP_M} m). More steps = slower and finer.")
parser.add_argument("--max-step-deg", type=float, default=20.0,
                    help="abort if any ONE sub-step moves a joint more than this (default 20)")
parser.add_argument("--allow-any-speed", action="store_true",
                    help=f"skip the '{REQUIRED_SPEED} preset' check (not recommended)")
parser.add_argument("--execute", action="store_true", help="actually command the arm")
args = parser.parse_args()

if args.dx == 0.0:
    sys.exit("--dx 0 -- nothing to do.")
if abs(args.dx) > MAX_DX_M:
    sys.exit(f"--dx {args.dx} exceeds the {MAX_DX_M} m cap for this test script.")
if args.steps < 0:
    sys.exit("--steps must be >= 0")

ai = ArmInterfaceClient()
state = ai.get_state()
ee = np.asarray(list(state["ee_pos"]), dtype=float)
start_pos, quat = ee[:3], ee[3:7]
start_joints = np.asarray(state["position"], dtype=float)
gripper = float(state.get("gripper_pos"))
speed = ai.get_speed()

print(f"start EE      : [{start_pos[0]:.4f}, {start_pos[1]:.4f}, {start_pos[2]:.4f}]")
# Record the pose being LEFT, not just the one being gone to: arm_commands_log.txt
# is wiped on every arm_server restart, so this print is the only way back to the
# starting posture if the move aborts partway.
print(f"start joints  : {[round(float(v), 6) for v in start_joints]}")
print(f"gripper       : {gripper:.4f}  ({'open' if gripper < 0.2 else 'CLOSED'})")
print(f"speed         : {speed}")

# Standing rule (CLAUDE.md): check gripper_pos before anything moves. It cannot
# tell you whether a grasp succeeded, but a closed gripper means the arm may be
# holding something -- and dragging that sideways is not what this script is for.
if gripper > 0.2:
    sys.exit("Gripper is CLOSED -- it may be holding something. Refusing to move.")
# The task scripts read the speed and never set it, so the arm keeps whatever the
# last session left it on. Owning that state here would fight arm_set_speed.py,
# so this verifies instead of setting.
if speed != REQUIRED_SPEED and not args.allow_any_speed:
    sys.exit(f"Speed is '{speed}', not '{REQUIRED_SPEED}' -- this is the slow-move test. "
             f"Run: $PY scripts/session/arm_set_speed.py {REQUIRED_SPEED}\n"
             f"(or pass --allow-any-speed to override)")

tgt = start_pos + np.array([args.dx, 0.0, 0.0])
print(f"target EE     : [{tgt[0]:.4f}, {tgt[1]:.4f}, {tgt[2]:.4f}]  (dx {args.dx:+.3f} m)")
print(f"orientation   : held fixed at {np.round(quat, 4).tolist()}")

# ---- safety gates -----------------------------------------------------------
fail = []
if np.linalg.norm(tgt) > MAX_REACH:
    fail.append(f"target range {np.linalg.norm(tgt):.3f} m > {MAX_REACH} m")
# dz is 0 by construction, so this really gates the pose the arm is STARTING in --
# it catches being parked somewhere the envelope was never checked for.
if not (MIN_Z <= tgt[2] <= MAX_Z):
    fail.append(f"z {tgt[2]:.3f} outside [{MIN_Z}, {MAX_Z}] -- reposition before testing")
if fail:
    sys.exit("SAFETY GATE FAILED: " + "; ".join(fail))
print("safety gates  : PASS (reach, height, offset cap)")

# ---- seeded IK over the straight line ---------------------------------------
scene = create_scene_description_from_config(
    "src/feeding_deployment/simulation/configs/vention.yaml", "skewer")
sim = FeedingDeploymentPyBulletSimulator(scene, use_gui=False)
rb = sim.robot


def ik(pose, seed):
    """Seeded IK for an arm_base_link pose. Returns (joints, world-frame error)."""
    for i, jj in enumerate(ARM):
        p.resetJointState(rb.robot_id, jj, float(seed[i]),
                          physicsClientId=rb.physics_client_id)
    # The sim places the robot at a world offset (it sits on the vention base), so
    # an arm_base_link target must be lifted into sim world coordinates before IK --
    # otherwise IK chases a point metres away.
    w = multiply_poses(scene.robot_base_pose, pose)
    sol = p.calculateInverseKinematics(
        rb.robot_id, rb.end_effector_id, list(w.position), list(w.orientation),
        maxNumIterations=400, residualThreshold=1e-5,
        physicsClientId=rb.physics_client_id)
    q = np.asarray(sol[:7], dtype=float)
    for i, jj in enumerate(ARM):
        p.resetJointState(rb.robot_id, jj, float(q[i]),
                          physicsClientId=rb.physics_client_id)
    ls = p.getLinkState(rb.robot_id, rb.end_effector_id,
                        physicsClientId=rb.physics_client_id)
    return q, float(np.linalg.norm(np.asarray(ls[4]) - np.asarray(w.position)))


def fk(joints):
    """EE position in arm_base_link for a joint vector."""
    for i, jj in enumerate(ARM):
        p.resetJointState(rb.robot_id, jj, float(joints[i]),
                          physicsClientId=rb.physics_client_id)
    ls = p.getLinkState(rb.robot_id, rb.end_effector_id,
                        physicsClientId=rb.physics_client_id)
    return np.asarray(ls[4]) - np.asarray(scene.robot_base_pose.position)


n = args.steps if args.steps > 0 else int(np.ceil(abs(args.dx) / DEFAULT_STEP_M))
increment = abs(args.dx) / n
print(f"\nplanning {n} sub-step(s) of {increment*100:.1f} cm "
      f"(guard {args.max_step_deg} deg/step):")

# Plan and gate the WHOLE chain in sim before commanding anything: a partially
# executed chain leaves the arm somewhere unplanned.
plan, seed, worst = [], start_joints, 0.0
for step in range(1, n + 1):
    want = start_pos + (tgt - start_pos) * (step / n)
    q, err = ik(Pose(tuple(want), tuple(quat)), seed)
    delta = float(np.degrees(np.max(np.abs(q - seed))))
    worst = max(worst, delta)
    bad = []
    if err > MAX_IK_ERR:
        bad.append(f"ik {err*100:.1f}cm")
    if delta > args.max_step_deg:
        bad.append(f"step {delta:.0f}deg")
    print(f"  step {step:02d}/{n}  EE {np.round(want, 4)}  ik {err*100:5.2f} cm  "
          f"step {delta:5.1f} deg  {'FAIL: ' + '; '.join(bad) if bad else 'OK'}")
    if bad:
        sys.exit(f"Sub-step {step} failed its guard -- refusing the whole move. "
                 f"Try more --steps, or a smaller --dx.")
    plan.append((step, want, q))
    seed = q

# Confirm the chain actually ends where it was asked to, and report how far the
# posture drifted getting there -- the null-space number the EE error hides.
end_err = float(np.linalg.norm(fk(plan[-1][2]) - tgt))
posture = float(np.degrees(np.max(np.abs(
    (plan[-1][2] - start_joints + np.pi) % (2 * np.pi) - np.pi))))
print(f"chain OK      : worst single step {worst:.1f} deg, "
      f"total posture change {posture:.1f} deg")
print(f"endpoint      : {end_err*100:.3f} cm from the gated target")
if end_err > MAX_IK_ERR:
    sys.exit(f"Chain ends {end_err*100:.1f} cm off target -- refusing.")

if not args.execute:
    sys.exit("\nDRY RUN -- nothing commanded. Re-run with --execute to move.")

# ---- execute ----------------------------------------------------------------
print(f"\nspeed: {speed}  -- commanding {len(plan)} move(s) ...")
for step, want, q in plan:
    ai.execute_command(JointCommand(pos=q.tolist()))
    # A move can return before it settles -- wait for velocity ~0, then re-check.
    for _ in range(150):
        time.sleep(0.2)
        if float(np.max(np.abs(np.asarray(
                ai.get_state()["velocity"], dtype=float)))) < 1e-3:
            break
    time.sleep(0.4)

    here = np.asarray(list(ai.get_state()["ee_pos"])[:3], dtype=float)
    track = float(np.linalg.norm(here - want))
    print(f"  step {step:02d}/{n}  EE {np.round(here, 4)}  tracking {track*100:4.1f} cm")
    if track > TRACK_ABORT:
        sys.exit(f"Tracking {track*100:.1f} cm at step {step} "
                 f"(> {TRACK_ABORT*100:.0f} cm) -- ABORT, holding here.")
    time.sleep(PAUSE_S)

st = ai.get_state()
final = np.asarray(list(st["ee_pos"])[:3], dtype=float)
final_joints = np.asarray(st["position"], dtype=float)
err = final - tgt
print(f"\nfinal EE      : [{final[0]:.4f}, {final[1]:.4f}, {final[2]:.4f}]")
print(f"target        : [{tgt[0]:.4f}, {tgt[1]:.4f}, {tgt[2]:.4f}]")
print(f"tracking error: [{err[0]:+.4f}, {err[1]:+.4f}, {err[2]:+.4f}]  "
      f"|e| = {np.linalg.norm(err)*100:.1f} cm")
print(f"travelled     : {np.linalg.norm(final-start_pos)*100:.1f} cm "
      f"(commanded {abs(args.dx)*100:.1f} cm)")
print(f"final joints  : {[round(float(v), 6) for v in final_joints]}")
print(f"\nto retrace:  $PY scripts/test_arm_movement_09_02.py --dx {-args.dx:g} --execute")
