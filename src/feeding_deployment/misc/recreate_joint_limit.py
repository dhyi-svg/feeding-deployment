"""Recreate the Jul 6 2026 19:45:16 CloseMicrowave Cartesian stall (joint limit /
singularity region).

Replays, with hardcoded values, the exact command sequence the arm server received
from the CloseMicrowave skill (NUC robot pane log, bundle session_20260706_131116):
retract preset, three approach poses, post-swing waypoint, then the offset
(hard-push) closing arc that the onboard Cartesian controller abandoned partway
("Arm did not reach desired position"). All poses are in the arm base frame,
quaternions xyzw.

Runs on compute through the arm server RPC (ArmInterfaceClient), so the
watchdog/bulldog e-stop chain stays functional.

Safety: the end effector sweeps a ~35 cm arc at z ~= 0.52 m in the arm base
frame. Make sure that volume is clear. No microwave/door contact is needed --
the stall is kinematic.

Usage:
    python recreate_joint_limit.py                # full sequence replay
    python recreate_joint_limit.py --from-stuck   # skip the sequence: joint-move
        straight to the stall configuration recovered from the run, then send
        the arc (deterministic even if the full replay lands in a different
        IK branch and completes)
    python recreate_joint_limit.py --reset-before-arc  # full replay, but joint-move
        to left_back_retract_pos right before approaching the arc start,
        so the arc begins from a fresh configuration at the same EE pose (tests
        whether a different IK branch avoids the stall)
"""

import sys
import time

import numpy as np
import rospy

from feeding_deployment.control.robot_controller.arm_client import ArmInterfaceClient
from feeding_deployment.control.robot_controller.command_interface import (
    CartesianCommand,
    CartesianTrajectoryCommand,
    CloseGripperCommand,
    JointCommand,
    OpenGripperCommand,
)

# close_microwave starts from this preset (behind_back_retract_pos), NUC log 19:44:25
BEHIND_BACK_RETRACT_POS = [
    3.141592653589793,
    -1.8338532592607812,
    3.1415681525077646,
    -2.5482659290666034,
    1.0329455279146852e-05,
    -0.8727280092311087,
    1.570780081512247,
]

# Posture reset target for --reset-before-arc (scene_description left_back_retract_pos:
# same posture as behind_back_retract but with joint 1 at -90 deg instead of 180)
LEFT_BACK_RETRACT_POS = [
    -1.57,
    -1.8338532592607812,
    3.1415681525077646,
    -2.5482659290666034,
    1.0329455279146852e-05,
    -0.8727280092311087,
    1.570780081512247,
]

# The three approach poses (before_above / above / closing waypoint), 19:44:29-19:44:38
APPROACH_POSES = [
    ([-0.2801022, -0.28035366, 0.68290432], [0.56934379, -0.41934192, -0.41934192, 0.56934379]),
    ([-0.30588061, -0.47718349, 0.68290432], [0.46856821, -0.52956948, -0.52956948, 0.46856821]),
    ([-0.30588061, -0.47718349, 0.60290432], [0.46856821, -0.52956948, -0.52956948, 0.46856821]),
]

# Reached right after the successful swing arc (closing_waypoints[-2]), 19:44:50.
# The swing arc itself was not logged; ending at this pose collapses its history.
POST_SWING_POSE = ([-0.33743565, -0.09030266, 0.60290432], [0.64472, -0.29040682, -0.29040682, 0.64472])

# offset_closing_waypoints -- the hard-push arc the firmware abandoned, 19:45:09-19:45:16.
# The first tuple is also the single-pose move sent just before the arc (19:44:52),
# which alone took 17 s -- the controller was already crawling.
OFFSET_CLOSING_ARC = [
    ([-0.30112464, -0.15563497, 0.51562633], [-0.21522222, 0.67355727, 0.67355727, -0.21522222]),
    ([-0.32086926, -0.10902577, 0.51562633], [-0.25053725, 0.66123452, 0.66123452, -0.25053725]),
    ([-0.34542574, -0.0647624, 0.51562633], [-0.28515129, 0.64706162, 0.64706162, -0.28515129]),
    ([-0.37451943, -0.02333989, 0.51562633], [-0.31896746, 0.63107825, 0.63107825, -0.31896746]),
    ([-0.40782495, 0.01477846, 0.51562633], [-0.35189117, 0.61332912, 0.61332912, -0.35189117]),
    ([-0.4449698, 0.04916635, 0.51562633], [-0.38383028, 0.59386389, 0.59386389, -0.38383028]),
    ([-0.48553854, 0.07943917, 0.51562633], [-0.41469543, 0.57273702, 0.57273702, -0.41469543]),
    ([-0.52907746, 0.10525834, 0.51562633], [-0.44440027, 0.55000764, 0.55000764, -0.44440027]),
    ([-0.5750996, 0.1263351, 0.51562633], [-0.47286167, 0.52573933, 0.52573933, -0.47286167]),
    ([-0.62309023, 0.14243371, 0.51562633], [0.5, -0.5, -0.5, 0.5]),
]

# Arm configuration where the run stalled (recovered from the teleop jog absolute
# targets). Joint 4 is at -127.8 deg (20 deg from its +-147.8 limit); nothing is
# hard-clamped, pointing at a singularity-region stall rather than a limit hit.
Q_STALL = [
    -1.68578783,
    -0.06365701,
    1.90270316,
    -2.23081553,
    -0.05163763,
    0.63729811,
    1.54551054,
]


def run_step(name, fn):
    print(f"--> {name}")
    start = time.time()
    result = fn()
    print(f"    done in {time.time() - start:.1f}s, returned {result}")
    return result


def print_state(client, label):
    state = client.get_state()
    print(f"{label} joint positions:", np.round(state["position"], 5).tolist())
    print(f"{label} end-effector pose:", np.round(state["ee_pos"], 5).tolist())
    return state


def main():
    from_stuck = "--from-stuck" in sys.argv[1:]
    reset_before_arc = "--reset-before-arc" in sys.argv[1:]

    rospy.init_node("recreate_joint_limit", anonymous=True)
    client = ArmInterfaceClient()

    print_state(client, "Current")

    if input("Press 'y' to run the recreation sequence: ") != "y":
        return

    client.set_speed("medium")

    if from_stuck:
        run_step(
            "joint move to Q_STALL (stuck configuration from the run)",
            lambda: client.execute_command(JointCommand(Q_STALL)),
        )
    else:
        run_step(
            "joint move to behind_back_retract_pos",
            lambda: client.execute_command(JointCommand(BEHIND_BACK_RETRACT_POS)),
        )
        run_step("open gripper", lambda: client.execute_command(OpenGripperCommand()))
        for i, (pos, quat) in enumerate(APPROACH_POSES):
            run_step(
                f"approach pose {i + 1}/3 {pos}",
                lambda pos=pos, quat=quat: client.execute_command(CartesianCommand(pos, quat)),
            )
        run_step(
            "post-swing pose (closing_waypoints[-2])",
            lambda: client.execute_command(CartesianCommand(*POST_SWING_POSE)),
        )
        run_step("close gripper", lambda: client.execute_command(CloseGripperCommand()))
        if reset_before_arc:
            run_step(
                "posture reset: joint move to behind_back_retract_pos",
                lambda: client.execute_command(JointCommand(BEHIND_BACK_RETRACT_POS)),
            )
            run_step(
                "posture reset: joint move to left_back_retract_pos",
                lambda: client.execute_command(JointCommand(LEFT_BACK_RETRACT_POS)),
            )
        run_step(
            "arc start pose (took 17s in the original run)",
            lambda: client.execute_command(CartesianCommand(*OFFSET_CLOSING_ARC[0])),
        )

    print_state(client, "At arc start")

    ok = run_step(
        "hard-push closing arc (10 waypoints) -- failed in the original run",
        lambda: client.execute_command(CartesianTrajectoryCommand(OFFSET_CLOSING_ARC)),
    )

    state = print_state(client, "Final")
    ee = np.asarray(state["ee_pos"], dtype=float)
    target = np.array(OFFSET_CLOSING_ARC[-1][0])
    print(f"Distance to final arc waypoint: {100 * np.linalg.norm(ee[:3] - target):.1f} cm")
    print("Stall config in the original run:", Q_STALL)
    if ok:
        print("RESULT: arc completed -- NOT reproduced. Try --from-stuck.")
    else:
        print("RESULT: arc did not reach its final waypoint -- REPRODUCED.")


if __name__ == "__main__":
    main()
