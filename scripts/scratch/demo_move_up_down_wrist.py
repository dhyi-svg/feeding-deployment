"""DEMO -- nudge the arm up/down and rotate the wrist, in joint space.

This script exists to teach the codebase's arm-control path, not to do anything
useful. It does three "there-and-back" moves so the arm always returns to where
it started:

    1. shoulder joint  +step  -> settle -> back to start   (raises/lowers the hand)
    2. shoulder joint  -step  -> settle -> back to start
    3. wrist  joint    +step  -> settle -> back to start   (rolls the gripper)
    4. wrist  joint    -step  -> settle -> back to start

Everything is JOINT space: we send target joint-angle vectors and let the arm's
own controller interpolate. No IK, no Cartesian. On this arm Cartesian moves
(`set_ee_pose`) abort at extended configurations, and IK needs the PyBullet/IKFast
path that is broken on the Jetson -- joint-space point-to-point is the reliable
primitive here.

--------------------------------------------------------------------------------
HOW THE ARM CONTROL PATH IS WIRED (what this script is a tiny example of)
--------------------------------------------------------------------------------
  hardware          Kinova Gen3, 192.168.1.10, spoken to via the Kortex SDK
     |
  KinovaArm         src/feeding_deployment/control/robot_controller/kinova.py
     |              wraps Kortex: move_angular(), move_cartesian(), get_state(), ...
     |
  ArmInterface      .../robot_controller/arm_interface.py
     |              adds the safety latches (bulldog heartbeat, halt, e-stop) and
     |              a command log. Exposed over an RPC "manager" on port 5000.
     |
  arm_server.py     .../robot_controller/arm_server.py
     |              the ONE process that holds the Kortex session. Run it first.
     |
  ArmManager RPC    multiprocess manager, authkey b"secret-key", port 5000
     |              host = $ARM_RPC_HOST (set it to 127.0.0.1 on a single box)
     |
  this script       connects as a client, calls ArmInterface methods directly.
                    The "real" code path (ArmInterfaceClient in arm_client.py) adds
                    a ROS watchdog handshake and wraps commands in KinovaCommand
                    dataclasses -- we skip that here and call the interface raw,
                    exactly like scripts/session/*.py do.

Motion stays LOCKED until bulldog_bypass.py calls register_bulldog() and keeps a
heartbeat alive. So the required background processes are, in order:

    export ARM_RPC_HOST=127.0.0.1
    python src/feeding_deployment/control/robot_controller/arm_server.py   # holds the arm
    python scripts/stub_base_server.py                                     # bulldog wants a base
    python scripts/bulldog_bypass.py                                       # UNLOCKS motion + heartbeat
    python scripts/session/arm_set_speed.py low                            # always set speed explicitly

Then, in another terminal with the same env:

    python scripts/scratch/demo_move_up_down_wrist.py            # DRY RUN: prints, moves nothing
    python scripts/scratch/demo_move_up_down_wrist.py --execute  # actually moves

There is NO software e-stop with the bypass -- keep a hand on the physical e-stop.
To stop a running script: physical e-stop, or `python scripts/session/arm_halt.py`.
"""
import argparse
import sys
import time

import numpy as np

# The RPC connection constants live next to the interface. NUC_HOSTNAME honours
# $ARM_RPC_HOST (default 192.168.1.3, the lab NUC) -- set it to 127.0.0.1 here.
from feeding_deployment.control.robot_controller.arm_interface import (
    ARM_RPC_PORT,
    NUC_HOSTNAME,
    RPC_AUTHKEY,
    ArmManager,
)
from feeding_deployment.control.robot_controller.command_interface import JointCommand

# Gen3 7-DOF joint map (0-indexed), as returned by get_state()["position"]:
#   0  base yaw            rotates the whole arm left/right
#   1  shoulder pitch      MAIN up/down of the hand (also shifts reach a little)
#   2  upper-arm roll
#   3  elbow pitch         also affects hand height/reach
#   4  forearm roll
#   5  wrist pitch         tilts the gripper up/down
#   6  wrist roll          spins the gripper about its own axis  <- "rotate the wrist"
UPDOWN_JOINT = 1   # shoulder pitch
WRIST_JOINT = 6    # wrist roll

MAX_JOINT_JUMP_DEG = 20.0   # this demo never needs more; a bigger delta = refuse
SETTLE_TIMEOUT_S = 30.0     # how long to wait for a move to finish

parser = argparse.ArgumentParser(
    description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
)
parser.add_argument("--execute", action="store_true",
                    help="actually command the arm (default is a dry run)")
parser.add_argument("--updown-deg", type=float, default=5.0,
                    help="degrees to nudge the shoulder joint up and down (default 5)")
parser.add_argument("--wrist-deg", type=float, default=15.0,
                    help="degrees to rotate the wrist roll joint (default 15)")
args = parser.parse_args()

if max(args.updown_deg, args.wrist_deg) > MAX_JOINT_JUMP_DEG:
    sys.exit(f"step > {MAX_JOINT_JUMP_DEG} deg -- too big for this demo, refusing.")


def connect():
    """Attach to the running arm_server.py over its RPC manager."""
    ArmManager.register("ArmInterface")  # no callable: we are the client
    manager = ArmManager(address=(NUC_HOSTNAME, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
    manager.connect()
    return manager.ArmInterface()


def read(arm):
    """Return (joint_angles[7] rad, ee_xyz[3] m, gripper_pos) and print them."""
    st = arm.get_state()
    q = np.asarray(st["position"], dtype=float)
    ee = np.asarray(list(st["ee_pos"])[:3], dtype=float)
    grip = float(st["gripper_pos"])
    print(f"  joints (deg): {[round(float(np.degrees(v)), 1) for v in q]}")
    print(f"  EE xyz  (m) : [{ee[0]:+.4f}, {ee[1]:+.4f}, {ee[2]:+.4f}]   "
          f"gripper: {grip:.3f} ({'open' if grip < 0.2 else 'CLOSED'})")
    return q, ee, grip


def wait_until_stopped(arm):
    """Block until joint velocities are ~0 (a move can return before it settles)."""
    deadline = time.time() + SETTLE_TIMEOUT_S
    while time.time() < deadline:
        time.sleep(0.2)
        vel = np.asarray(arm.get_state()["velocity"], dtype=float)
        if float(np.max(np.abs(vel))) < 1e-3:
            return
    print("  WARNING: arm still moving after settle timeout")


def move_to(arm, target_q, label):
    """One joint-space move to target_q (7 rad), then wait and report."""
    print(f"\n-> {label}")
    if not args.execute:
        print(f"   (dry run) would send joints (deg): "
              f"{[round(float(np.degrees(v)), 1) for v in target_q]}")
        return
    # JointCommand validates shape == (7,); execute via the same entry point the
    # real ArmInterfaceClient.execute_command() uses under the hood.
    arm.set_joint_position(JointCommand(target_q).pos.tolist())
    wait_until_stopped(arm)
    read(arm)


def main():
    print("connecting to arm_server.py ...")
    arm = connect()

    # ---- refuse to run if the arm is not in a safe, ready state ----------------
    # (The RPC proxy only exposes methods, not attributes, so we check via is_halted();
    # an active e-stop surfaces as a clean exception from set_joint_position server-side.)
    if arm.is_halted():
        sys.exit("arm is HALTED -- run: python scripts/session/arm_halt.py --clear")

    print("\nstart state:")
    start_q, start_ee, grip = read(arm)

    if grip > 0.2:
        sys.exit("gripper is CLOSED -- it may be holding something. Refusing to move.")

    try:
        print(f"\nspeed preset: {arm.get_speed()}   (set with scripts/session/arm_set_speed.py)")
    except Exception as e:  # get_speed asserts if bulldog_bypass.py is not running yet
        print(f"\nspeed preset: <unavailable: {e}>")
        if args.execute:
            sys.exit("bulldog is not running -- start scripts/bulldog_bypass.py first.")
    if not args.execute:
        print("\n*** DRY RUN -- nothing will move. Re-run with --execute. ***")

    ud = np.radians(args.updown_deg)
    wr = np.radians(args.wrist_deg)

    # Build the four targets by copying the start vector and changing ONE joint.
    up_q = start_q.copy();     up_q[UPDOWN_JOINT] += ud
    down_q = start_q.copy();   down_q[UPDOWN_JOINT] -= ud
    wpos_q = start_q.copy();   wpos_q[WRIST_JOINT] += wr
    wneg_q = start_q.copy();   wneg_q[WRIST_JOINT] -= wr

    # Each nudge is followed by a return to start, so the arm ends where it began.
    # Watch the printed EE z to learn which sign of the shoulder joint is "up".
    move_to(arm, up_q,    f"shoulder joint {UPDOWN_JOINT}  +{args.updown_deg} deg")
    move_to(arm, start_q, "back to start")
    move_to(arm, down_q,  f"shoulder joint {UPDOWN_JOINT}  -{args.updown_deg} deg")
    move_to(arm, start_q, "back to start")
    move_to(arm, wpos_q,  f"wrist joint {WRIST_JOINT}  +{args.wrist_deg} deg")
    move_to(arm, start_q, "back to start")
    move_to(arm, wneg_q,  f"wrist joint {WRIST_JOINT}  -{args.wrist_deg} deg")
    move_to(arm, start_q, "back to start")

    if args.execute:
        print("\nfinal state:")
        final_q, final_ee, _ = read(arm)
        drift = np.degrees(final_q - start_q)
        print(f"\ndrift from start (deg): {[round(float(v), 2) for v in drift]}")
    print("\ndone.")


if __name__ == "__main__":
    main()
