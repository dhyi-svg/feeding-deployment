"""Placement motion on sheppy: carry the held container (or the tool) to the placement point, built on
Part 1 (microwave/xyz_move.py). cuRobo plans with HOLD_LEVEL so the container stays level the whole way,
and every leg is executed in JOINT IMPEDANCE (sheppy_arm.IMPEDANCE) through SheppyArm -> the arm driver's
/execute_joint_trajectory. In impedance the driver's path tolerance doubles as a force stop: a hand that
meets the microwave lags its reference, and the trajectory is aborted instead of pushed through.

    source microwave/sheppy/env.sh      # sheppy profile real-arm-camera-planner (arm driver + cuRobo planner)
    python3 -u microwave/placement/place_move.py --config microwave/placement/place_move.yaml            # dry run
    python3 -u microwave/placement/place_move.py --config microwave/placement/place_move.yaml --execute  # attended
    python3 -u microwave/placement/place_move.py --xyz 0.55 0.0 0.30 --start-joints <7 deg>   # no arm: plan only
Run from the repo root (the sim config path is relative).

STARTING STATE: the container is already in the gripper (--target container, the default), held at the
task roll -- a held box is never rolled (xyz_move refuses that), so only the HOLD_LEVEL move runs.

THE PLACEMENT POINT is a PLACEHOLDER until Part 2 (microwave perception) is wired in: `placement_point()`
takes it from --xyz / `xyz:` in the config (base_link, metres). Part 2 replaces that function's body; the
planning and the impedance execution below stay as they are.

What moves (all from xyz_move, unchanged): one cuRobo HOLD_LEVEL move to the point (with --target tool and
an empty gripper, first a J7-only roll to the task's roll: -90 = wrist camera on the LEFT seen from behind
the wrist, confirmed 10-23). Every leg is gated in sim before anything moves (reach, z band, J4/J6
guards, joint steps, distance from the straight line, held tilt, end pose); --execute also asks for `go`.

cuRobo's collision world does NOT contain the microwave, counter or user: the straight-line gate and the
impedance force stop are what protect the path -- clear the segment yourself until Part 2 adds the cavity.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))       # microwave/: xyz_move, sheppy_arm, ...
import xyz_move as xm  # noqa: E402

TASK_ROLL_DEG = -90.0               # wrist camera LEFT seen from behind the wrist (user-confirmed 10-23)
GRIPPER_HOLDING_MIN = 0.2           # gripper_pos below this = open = nothing held (= placement_workflow)


def placement_point(args):
    """PLACEHOLDER for Part 2. Returns the point (base_link, m) the `--target` (tool or container centre)
    goes to. Part 2 (perception of the open microwave) replaces this body; until then the point comes from
    --xyz / `xyz:` in the config."""
    if args.xyz is None:
        raise xm.Refused("no placement point: Part 2 (microwave perception) is not wired in yet -- "
                         "give --xyz or xyz: in the config")
    return np.asarray(args.xyz, float)


def plan_placement(sim, planner, q0, point, args, gates, log=print):
    """Part 1's legs (roll if needed, then the HOLD_LEVEL move) from joints q0 to `point`. Raises Refused."""
    return xm.plan_move(sim, planner, q0, point, "base", args.target, args.roll_deg, args.yaw_deg,
                        args.grasp_offset, gates, log=log)


def execute_placement(arm, sim, planner, legs, gates, speed_scale, log=print):
    """The gated legs, always in joint impedance (sheppy_arm.IMPEDANCE). Raises Refused on the first problem."""
    xm.execute_legs(arm, sim, planner, legs, gates, impedance=True, speed_scale=speed_scale, log=log)


def check_holding(arm, args):
    """With --target container the gripper must be closed on something."""
    g = float(arm.get_state()["gripper_pos"])
    if args.target == "container" and g < GRIPPER_HOLDING_MIN:
        raise xm.Refused(f"--target container but the gripper is open ({g:.2f}) -- nothing held")
    return g


def parse_args(argv=None):
    a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument("--config", type=Path, help="YAML with any of the options below (keys use _); the command line wins")
    a.add_argument("--xyz", type=float, nargs=3, metavar=("X", "Y", "Z"),
                   help="placement point, base_link m (PLACEHOLDER until Part 2 supplies it)")
    a.add_argument("--target", choices=("tool", "container"), default="container",
                   help="what goes to the point: the held container's centre (default) or the tool centre")
    a.add_argument("--grasp-offset", type=float, nargs=3, metavar=("X", "Y", "Z"),
                   help="tool frame -> held container centre, in the TOOL frame (m); required for --target container")
    a.add_argument("--roll-deg", type=float, default=TASK_ROLL_DEG, help="about the approach (see xyz_move.py)")
    a.add_argument("--yaw-deg", type=float, help="approach heading about base z (default: the current one)")
    a.add_argument("--max-deviation", type=float, default=xm.MAX_DEVIATION_M, help="m off the straight line")
    a.add_argument("--z-min", type=float, default=xm.Z_RANGE_M[0])
    a.add_argument("--z-max", type=float, default=xm.Z_RANGE_M[1])
    a.add_argument("--speed-scale", type=float, default=0.67, help="x SheppyArm's speed caps")
    a.add_argument("--start-joints", type=float, nargs=7, metavar="DEG",
                   help="plan from these joints instead of the arm's (no arm needed; never executes)")
    a.add_argument("--execute", action="store_true", help="command the arm (asks for `go`); omit for a dry run")
    args = a.parse_args(argv)
    if args.config:
        cfg = yaml.safe_load(args.config.read_text()) or {}
        unknown = set(cfg) - (set(vars(args)) - {"execute", "config"})   # --execute only on the command line
        if unknown:
            a.error(f"unknown keys in {args.config}: {sorted(unknown)}")
        a.set_defaults(**cfg)
        args = a.parse_args(argv)
    if args.start_joints is not None and args.execute:
        a.error("--start-joints plans from given joints: it never executes")
    if not 0.01 <= args.speed_scale <= 1.0:
        a.error("--speed-scale 0.01..1.0")
    return args


def main(argv=None, confirm=input):
    args = parse_args(argv)
    from microwave_common import make_sim
    sim = make_sim()
    arm = None
    if args.start_joints is not None:
        import threading

        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        rclpy.init()
        node = rclpy.create_node("place_move_planner")
        ex = SingleThreadedExecutor()
        ex.add_node(node)
        threading.Thread(target=ex.spin, daemon=True).start()
        q0 = np.radians(args.start_joints)
    else:
        from sheppy_arm import SheppyArm
        arm = SheppyArm(node_name="place_move")
        node, q0 = arm.node, arm.raw_joints()
    gates = xm.Gates(z_range=(args.z_min, args.z_max), max_deviation=args.max_deviation)
    try:
        if arm is not None:
            g = check_holding(arm, args)
            print(f"gripper {g:.3f}  joints {np.round(np.degrees(q0), 1).tolist()}")
        point = placement_point(args)
        print(f"placement point ({args.target}): {np.round(point, 3).tolist()} m, base_link")
        planner = xm.CuroboPlanner(node)
        legs = plan_placement(sim, planner, q0, point, args, gates)
        if not args.execute:
            print("\nDRY RUN -- planned and gated, nothing commanded. --execute runs it in joint impedance.")
            return 0
        if confirm(f"\n{len(legs)} leg(s), JOINT IMPEDANCE, e-stop in hand. Type go: ").strip() != "go":
            print("not go -- nothing commanded.")
            return 1
        execute_placement(arm, sim, planner, legs, gates, args.speed_scale)
        print("DONE -- at the placement point, container still held.")
        return 0
    except xm.Refused as e:
        print(f"REFUSED: {e}" + ("" if args.execute else " (dry run)"))
        return 2


if __name__ == "__main__":
    sys.exit(main())
