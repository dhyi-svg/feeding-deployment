"""Part 1 accuracy test on sheppy: park -> (user hand-places the arm) record -> park -> go to the recording.

    source microwave/sheppy/env.sh; cd ~/feeding-deployment-integration; export PYTHONPATH=$PWD/src:$PYTHONPATH
    python3 -u microwave/tools/part1_accuracy.py park              # cuRobo joint plan to microwave/park_pose.json
    python3 -u microwave/tools/part1_accuracy.py record            # save where the hand-placed arm is
    python3 -u microwave/tools/part1_accuracy.py goto [--impedance] # Part 1 (xyz_move.plan_move, HOLD_LEVEL) to it
    python3 -u microwave/tools/part1_accuracy.py status            # read only
Add --dry-run to plan and gate without moving. Recordings: ~/part1_accuracy/.

park: if J4 rests past the J4 guard (the folded retract leaves it at ~-152, outside Kinova's +-147.8 soft
limit, and cuRobo rejects that start), first J4 alone is moved just inside the guard. Then cuRobo
/rammp_curobo/plan_to_joints to the park joints (continuous joints taken the short way), every point
gated in sim FK (joint step, tool height, reach, J4/J6 guards past the start) before anything is sent.
goto: Part 1 unchanged -- xyz_move.plan_move to the recorded tool position with the recorded heading and
roll (HOLD_LEVEL holds park's tilt; a recording whose tilt differs is rolled first by Part 1's own legs or
refused), executed by xyz_move.execute_legs; then the error vs the recording is measured from /ee_state.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import xyz_move as xm  # noqa: E402
from microwave_common import CONTINUOUS, J4_GUARD_DEG, J6_GUARD_DEG, short_delta  # noqa: E402

PARK_FILE = HERE.parent / "park_pose.json"
OUT = Path.home() / "part1_accuracy"
MAX_STEP_DEG = 10.0
MIN_TOOL_Z = 0.10
MAX_REACH = 0.91
END_TOL_DEG = 0.5


def deg(q):
    return np.round(np.degrees(q), 1).tolist()


class JointPlanner:
    def __init__(self, node):
        from rclpy.action import ActionClient
        from rammp_curobo_interfaces.action import PlanToJoints
        self._Goal = PlanToJoints.Goal
        self._client = ActionClient(node, PlanToJoints, "/rammp_curobo/plan_to_joints")

    def plan(self, start, target):
        if not self._client.wait_for_server(timeout_sec=5.0):
            raise xm.Refused("no /rammp_curobo/plan_to_joints (planner node up?)")
        g = self._Goal()
        g.start_joints, g.target_joints = [float(v) for v in start], [float(v) for v in target]
        gh = xm._wait(self._client.send_goal_async(g), 10.0)
        if gh is None or not gh.accepted:
            raise xm.Refused("cuRobo refused the joint goal")
        res = xm._wait(gh.get_result_async(), 30.0)
        if res is None or not res.result.success:
            raise xm.Refused(f"cuRobo: {res.result.message if res else 'no answer in 30 s'}")
        names = list(res.result.trajectory.joint_names)
        idx = [names.index(f"joint_{i}") for i in range(1, 8)]
        qs = [np.asarray(p.positions, float)[idx] for p in res.result.trajectory.points]
        out, prev = [], np.asarray(start, float)
        for k in qs:
            d = k - prev
            d[CONTINUOUS] = (d[CONTINUOUS] + np.pi) % (2 * np.pi) - np.pi
            prev = prev + d
            out.append(prev.copy())
        return np.array(out)


def gate_joint_path(sim, qs, target, name, guard_from=0):
    worst_z, worst_step = 9.0, 0.0
    for i, q in enumerate(qs):
        pos, _ = xm.fk(sim, q)
        bad = []
        worst_z = min(worst_z, pos[2])
        if pos[2] < MIN_TOOL_Z:
            bad.append(f"tool z {pos[2]:.3f} < {MIN_TOOL_Z}")
        if np.linalg.norm(pos) > MAX_REACH:
            bad.append(f"reach {np.linalg.norm(pos):.3f}")
        if i >= guard_from and (abs(np.degrees(q[3])) > J4_GUARD_DEG or abs(np.degrees(q[5])) > J6_GUARD_DEG):
            bad.append(f"J4 {np.degrees(q[3]):.1f} / J6 {np.degrees(q[5]):.1f} past the guards")
        if i:
            step = float(np.degrees(np.max(np.abs(short_delta(qs[i - 1], q)))))
            worst_step = max(worst_step, step)
            if step > MAX_STEP_DEG:
                bad.append(f"joint step {step:.1f} deg")
        if bad:
            raise xm.Refused(f"{name}: point {i + 1}/{len(qs)}: {'; '.join(bad)}")
    end = float(np.degrees(np.max(np.abs(short_delta(qs[-1], target)))))
    if end > END_TOL_DEG:
        raise xm.Refused(f"{name}: ends {end:.2f} deg from the target joints")
    print(f"{name}: {len(qs)} pts gated -- max joint step {worst_step:.1f} deg, lowest tool z {worst_z:.3f} m, "
          f"ends {end:.2f} deg from target")


def connect():
    from sheppy_arm import SheppyArm
    arm = SheppyArm(node_name="part1_accuracy")
    time.sleep(0.5)
    return arm


def show(arm, label="now"):
    s = arm.get_state()
    q = arm.raw_joints()
    e = np.asarray(s["ee_pos"], float)
    print(f"{label}: joints {deg(q)} deg | tool {np.round(e[:3], 4).tolist()} m | gripper {s['gripper_pos']:.3f}")
    return q, e


def do_park(arm, sim, speed, dry, park=None):
    """To `park` (raw joints, rad), default park_pose.json."""
    park = np.asarray(json.loads(PARK_FILE.read_text())["joints"] if park is None else park, float)
    q = arm.raw_joints()
    target = q + short_delta(q, park)
    target[[1, 3, 5]] = park[[1, 3, 5]]
    print(f"target {deg(target)} deg")
    if abs(np.degrees(q[3])) > J4_GUARD_DEG:
        qn = q.copy()
        qn[3] = np.sign(q[3]) * np.radians(J4_GUARD_DEG - 1.0)
        p0, _ = xm.fk(sim, q)
        p1, _ = xm.fk(sim, qn)
        print(f"J4 at {np.degrees(q[3]):.1f} is past the {J4_GUARD_DEG} guard: J4 alone -> {np.degrees(qn[3]):.1f} "
              f"(tool moves {np.linalg.norm(p1 - p0) * 100:.1f} cm, to z {p1[2]:.3f})")
        if p1[2] < MIN_TOOL_Z:
            raise xm.Refused("that J4 move would take the tool too low")
        if not dry:
            ok = arm.move_joints([qn], speed_scale=0.3)
            time.sleep(0.3)
            if not ok:
                raise xm.Refused("the driver refused/stopped the J4 move (it may reject a start past the soft "
                                 "limit) -- move J4 inside +-147 by hand")
            q = arm.raw_joints()
            print(f"J4 now {np.degrees(q[3]):.1f}")
        else:
            q = qn
    qs = JointPlanner(arm.node).plan(q, target)
    gate_joint_path(sim, qs, target, "to park")
    if dry:
        print("DRY RUN -- nothing sent")
        return
    ok = arm.move_joints(list(qs[1:]), speed_scale=speed)
    time.sleep(0.3)
    show(arm, "after park")
    off = float(np.degrees(np.max(np.abs(short_delta(arm.raw_joints(), target)))))
    print(f"{'AT PARK' if ok and off < 1.0 else 'STOPPED'}: {off:.2f} deg max joint error")


def do_record(arm):
    q, e = show(arm, "recorded")
    OUT.mkdir(exist_ok=True)
    rec = {"time": time.strftime("%Y-%m-%d %H:%M:%S"), "joints": q.tolist(), "tool_pos": e[:3].tolist(),
           "tool_quat": e[3:7].tolist()}
    rot = R.from_quat(e[3:7])
    rec["yaw_deg"], rec["roll_deg"] = xm.approach_yaw_deg(rot), xm.roll_of(rot)
    p = OUT / f"recorded_{time.strftime('%Y%m%d_%H%M%S')}.json"
    p.write_text(json.dumps(rec, indent=1))
    (OUT / "latest.json").write_text(json.dumps(rec, indent=1))
    print(f"saved {p} (heading {rec['yaw_deg']}, roll {rec['roll_deg']}; {xm.describe(rot)})")


def do_goto(arm, sim, speed, impedance, dry, path):
    rec = json.loads(Path(path).read_text())
    goal = np.asarray(rec["tool_pos"], float)
    rot_rec = R.from_quat(rec["tool_quat"])
    q0 = arm.raw_joints()
    _, rot0 = xm.fk(sim, q0)
    print(f"recorded tool {np.round(goal, 4).tolist()}, heading {rec['yaw_deg']}, roll {rec['roll_deg']}; "
          f"tilt vs now {xm.tilt_deg(rot0, rot_rec):.1f} deg")
    if rec["yaw_deg"] is None or rec["roll_deg"] is None:
        raise xm.Refused("the recorded approach is ~vertical: Part 1 (horizontal-gripper targets) cannot express it")
    planner = xm.CuroboPlanner(arm.node)
    legs = xm.plan_move(sim, planner, q0, goal, "base", "tool", rec["roll_deg"], rec["yaw_deg"])
    rot_goal = legs[-1].goal_rot
    print(f"Part 1's goal orientation is {np.degrees((rot_goal * rot_rec.inv()).magnitude()):.2f} deg from the "
          "recorded one (Part 1 targets a level approach; any pitch in the recording is not reproduced)")
    if dry:
        print("DRY RUN -- nothing sent")
        return
    xm.execute_legs(arm, sim, planner, legs, impedance=impedance, speed_scale=speed)
    time.sleep(1.0)
    q, e = show(arm, "arrived")
    perr = float(np.linalg.norm(e[:3] - goal))
    print(f"\nACCURACY vs the recording ({'impedance' if impedance else 'position'}):")
    print(f"  position error {perr * 1000:.1f} mm  (dx,dy,dz mm {np.round((e[:3] - goal) * 1000, 1).tolist()})")
    print(f"  orientation error {np.degrees((R.from_quat(e[3:7]) * rot_rec.inv()).magnitude()):.2f} deg vs recorded, "
          f"{np.degrees((R.from_quat(e[3:7]) * rot_goal.inv()).magnitude()):.2f} deg vs Part 1's goal")
    print(f"  joints vs recorded (deg) {np.round(np.degrees(short_delta(np.asarray(rec['joints']), q)), 1).tolist()}")
    res = {"time": time.strftime("%Y-%m-%d %H:%M:%S"), "recording": str(path), "impedance": impedance,
           "arrived_tool": e.tolist(), "arrived_joints": q.tolist(), "pos_err_m": perr}
    (OUT / f"result_{time.strftime('%Y%m%d_%H%M%S')}.json").write_text(json.dumps(res, indent=1))


def main(argv=None):
    a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument("step", choices=("status", "park", "record", "goto"))
    a.add_argument("--recording", type=Path, default=OUT / "latest.json")
    a.add_argument("--impedance", action="store_true", help="goto in joint impedance (default position)")
    a.add_argument("--speed-scale", type=float, default=0.5)
    a.add_argument("--dry-run", action="store_true")
    args = a.parse_args(argv)
    arm = connect()
    try:
        if args.step == "status":
            show(arm)
        elif args.step == "record":
            do_record(arm)
        else:
            from microwave_common import make_sim
            sim = make_sim()
            if args.step == "park":
                do_park(arm, sim, args.speed_scale, args.dry_run)
            else:
                do_goto(arm, sim, args.speed_scale, args.impedance, args.dry_run, args.recording)
    except xm.Refused as e:
        print(f"REFUSED: {e}")
        return 2
    finally:
        arm.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
