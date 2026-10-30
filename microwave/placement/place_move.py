"""Place the OXO container in the open microwave: Part 2 says where, Part 1 moves there, impedance lowers it.

    source microwave/sheppy/env.sh; cd ~/feeding-deployment-integration; export PYTHONPATH=$PWD/src:$PYTHONPATH
    # sheppy profile real-arm-camera-planner, door open, gripper empty, e-stop in hand
    python3 -u microwave/placement/place_move.py                     # dry run: perceive + plan, nothing commanded
    python3 -u microwave/placement/place_move.py --execute           # asks `go` before each moving stage
    python3 -u microwave/placement/place_move.py --execute --stop-before-lower   # stops above the floor
    python3 -u microwave/placement/place_move.py --recorded f1 f2 f3 --mask m1 m2 m3 --start-joints <7 deg>   # no arm

1. staging  cuRobo joint plan to `staging_joints_deg` (part1_accuracy.do_park, its gates), empty hand.
2. perceive perceive_placement.py in its own process: SAM 3 exits before the arm moves again. Placement point,
            insertion axis, cavity; everything after this is planned and gated before anything else moves.
3. roll     J7 alone to `roll_deg`, in place.
4. load     wait for the user to put the container in, close the gripper, confirm the grip.
5. transit  cuRobo HOLD_LEVEL to the pre-insert point on the insertion axis, in front of the opening (position).
6. insert   straight Cartesian line through the opening to above the placement point (position, watched).
7. lower    lower.py: impedance until floor contact, then hold. NO release and NO retract yet.

cuRobo's collision world does not contain the microwave: the insertion is a checked straight line along the
measured axis, with the container's clearance to the opening computed from the measured cavity. Nothing feels
contact during 5-6 (position control, and no wrench on this driver): a guard stops the hand if it leaves the line.
Frames: arm_base_link, metres. Tool = Kinova tool_frame; container centre = tool + R @ grasp_offset.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parent), str(HERE.parent / "tools")]
import floor_target as ft  # noqa: E402
import lower as lw  # noqa: E402
import sheppy_arm  # noqa: E402
import xyz_move as xm  # noqa: E402
from microwave_common import short_delta  # noqa: E402
from rgbd_source import BASE_FRAME  # noqa: E402
from xyz_move import Refused  # noqa: E402

GRIPPER_HOLDING_MIN = 0.2           # gripper_pos below this = open = nothing held
DEFAULT_CONFIG = HERE / "place_move.yaml"
STAGING_TOL_DEG = 1.0
DEFAULTS = dict(
    staging_joints_deg=None, cavity_method="opening", frames=3, roll_deg=-90.0,
    insert_above_m=0.07, pre_gap_m=0.05, door_clear_m=0.02, side_clear_m=0.02, top_clear_m=0.02,
    transit_speed_scale=0.4, insert_speed_scale=0.3, insert_max_dev_m=0.01, overtravel_m=0.03,
    settle_s=3.0, lower_speed=0.02, vz_stop=-0.01, hold_press_m=0.003, early_stop_m=0.03, max_settle_sag_m=0.06,
    min_settle_gap_m=0.005, stall_m=0.02, max_xy_dev_m=0.02, max_tilt_deg=3.0, kq=None, zeta=None, torque_limit=None)


def load_config(path=None):
    cfg = dict(DEFAULTS)
    path = Path(path) if path else DEFAULT_CONFIG
    data = yaml.safe_load(path.read_text()) or {}
    unknown = set(data) - set(cfg)
    if unknown:
        raise ValueError(f"unknown keys in {path}: {sorted(unknown)}")
    cfg.update(data)
    if cfg["staging_joints_deg"] is None or len(cfg["staging_joints_deg"]) != 7:
        raise ValueError(f"staging_joints_deg (7 joints, deg) must be set in {path}")
    return cfg


def perceive_process(out, cfg, recorded=None, masks=None):
    """perceive_placement.py in a child process, so SAM 3 is gone when this returns. Returns its result dict."""
    cmd = [sys.executable, "-u", str(HERE / "perceive_placement.py"), "--cavity-method", cfg["cavity_method"], "--out", str(out)]
    cmd += ["--recorded", *map(str, recorded)] if recorded else ["--live", "--frames", str(cfg["frames"])]
    if masks:
        cmd += ["--mask", *map(str, masks)]
    rc = subprocess.run(cmd, check=False).returncode
    path = Path(out) / "result.json"
    res = json.loads(path.read_text()) if path.exists() else {"ok": False, "reason": f"perception exit {rc}"}
    if not res.get("ok"):
        raise Refused(f"perception: {res.get('reason')}")
    return res


def insertion_points(res, spec, cfg):
    """Tool points of the place, checked against the measured cavity. Refused if the container cannot pass the opening."""
    if res["frame"] != BASE_FRAME:
        raise Refused(f"the target is in {res['frame']}, Part 1 works in {BASE_FRAME}")
    if res["grasp_offset"] is None:
        raise Refused("container.grasp_offset is not set")
    c, f = res["cavity"], np.asarray(res["insertion_axis"], float)
    l, up = np.asarray(c["l"], float), np.array([0.0, 0.0, 1.0])
    tool = np.asarray(res["tool_center"], float)
    centre = np.asarray(res["container_center"], float)
    lift = cfg["insert_above_m"]
    bottom = res["support_z"] + lift + (centre[2] - spec.height_m / 2 - res["support_z"])
    top, lip = bottom + spec.height_m, c["floor_z"]
    half = spec.width_m / 2
    side = float(centre @ l)
    if bottom < lip + cfg["door_clear_m"]:
        raise Refused(f"container bottom {bottom:.3f} clears the lip {lip:.3f} by less than {cfg['door_clear_m'] * 100:.0f} cm")
    if top > c["top"] - cfg["top_clear_m"]:
        raise Refused(f"container top {top:.3f} too close to the opening's top {c['top']:.3f}")
    if side - half < c["right"] + cfg["side_clear_m"] or side + half > c["left"] - cfg["side_clear_m"]:
        raise Refused(f"container [{side - half:.3f}, {side + half:.3f}] too close to the opening's sides "
                      f"[{c['right']:.3f}, {c['left']:.3f}]")
    finger = spec.finger_below_tool_m
    if finger is not None and tool[2] + lift - finger < lip + cfg["door_clear_m"]:
        raise Refused(f"lower finger z {tool[2] + lift - finger:.3f} too close to the lip {lip:.3f}")
    above = tool + up * lift
    pull = (centre @ f + spec.length_m / 2) - (c["front"] - cfg["pre_gap_m"])       # far end stops pre_gap short
    pre = above - f * pull
    contact_z = float(tool[2] - (centre[2] - spec.height_m / 2 - res["support_z"]))
    return dict(pre=pre, above=above, end=tool * [1, 1, 0] + up * (contact_z - cfg["overtravel_m"]), contact_z=contact_z,
                clear=dict(lip=bottom - lip, top=c["top"] - top, right=side - half - c["right"], left=c["left"] - side - half))


def straight_line(sim, q, goal, rot, gates, name):
    """Gated straight Cartesian joint path from joints q to `goal`, orientation `rot`: (joints incl. q, lengths)."""
    p0, rot0 = xm.fk(sim, q)
    knots, lengths = sheppy_arm.cartesian_knots(sim, np.concatenate([p0, rot0.as_quat()]), q, [(goal, rot.as_quat())])
    if knots is None:
        raise Refused(f"{name}: no straight path to {np.round(goal, 3).tolist()}")
    line = np.vstack([np.asarray(q, float)[None, :], *[np.asarray(k)[None, :] for k in knots]])
    stats = xm.gate_path(sim, line, goal, rot, True, gates, name)
    return line, lengths, stats


def line_guard(arm, a, b, rot, max_dev, max_tilt, why):
    """A move_joints monitor: True (stop) when the measured tool leaves the segment a-b or tilts."""
    def monitor():
        ee = np.asarray(arm.get_state()["ee_pos"], float)
        dev, tilt = xm.seg_distance(ee[:3], a, b), xm.tilt_deg(R.from_quat(ee[3:7]), rot)
        if dev > max_dev or tilt > max_tilt:
            why.append(f"{dev * 100:.1f} cm off the line, {tilt:.1f} deg tilt")
            return True
        return False
    return monitor


def plan_place(sim, planner, q, res, spec, cfg, log=print):
    """Everything from the staging joints q: roll, transit, insertion and descent, all gated. Moves nothing."""
    pts = insertion_points(res, spec, cfg)
    yaw, rot = res["yaw_deg"], xm.target_orientation(res["yaw_deg"], cfg["roll_deg"])
    log(f"container clearance through the opening: lip {pts['clear']['lip'] * 100:.1f}, top {pts['clear']['top'] * 100:.1f}, "
        f"right {pts['clear']['right'] * 100:.1f}, left {pts['clear']['left'] * 100:.1f} cm")
    gates, line_gates = xm.Gates(), xm.Gates(max_deviation=cfg["insert_max_dev_m"])
    rot_q = xm.fk(sim, q)[1]             # J7 only rolls, at the staging heading; the transit turns to `yaw`
    qr, left = xm.j7_roll(sim, q, xm.target_orientation(xm.approach_yaw_deg(rot_q), cfg["roll_deg"]))
    roll = None
    if qr is not None:
        if left > xm.ROLL_MATCH_DEG:
            raise Refused(f"the staging pose is {left:.1f} deg from level: J7 alone cannot reach the task orientation")
        n = max(2, int(np.ceil(abs(qr[6] - q[6]) / np.radians(gates.max_step_deg / 2))) + 1)
        roll = xm.Leg("roll", np.linspace(q, qr, n), xm.fk(sim, q)[0], rot, {}, None)
        roll.stats = xm.gate_path(sim, roll.joints, roll.goal_pos, xm.fk(sim, qr)[1], False, gates, "roll")
        log(f"roll: J7 {np.degrees(qr[6] - q[6]):+.1f} deg in place")
    else:
        qr = q
    transit = xm.plan_move(sim, planner, qr, pts["pre"], "base", "tool", cfg["roll_deg"], yaw, None, gates, log=log)
    q_pre = transit[-1].joints[-1]
    ins, ins_len, ins_s = straight_line(sim, q_pre, pts["above"], rot, line_gates, "insert")
    log(f"insert: {sum(ins_len) * 100:.1f} cm straight, <= {ins_s['deviation_m'] * 1000:.1f} mm off the line, tilt "
        f"{ins_s['tilt_deg']:.2f} deg")
    down, down_len, down_s = straight_line(sim, ins[-1], pts["end"], rot, line_gates, "lower")
    log(f"lower: {sum(down_len) * 100:.1f} cm line from tool z {pts['above'][2]:.3f}, expected contact at {pts['contact_z']:.3f}, "
        f"tilt {down_s['tilt_deg']:.2f} deg")
    return dict(pts=pts, rot=rot, roll=roll, transit=transit, gates=gates, line_gates=line_gates)


def run(arm, sim, planner, cfg, perceive, spec, execute, confirm, log_dir, q_start=None, stop_before_lower=False,
        to_staging=None, log=print):
    """The workflow. arm None = plan only from q_start. Returns True when it ended at the planned end."""
    staging = np.radians(cfg["staging_joints_deg"])

    def ask(what):
        if confirm(f"\n{what} Type go: ").strip() != "go":
            raise Refused("not go -- stopped here")

    if arm is not None:
        g = float(arm.get_state()["gripper_pos"])
        if g >= GRIPPER_HOLDING_MIN:
            raise Refused(f"the gripper is closed ({g:.2f}): this starts with an empty, open hand")
        off = float(np.degrees(np.max(np.abs(short_delta(arm.raw_joints(), staging)))))
        if off > STAGING_TOL_DEG:
            if execute:
                ask(f"Move to staging ({off:.0f} deg of joint travel), position control.")
            to_staging(not execute)
            if not execute:
                raise Refused("dry run: not at the staging pose, so perception and the rest would be from the wrong view")
        q_start = arm.raw_joints()
    res = perceive()
    log(f"placement point (container centre) {np.round(res['container_center'], 3).tolist()}, tool "
        f"{np.round(res['tool_center'], 3).tolist()}, yaw {res['yaw_deg']:+.1f}, support z {res['support_z']:.3f}")
    plan = plan_place(sim, planner, np.asarray(q_start, float), res, spec, cfg, log)
    pts, rot = plan["pts"], plan["rot"]
    if not execute:
        log("\nDRY RUN -- perceived, planned and gated, nothing commanded.")
        return True

    # roll, then load the container
    if plan["roll"] is not None:
        ask("Roll J7 in place.")
        xm.execute_legs(arm, sim, planner, [plan["roll"]], plan["gates"], speed_scale=0.5, log=log)
    ask("Put the container between the fingers and hold it. Closing the gripper next.")
    arm.gripper(1.0)
    g = float(arm.get_state()["gripper_pos"])
    if g < GRIPPER_HOLDING_MIN:
        raise Refused(f"gripper only at {g:.2f}: nothing held")
    ask(f"Gripper at {g:.2f}, which cannot prove a grip. Check it by hand, let go of the container.")

    # transit + straight in. Replanned from the joints the arm really has.
    q = arm.raw_joints()
    transit = xm.plan_move(sim, planner, q, pts["pre"], "base", "tool", cfg["roll_deg"], res["yaw_deg"], None, plan["gates"], log=log)
    ask("Carry to the opening, then straight in. Position control, nothing feels contact: watch it.")
    xm.execute_legs(arm, sim, planner, transit, plan["gates"], speed_scale=cfg["transit_speed_scale"], log=log)
    time.sleep(0.3)
    line, lengths, _ = straight_line(sim, arm.raw_joints(), pts["above"], rot, plan["line_gates"], "insert")
    why = []
    ok = arm.move_joints(list(line[1:]), cart_lengths=lengths, speed_scale=cfg["insert_speed_scale"],
                         monitor=line_guard(arm, xm.fk(sim, line[0])[0], pts["above"], rot, 2 * cfg["insert_max_dev_m"],
                                            cfg["max_tilt_deg"], why))
    if why or not ok:
        raise Refused(f"STOPPED in the insertion: {why[0] if why else 'the driver stopped it'} -- container still held")
    time.sleep(0.3)
    if stop_before_lower:
        log("Stopped above the floor as asked, position control, container held.")
        return True

    # lower until contact
    line, lengths, _ = straight_line(sim, arm.raw_joints(), pts["end"], rot, plan["line_gates"], "lower")
    ask("Lower in IMPEDANCE until floor contact.")
    lw.lower(arm, sim, line, lengths, pts["contact_z"], rot, cfg, log, Path(log_dir))
    ee = np.asarray(arm.get_state()["ee_pos"], float)
    log(f"PLACED: floor contact, tool {np.round(ee[:3], 3).tolist()}, tilt {xm.tilt_deg(R.from_quat(ee[3:7]), rot):.1f} deg. "
        "Container held, arm in impedance. Not released, not retracted.")
    return True


def parse_args(argv=None):
    a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument("--config", type=Path, help=f"default {DEFAULT_CONFIG.name}")
    a.add_argument("--execute", action="store_true", help="command the arm (asks go per stage); omit for a dry run")
    a.add_argument("--stop-before-lower", action="store_true", help="stop above the floor, no impedance")
    a.add_argument("--recorded", nargs="+", type=Path, help="frames instead of the live camera; never executes")
    a.add_argument("--mask", nargs="+", type=Path, help="saved masks for --recorded (skips SAM 3)")
    a.add_argument("--start-joints", type=float, nargs=7, metavar="DEG", help="plan from these (no arm); never executes")
    args = a.parse_args(argv)
    if args.execute and (args.recorded or args.start_joints is not None):
        a.error("--recorded / --start-joints never execute")
    return args


def main(argv=None, confirm=input):
    args = parse_args(argv)
    cfg = load_config(args.config)
    spec = ft.load_perception_config().container
    out = Path.home() / "microwave_place_logs" / f"place_{time.strftime('%Y%m%d_%H%M%S')}"
    from microwave_common import make_sim
    sim = make_sim()
    arm, q_start, to_staging = None, None, None
    if args.start_joints is not None:
        import threading

        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        rclpy.init()
        node = rclpy.create_node("place_move_planner")
        ex = SingleThreadedExecutor()
        ex.add_node(node)
        threading.Thread(target=ex.spin, daemon=True).start()
        q_start = np.radians(args.start_joints)
    else:
        import part1_accuracy as pa
        from sheppy_arm import SheppyArm
        arm = SheppyArm(node_name="place_move")
        node = arm.node
        to_staging = lambda dry: pa.do_park(arm, sim, 0.3, dry, np.radians(cfg["staging_joints_deg"]))  # noqa: E731
    planner = xm.CuroboPlanner(node)
    try:
        run(arm, sim, planner, cfg, lambda: perceive_process(out / "perception", cfg, args.recorded, args.mask), spec,
            args.execute, confirm, out, q_start, args.stop_before_lower, to_staging)
        return 0
    except Refused as e:
        print(f"REFUSED: {e}" + ("" if args.execute else " (dry run)"))
        return 2


if __name__ == "__main__":
    sys.exit(main())
