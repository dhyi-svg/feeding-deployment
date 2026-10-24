"""Replay a teleop keypoint demo relative to the microwave door (push-open, push-close, ...). Dry run unless --execute.

    python3 -u microwave/demo_replay.py encode demos/push_open_2026-10-07.json   # -> demos/push_open_2026-10-07.door.json
    python3 -u microwave/demo_replay.py run demos/push_open_2026-10-07.door.json [--trust-demo 3] [--execute]

A demo is recorded with `tools/kp.sh` (one keypoint per pose, arm teleoped in between) plus a
copy of the door file at that time (`<demo>_door.json`). `encode` turns it into DOOR coordinates,
so a replay adapts to where today's microwave is, how it's turned, how big it is and how high:

* horizontal, in the closed door's frame at the detected hinge: `a` = along the door (hinge ->
  handle), as a fraction of the hinge-to-handle distance (scales with the door's size); `b` =
  along the door's outward normal (toward the arm), in metres (clearance doesn't scale);
* height: metres above the door top for points that were above it, else metres from the grasp height;
* hand orientation relative to the door's facing.

`run` maps it onto today's door file (~/.microwave_door.json), checks the hand is at the demo's
first keypoint, sim-gates every leg (IK, joint jump, J4/J6, wrap, >= DEMO_MIN_CLEAR to the door
model; legs from --trust-demo on against the microwave body only -- the 10-07 door model put the
user's REAL push-open poses 2-7 cm inside the door), then (--execute) does the demo's gripper action
and follows the keypoints as one smooth trajectory.

--wrist-side right|left: hold the arm in the configuration with the wrist on that side of the elbow (as
seen from behind the arm) from the first keypoint on: a searched joint path that keeps the wrist >= 3 cm
on that side all the way, with the least joint motion (microwave_common.plan_side_path), and the PLANNED
joint path is what is sent. (Without it the arm re-solves IK from its current joints, which
takes the nearest configuration -- e.g. in the 10-07 close demo the elbow stays put while the hand goes
left, and the wrist ends up left of it.)
"""
import argparse, json, sys, time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R

import sheppy_arm
from door_push import DoorFrame, drop
from microwave_common import (DOOR_FILE, make_sim, plan_cartesian, plan_side_path, run_cartesian_trajectory,
                              trim_to_reach, wrist_offset)

OVER_TOP_MIN = 0.08          # points above the door top stay at least this far above today's top
START_TOL = 0.06             # the hand must be within this of the demo's first keypoint
DEMO_MIN_CLEAR = 0.02        # to the door model, on legs before --trust-demo
LIFT_OFF_MIN_CLEAR = 0.005   # first leg of a demo that starts holding the handle: the fingers straddle the bar, i.e.
                             # start next to the door face (as door_push.RETREAT_OUT_MIN_CLEAR)


def door_frame(door):
    """(hinge xy, e = hinge->handle along the closed door, n = outward normal, r = hinge-handle distance, yaw of n)."""
    h = np.asarray(door["hinge"][:2], float)
    e = np.asarray(door["closed_handle"][:2], float) - h
    r = float(np.linalg.norm(e))
    e /= r
    n = np.asarray(door["closed_normal"][:2], float)
    n -= e * float(n @ e)
    n /= np.linalg.norm(n)
    return h, e, n, r, float(np.arctan2(n[1], n[0]))


def encode(demo_path, adjust=None, start=0):
    """Keypoint demo (+ its _door.json) -> door-coordinate demo. `adjust`: {index: (da metres, db metres)}
    (indices of the RECORDED keypoints); `start`: drop the recorded keypoints before this one."""
    demo_path = Path(demo_path)
    demo = json.loads(demo_path.read_text())
    door = json.loads(demo_path.with_name(demo_path.stem + "_door.json").read_text())
    h, e, n, r, yaw = door_frame(door)
    top, zg = float(door["door_z"][1]), float(door["closed_grasp_pos"][2])
    out = []
    for i, k in enumerate(demo["keypoints"]):
        if i < start:
            continue
        p = np.asarray(k["tool"], float)
        d = p[:2] - h
        da, db = (adjust or {}).get(i, (0.0, 0.0))
        rel_q = (R.from_euler("z", -yaw) * R.from_quat(k["quat"])).as_quat()
        out.append({"label": k["label"], "a": float((d @ e + da) / r), "b": float(d @ n + db),
                    "z_ref": "top" if p[2] > top else "grasp",
                    "dz": float(p[2] - (top if p[2] > top else zg)), "quat_door": rel_q.tolist(),
                    "gripper": k["gripper"]})
    return {"source": demo_path.name, "door_open_deg": door.get("door_open_deg"), "hinge_to_handle_m": r,
            "keypoints": out, "adjust": {str(i): v for i, v in (adjust or {}).items()}, "start": start,
            "notes": demo.get("notes", [])}


def decode(enc, door):
    """Door-coordinate demo -> [(pos, quat)] on today's door."""
    h, e, n, r, yaw = door_frame(door)
    top, zg = float(door["door_z"][1]), float(door["closed_grasp_pos"][2])
    pts = []
    for k in enc["keypoints"]:
        xy = h + e * k["a"] * r + n * k["b"]
        z = (top + max(k["dz"], OVER_TOP_MIN)) if k["z_ref"] == "top" else zg + k["dz"]
        pts.append((np.r_[xy, z], (R.from_euler("z", yaw) * R.from_quat(k["quat_door"])).as_quat()))
    return pts


def plan(scene, rb, door, st, kps, trust_from, lift_off=False, wrist_side=None):
    """Sim-gated joint path through `kps` from the arm's state `st`: one list of joints per leg, or None.
    `wrist_side` ("right"/"left"): keep the wrist on that side of the elbow from the first keypoint on."""
    df = DoorFrame(door, door.get("open_sign", -1.0))
    q, legs = np.asarray(st["position"], float), []
    bodies = df.bodies(scene, rb, float(door.get("door_open_deg", 50.0)))
    try:
        pts = [(np.asarray(st["ee_pos"][:3]), np.asarray(st["ee_pos"][3:7]))] + kps
        if wrist_side:
            print(f"keeping the wrist {wrist_side} of the elbow; now {wrist_offset(scene, rb, q) * 100:+.0f} cm right of it")
            trusted = [trust_from is not None and i + 1 >= trust_from for i in range(len(kps))]
            return plan_side_path(
                scene, rb, pts, q, wrist_side,
                [[b for b in bodies if b[1] == "microwave body"] if t else bodies for t in trusted],
                [LIFT_OFF_MIN_CLEAR if (i == 0 and lift_off) else DEMO_MIN_CLEAR for i in range(len(kps))],
                "leg")
        for i in range(len(pts) - 1):
            trusted = trust_from is not None and i + 1 >= trust_from
            check = [b for b in bodies if b[1] == "microwave body"] if trusted else bodies
            clear = LIFT_OFF_MIN_CLEAR if (i == 0 and lift_off) else DEMO_MIN_CLEAR
            leg = plan_cartesian(scene, rb, pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1], q,
                                 f"{'trusted' if trusted else 'leg'} {i + 1}", check, min_clear=clear)
            if leg is None:
                return None
            legs.append(leg)
            q = leg[-1]
    finally:
        drop(rb, bodies)
    return legs


def leg_lengths(pts, legs):
    """Per-step travel (m, a turn counted like SheppyArm) for timing a planned joint path."""
    from sheppy_arm import ROT_EQUIV_M
    out = []
    for (p0, q0), (p1, q1), leg in zip(pts[:-1], pts[1:], legs):
        n = len(leg)
        ang = float((R.from_quat(q0).inv() * R.from_quat(q1)).magnitude())
        out += [max(1e-4, float(np.linalg.norm(np.asarray(p1) - p0)) / n, ang / n * ROT_EQUIV_M)] * n
    return out


def run(enc_path, execute, trust_from, ai=None, wrist_side=None, max_end_trim=0.0):
    """Replay one encoded demo. `ai`: an open SheppyArm to reuse (door_task chains steps); else one is made here.
    `wrist_side`: see the module doc. `max_end_trim` (m): if the last leg runs out of reach on today's door
    (e.g. the close's push when the microwave sits further out), pull its end back along the leg by up to this
    much. Returns True when it reached the last keypoint (or the dry run passed)."""
    enc = json.loads(Path(enc_path).read_text())
    door = json.loads(DOOR_FILE.read_text())
    pts = decode(enc, door)
    _, _, _, r, _ = door_frame(door)
    print(f"demo {enc['source']}: {len(pts)} keypoints; hinge-handle {enc['hinge_to_handle_m'] * 100:.1f} cm then, "
          f"{r * 100:.1f} cm now; door file says ~{door.get('door_open_deg', 0):.0f} deg open")
    for k, (p, _) in zip(enc["keypoints"], pts):
        print(f"  {k['label'][:30]:30s} -> {np.round(p, 3)}")
    return replay(pts, [k["gripper"] for k in enc["keypoints"]], door, execute, trust_from, ai, wrist_side,
                  max_end_trim)


def replay(pts, grippers, door, execute, trust_from, ai=None, wrist_side=None, max_end_trim=0.0):
    """`run` from the mapped keypoints on: `pts` [(pos, quat)] (the first = where the hand must be now), `grippers` the
    gripper reading at each, `door` the door dict the legs are checked against. Also what the fridge push-open runs
    (fridge_push.py: its keypoints are computed for today's door, not decoded from a recorded demo)."""
    from sheppy_arm import SheppyArm
    own = ai is None
    ai = ai or SheppyArm()
    try:
        st = ai.get_state()
        off = float(np.linalg.norm(np.asarray(st["ee_pos"][:3]) - pts[0][0]))
        print(f"hand now {np.round(st['ee_pos'][:3], 3)}, {off * 100:.1f} cm from the demo's start; gripper {st['gripper_pos']:.2f}")
        if off > START_TOL:
            print(f"not at the demo's start (> {START_TOL * 100:.0f} cm) -- refusing.")
            return False
        scene, rb = make_sim()
        if max_end_trim > 0:
            p0, q0 = pts[-2] if len(pts) > 1 else (np.asarray(st["ee_pos"][:3]), np.asarray(st["ee_pos"][3:7]))
            end, trim = trim_to_reach(scene, rb, p0, q0, pts[-1][0], pts[-1][1], wrist_side, max_end_trim)
            if end is None:
                print(f"the last leg is out of reach by more than {max_end_trim * 100:.0f} cm -- refusing.")
                return False
            if trim > 0:
                print(f"last keypoint out of reach: pulled back {trim * 100:.0f} cm along the last leg -> "
                      f"{np.round(end, 3)} (the push ends short of the demo's)")
                pts[-1] = (end, pts[-1][1])
        lift_off = grippers[0] > 0.2 and grippers[1] < 0.2
        legs = plan(scene, rb, door, st, pts[1:], trust_from, lift_off, wrist_side)
        if legs is None:
            print("a leg fails a gate -- refusing, arm untouched.")
            return False
        print(f"all {len(legs)} legs pass ({sum(len(l) for l in legs)} steps).")
        if not execute:
            print("\nDRY RUN -- nothing commanded.")
            return True
        g0, g1 = grippers[0], grippers[1]
        if g0 > 0.2 and g1 < 0.2:              # the demo let go of the handle first
            print("opening the gripper ...")
            ai.gripper(0.0)
            time.sleep(0.5)
        # The push-further-open CAN'T HANDLE MORE SPEED: always run it at the contact speed
        # (sheppy_arm.CONTACT_*_SPEED = 0.5 rad/s / 0.09 m/s), never the faster free-air speed -- in BOTH
        # branches below (the microwave takes the Cartesian `else`). 10-24: at 0.65/0.12 and 0.85/0.16 its
        # last leg (keypoint 4 -> 5: the hand pushing into the door, which pushes back, while turning 68 deg)
        # aborted PATH_TOLERANCE_VIOLATED ~12.5 cm short both times; it ran clean three times at 0.5 / 0.09.
        slow = sheppy_arm.contact_speed_scale()
        if wrist_side:
            # the planned joints themselves: re-solving IK on the arm would drift back to the nearest configuration
            path = [(np.asarray(st["ee_pos"][:3]), np.asarray(st["ee_pos"][3:7]))] + pts[1:]
            sent = ai.move_joints([qq for leg in legs for qq in leg], cart_lengths=leg_lengths(path, legs),
                                  speed_scale=slow)
            time.sleep(0.3)
            err = float(np.linalg.norm(np.asarray(ai.get_state()["ee_pos"][:3]) - pts[-1][0]))
            ok = sent and err < 0.01
        else:
            ok, err, _ = run_cartesian_trajectory(ai, [(list(p), list(q)) for p, q in pts[1:]], speed_scale=slow)
        print(f"{'DONE' if ok else 'STOPPED SHORT'}: hand {np.round(ai.get_state()['ee_pos'][:3], 3)} "
              f"({err * 100:.1f} cm from the last keypoint)")
        return bool(ok)
    finally:
        if own:
            ai.close()


def main():
    a = argparse.ArgumentParser()
    sub = a.add_subparsers(dest="cmd", required=True)
    e = sub.add_parser("encode")
    e.add_argument("demo")
    e.add_argument("--adjust", default="", help="'i:da:db,...' metres added at recorded keypoint i (along the door "
                   "hinge->handle, off its face toward the arm)")
    e.add_argument("--start", type=int, default=0, help="first recorded keypoint to keep")
    r_ = sub.add_parser("run")
    r_.add_argument("encoded")
    r_.add_argument("--execute", action="store_true")
    r_.add_argument("--trust-demo", type=int, default=None, metavar="LEG")
    r_.add_argument("--wrist-side", choices=["right", "left"], default=None,
                    help="hold the wrist on this side of the elbow (default: whatever IK gives)")
    r_.add_argument("--max-end-trim", type=float, default=0.0, metavar="M",
                    help="pull the last keypoint back along the last leg by up to M metres if out of reach")
    args = a.parse_args()
    if args.cmd == "encode":
        adj = {int(i): (float(da), float(db)) for i, da, db in
               (t.split(":") for t in args.adjust.split(",") if t)}
        enc = encode(args.demo, adj, args.start)
        out = Path(args.demo).with_suffix(".door.json")
        out.write_text(json.dumps(enc, indent=1))
        print(f"wrote {out}")
        for k in enc["keypoints"]:
            print(f"  {k['label'][:30]:30s} a {k['a']:+.3f} x r   b {k['b'] * 100:+6.1f} cm   z {k['dz'] * 100:+6.1f} cm from {k['z_ref']}")
    else:
        sys.exit(0 if run(args.encoded, args.execute, args.trust_demo, wrist_side=args.wrist_side,
                       max_end_trim=args.max_end_trim) else 1)


if __name__ == "__main__":
    main()
