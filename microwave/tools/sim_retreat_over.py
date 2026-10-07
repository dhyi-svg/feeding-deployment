"""Sim-only: route variants for getting OVER the open door after the swing (user 10-07: up and over,
not back toward the base; diagonals fine, but don't head into the microwave). No arm, no camera.

    python3 microwave/tools/sim_retreat_over.py              # from ~/.microwave_door.json + the 10-07 view pose

Replays the grasp (25-deg tilt) + swing from the door file, then for every variant chains
plan_cartesian (IK / jump / J4 / J6 / wrap gates, clearance to the door model at its angle + the
microwave body) through the corners and prints the first failing gate.
"""
import argparse, contextlib, io, json, sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # microwave/
sys.path.insert(0, str(Path(__file__).resolve().parent))
import numpy as np
from scipy.spatial.transform import Rotation as R

import door_push as dp
import real_gen3_ros2_grasp_and_swing_microwave as mw
from handle_detect import HANDLE_QUAT
from microwave_common import DOOR_FILE, make_sim, plan_cartesian
from sim_push_open import VIEW_Q_DEG, fk_tool


def main():
    a = argparse.ArgumentParser()
    a.add_argument("--tilt", type=float, default=25.0)
    a.add_argument("--swing", type=int, default=75)
    a.add_argument("--above", type=float, default=0.12, help="clearance over the detected door top (m)")
    a.add_argument("--out", type=float, default=0.05, help="first move off the handle along -approach (m)")
    a.add_argument("--xs", default="0.40,0.45,0.50", help="x of the across / turn / down legs to try")
    args = a.parse_args()

    door = json.loads(DOOR_FILE.read_text())
    scene, rb = make_sim()
    h, n = np.array(door["closed_handle"]), np.array(door["closed_normal"])
    q_view = np.radians(VIEW_Q_DEG)
    ee_view = fk_tool(scene, rb, q_view)
    with contextlib.redirect_stdout(io.StringIO()):
        _, g = mw.grasp_pose(h - np.array([0, 0, mw.VERTICAL_CORR]), HANDLE_QUAT, n, args.tilt)
        leg = plan_cartesian(scene, rb, ee_view[:3], ee_view[3:], np.array(g.position), np.array(g.orientation),
                             q_view, "g")
    ok, q0, msg = mw._check_arc(scene, rb, mw._swing_waypoints(g, np.array(door["hinge"]), args.swing), leg[-1])
    if not ok:
        sys.exit(f"swing fails: {msg}")
    ee = fk_tool(scene, rb, q0)
    ang = dp.DoorFrame(door, 1.0).angle_of(ee)
    df = dp.DoorFrame(door, np.sign(ang) or 1.0)
    deg = abs(ang)
    quat0 = tuple(ee[3:])
    approach = R.from_quat(quat0).as_matrix()[:, 2]
    p_out = ee[:3] - approach * args.out
    z_over = float(door["door_z"][1]) + args.above
    z_down = float(door["closed_grasp_pos"][2])
    y_mid = float(door["door_mid"][1])
    q_face = tuple(dp._facing_forward(quat0, np.r_[-df.n, 0.0]))
    print(f"end of swing: door {deg:.0f} deg, hand {np.round(ee[:3], 3)}, J {np.round(np.degrees(q0), 1)}")
    print(f"out {np.round(p_out, 3)}, over at z {z_over:.3f}, down to z {z_down:.3f} at y {y_mid:.3f}; "
          f"microwave front (closed face) x ~{np.dot(door['closed_grasp_pos'][:2], [1, 0]) + 0.05:.2f}")

    variants = {}
    for x in [float(v) for v in args.xs.split(",")]:
        up, over, down = np.r_[p_out[:2], z_over], np.array([x, y_mid, z_over]), np.array([x, y_mid, z_down])
        variants[f"x {x:.2f}  A: up / across / turn / down"] = [(ee[:3], quat0), (p_out, quat0), (up, quat0),
                                                            (over, quat0), (over, q_face), (down, q_face)]
        variants[f"x {x:.2f}  B: up / across / turn WHILE down"] = [(ee[:3], quat0), (p_out, quat0), (up, quat0),
                                                                (over, quat0), (down, q_face)]
        variants[f"x {x:.2f}  C: up+across diagonal / turn while down"] = [(ee[:3], quat0), (p_out, quat0),
                                                                       (over, quat0), (down, q_face)]
    bodies = df.bodies(scene, rb, deg)
    try:
        for name, pts in variants.items():
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                legs, qe = dp._chain(scene, rb, pts, q0, bodies, "over", min_clear={0: dp.RETREAT_OUT_MIN_CLEAR})
            lines = [l for l in buf.getvalue().splitlines() if l.strip()]
            if legs:
                worst = min(float(l.split("worst clearance ")[1].split(" cm")[0]) for l in lines if "worst clearance" in l)
                print(f"PASS  {name}   end J4 {np.degrees(qe[3]):.0f} J6 {np.degrees(qe[5]):.0f}, worst clearance "
                      f"{worst:.1f} cm")
            else:
                bad = [l for l in lines if "->" in l][-1].strip()
                print(f"fail  {name}   at: {bad}")
    finally:
        dp.drop(rb, bodies)


if __name__ == "__main__":
    main()
