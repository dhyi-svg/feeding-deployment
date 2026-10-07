"""Touch check: detected grasp vs where you put the gripper by hand. No motion.

    python3 microwave/tools/touch_check.py detect    # from the view pose: detect, save the planned grasp
    # release the arm (sheppy camera-only), teleop the open gripper onto the handle as for a grasp,
    # bring the arm node back (sheppy real-arm-camera), then:
    python3 microwave/tools/touch_check.py compare   # tool pose now vs the saved planned grasp

`detect` uses the open script's own detect_handle() + grasp_pose(), i.e. exactly the target a real
run would command (GRIP_EXT / VERTICAL_CORR / GRASP_X_ADJUST included). `compare` splits the error
along the grasp's approach axis, sideways (horizontal, across the approach) and vertical; a positive
`approach` error means the hand-placed gripper is further INTO the door than the planned grasp.
"""
import json, sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # microwave/
import numpy as np
from scipy.spatial.transform import Rotation as R

SAVE = Path.home() / ".microwave_touch_check.json"


def detect():
    import real_gen3_ros2_grasp_and_swing_microwave as mw
    h, orient, n, _, _, hinge = mw.detect_handle()
    hc, grasp = mw.grasp_pose(h, orient, n)
    rec = {"handle": list(map(float, h)), "handle_vcorr": list(map(float, hc)), "normal": list(map(float, n)),
           "hinge": list(map(float, hinge)), "grasp_pos": list(map(float, grasp.position)),
           "grasp_quat": list(map(float, grasp.orientation)),
           "constants": {"GRIP_EXT": mw.GRIP_EXT, "VERTICAL_CORR": mw.VERTICAL_CORR,
                         "GRASP_X_ADJUST": mw.GRASP_X_ADJUST}}
    SAVE.write_text(json.dumps(rec, indent=1))
    print(f"saved planned grasp {np.round(grasp.position, 4)} to {SAVE}")


def compare():
    from sheppy_arm import SheppyArm
    rec = json.loads(SAVE.read_text())
    arm = SheppyArm()
    try:
        ee = np.asarray(arm.get_state()["ee_pos"])
    finally:
        arm.close()
    g_p, g_q = np.asarray(rec["grasp_pos"]), np.asarray(rec["grasp_quat"])
    approach = R.from_quat(g_q).as_matrix()[:, 2]
    side = np.cross([0.0, 0.0, 1.0], approach)
    side /= np.linalg.norm(side)
    d = ee[:3] - g_p
    ang = np.degrees(2 * np.arccos(min(1.0, abs(float(np.dot(ee[3:], g_q))))))
    print(f"planned grasp  {np.round(g_p, 4)}")
    print(f"touched (tool) {np.round(ee[:3], 4)}")
    print(f"error: total {np.linalg.norm(d) * 100:.1f} cm | approach {np.dot(d, approach) * 100:+.1f} cm "
          f"(+ = touched deeper into the door) | sideways {np.dot(d, side) * 100:+.1f} cm "
          f"(+ = robot's left) | vertical {d[2] * 100:+.1f} cm | orientation {ang:.1f} deg")
    print(f"constants used: {rec['constants']}")


if __name__ == "__main__":
    {"detect": detect, "compare": compare}.get(sys.argv[1] if len(sys.argv) > 1 else "", lambda: sys.exit(__doc__))()
