"""Grab the tupperware by the protruding sleeve ring, on THIS rig (RammpArm + cuRobo + tf2).

    python3 grasp_tupperware.py            # detect + plan + gates only (DRY, nothing moves)
    python3 grasp_tupperware.py --execute  # actually run it (attended, e-stop in hand)

Run with arm + camera + planner + calib_tf up under sheppy, and in this terminal:
    source ~/feeding-deployment/rammp_button_press/sheppy/env.sh

GRASP TARGET: the grabber is a protruding RING. The grasp point is the near rim (nearest-depth
slice of the ring mask), NOT the ring center (that's the empty hole over the box).
GRIPPER: published to /setpoint/gripper (GripperSetpoint: position 0=open..1=closed, + arm token).
"""
import sys, json, time
from pathlib import Path
import numpy as np, rclpy
from scipy.spatial.transform import Rotation
from rclpy.qos import qos_profile_sensor_data
from ultralytics import YOLO

from rammp_button_press import Abort
from rammp_button_press.rammp_arm import RammpArm
from rammp_button_press.perception import Perception, to_bgr_depth
from rammp_arm_interfaces.msg import GripperSetpoint
from cv_bridge import CvBridge

# ============================================================
#  FILL-IN
# ============================================================
MODEL_PATH = "/home/jetson/oxo/best.pt"
GRAB_CLASS = "grabber"
ARM_FRAME  = "base_link"
CAM_FRAME  = "camera_color_optical_frame"   # confirm: ros2 run tf2_ros tf2_echo base_link <cam>
CAMERA_NS  = "/wrist_camera"

# ============================================================
#  CALIBRATE / tune
# ============================================================
# ---- SIDE PINCH of the flat black brim: reach in horizontally, one finger above
#      the brim, one below. Tune these three while watching a dry/slow run. ----
APPROACH_YAW_DEG  = 0.0    # which horizontal direction (base frame, about +z) the gripper
                           #   reaches ALONG. 0 = +x. Rotate if it comes from a blocked side.
APPROACH_TILT_DEG = 75.0   # 0 = straight top-down pinch, 90 = pure horizontal side approach.
                           #   she wants ~horizontal; back off toward 75-80 for a slight down-tilt.
FINGER_ROLL_DEG   = 90.0    # roll the fingers about the approach axis. If they come out stacked
                           #   the WRONG way (side-by-side instead of top/bottom), set this to 90.
# Fine-nudge the grasp target off the detected point, BASE frame [dx, dy, dz] meters (z up).
GRASP_OFFSET_BASE = np.array([0.0, 0.0, 0.0])
GRASP_DEPTH_M = -0.03

STANDOFF_M = 0.10; LIFT_M = 0.07; BACKOUT_M = 0.15
RAISE_M = 0.06

CHECK_LIFT_M = 0.05; CHECK_TOL_M = 0.02
N_FRAMES = 5; AGREE_M = 0.01; NEAR_FRAC = 0.20
CONF = 0.5; DEPTH_MIN = 0.2; DEPTH_MAX = 2.0; MIN_PTS = 30

# gripper
GRIP_OPEN = 0.0; GRIP_CLOSE = 1.0     # CLOSE stalls on the ring at the force ceiling
GRIP_SPEED = 1.0; GRIP_FORCE = 0.5    # force = motor-current ceiling, not a force setpoint
GRIP_SETTLE_S = 2.0; GRIP_TOL = 0.05


# ============================================================
#  GRIPPER  (publish GripperSetpoint on /setpoint/gripper, riding the arm's token)
# ============================================================
def _grip_pub(arm):
    if not hasattr(arm, "_grasp_grip_pub"):
        arm._grasp_grip_pub = arm.node.create_publisher(
            GripperSetpoint, "/setpoint/gripper", qos_profile_sensor_data)
    return arm._grasp_grip_pub

def set_gripper(arm, position, label):
    print(f"  [gripper] {label} -> position {position}")
    if not arm.execute:
        return
    pub = _grip_pub(arm)
    m = GripperSetpoint()
    m.position, m.speed, m.force = float(position), float(GRIP_SPEED), float(GRIP_FORCE)
    m.token = list(arm._token)                      # same arbitration token take_control() got
    t0 = time.time()
    while time.time() - t0 < GRIP_SETTLE_S:         # not sticky + best-effort: keep publishing
        pub.publish(m)
        time.sleep(0.05)
        try:                                        # open reaches target; close stalls on the ring
            if abs(arm.state()["gripper_pos"] - position) < GRIP_TOL:
                break
        except Abort:
            pass

def open_gripper(arm):  set_gripper(arm, GRIP_OPEN, "OPEN")
def close_gripper(arm): set_gripper(arm, GRIP_CLOSE, "CLOSE")


def near_rim_point(pts):
    """Point ON the near rim: keep the nearest-depth slice of the ring mask and median it."""
    pts = np.asarray(pts)
    keep = pts[:, 2] <= np.quantile(pts[:, 2], NEAR_FRAC)
    return np.median(pts[keep], axis=0)


def detect_point_cam(perc, model, bridge):
    """One agreed near-rim grab point in the CAMERA frame (N frames must agree)."""
    frames = []
    for _ in range(N_FRAMES * 3):
        if len(frames) >= N_FRAMES: break
        color = perc.wait("color", 2.0); depth = perc.wait("depth", 2.0)
        if color is None or depth is None: continue
        bgr, dm = to_bgr_depth(bridge, color, depth)
        fx, fy, cx, cy = perc.intrinsics()
        r = model.predict(bgr, conf=CONF, retina_masks=True, verbose=False)[0]
        if r.masks is None or len(r.boxes) == 0: continue
        idx = [i for i, c in enumerate(r.boxes.cls.cpu().numpy()) if r.names[int(c)] == GRAB_CLASS]
        if not idx: continue
        best = max(idx, key=lambda i: float(r.boxes.conf[i]))
        m = r.masks.data[best].cpu().numpy()
        H, W = dm.shape[:2]
        if m.shape != (H, W):
            import cv2; m = cv2.resize(m, (W, H))
        ys, xs = np.where(m > 0.5)
        pts = []
        for u, v in zip(xs, ys):
            z = float(dm[v, u])
            if DEPTH_MIN < z < DEPTH_MAX:
                pts.append([(u - cx) * z / fx, (v - cy) * z / fy, z])
        if len(pts) >= MIN_PTS:
            frames.append(near_rim_point(pts))
    if len(frames) < N_FRAMES:
        raise Abort(f"ring not seen stably ({len(frames)}/{N_FRAMES} frames)")
    arr = np.array(frames)
    if np.max(np.linalg.norm(arr - np.median(arr, axis=0), axis=1)) > AGREE_M:
        raise Abort("frames disagree > {:.0f} cm -- not grasping".format(AGREE_M * 100))
    return np.median(arr, axis=0)


def cam_to_base(perc, p_cam):
    R_cb = perc.cam_rotation_in_base()
    t_cb = perc.cam_position_in_base()
    return R_cb @ np.asarray(p_cam) + t_cb


def grasp_orientation():
    """Build the grasp rotation for a SIDE pinch of the flat brim.
    Tool axes: z = approach (into object), y = finger separation, x = the third.
    yaw/tilt aim the approach; roll spins the fingers about it."""
    yaw, tilt, roll = np.radians([APPROACH_YAW_DEG, APPROACH_TILT_DEG, FINGER_ROLL_DEG])
    a = np.array([np.sin(tilt) * np.cos(yaw),
                  np.sin(tilt) * np.sin(yaw),
                  -np.cos(tilt)])                 # approach: tilt=0 -> straight down, 90 -> horizontal
    a /= np.linalg.norm(a)
    up = np.array([0.0, 0.0, 1.0])
    s0 = up - np.dot(up, a) * a                   # "as vertical as possible" finger-separation axis
    if np.linalg.norm(s0) < 1e-6:                 # approach was vertical; pick any perpendicular
        s0 = np.array([1.0, 0.0, 0.0])
    s0 /= np.linalg.norm(s0)
    s = np.cos(roll) * s0 + np.sin(roll) * np.cross(a, s0)   # roll fingers about the approach axis
    s /= np.linalg.norm(s)
    t = np.cross(s, a)
    return np.column_stack([t, s, a])             # columns = world dirs of tool x, y(sep), z(approach)



def main():
    execute = "--execute" in sys.argv
    out = open(Path.home() / f"grasp_{time.strftime('%Y%m%d_%H%M%S')}.jsonl", "w")
    log = lambda r: (out.write(json.dumps(r) + "\n"), out.flush())

    rclpy.init()
    model = YOLO(MODEL_PATH)
    bridge = CvBridge()
    perc = Perception(ARM_FRAME, CAM_FRAME, camera_ns=CAMERA_NS)
    arm = RammpArm(execute=execute, acquire_control=execute)
    try:
        problems, info = arm.status()
        for i in info: print(" ", i)
        if problems: raise Abort("; ".join(problems))

        p_cam = detect_point_cam(perc, model, bridge)
        p_base = cam_to_base(perc, p_cam) + GRASP_OFFSET_BASE
        R = grasp_orientation()
        approach = R @ np.array([0, 0, 1.0])

        p_base = p_base + GRASP_DEPTH_M * approach
        pre = p_base - STANDOFF_M * approach
        print(f"  grab point base xyz {np.round(p_base, 3)}  approach {np.round(approach, 2)}")

        arm.take_control()
        open_gripper(arm)
        arm.curobo_move(pre,    R, "pre-grasp", log)
        arm.curobo_move(p_base, R, "grasp",     log)
        close_gripper(arm)

        # One gentle raise, then HOLD. No re-detect check and no auto-reopen:
        # the wrist camera moves with the arm, so a re-detect after lifting isn't
        # reliable -- we just lift a little and keep the grip, confirm by eye.
        try:
            arm.curobo_move(p_base + np.array([0, 0, RAISE_M]), R, "raise", log)
            print("  raised + holding")
        except Abort as e:
            print(f"  raise skipped ({e}) -- holding in place")
        print("  DRY: planned + gated only" if not execute else "  done -- holding")


    except Abort as e:
        print(f"ABORT: {e}"); sys.exit(2)
    finally:
        arm.close()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == "__main__":
    main()
