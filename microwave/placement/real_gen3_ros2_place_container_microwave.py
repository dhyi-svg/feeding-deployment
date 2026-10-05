"""Place the held container into the OPEN microwave on `rchi-cpu-5`: look inside, measure the
cavity from depth, insert the container level, lower it, release.

    python3 -u microwave/placement/real_gen3_ros2_place_container_microwave.py --container-drop 0.05             # dry run
    python3 -u microwave/placement/real_gen3_ros2_place_container_microwave.py --container-drop 0.05 --execute
    python3 -u microwave/placement/real_gen3_ros2_place_container_microwave.py --release --execute

Start: door open (the open task's swing wrote `door_open_deg` to `~/.microwave_door.json`), the
container held in the gripper level and sticking out along the approach axis (the close task's
container hold), and the wrist camera looking into the microwave.

1. Look -- perceived NOW, with the door open; nothing is cached from the closed door. SAM 3
   segments the interior from the colour image by text prompt, every (eroded) mask pixel with
   depth goes to arm_base_link, and `microwave_cavity.estimate_microwave_cavity` turns the points
   into front/back/floor/top/side bounds and a container placement point. An implausible cavity
   is refused -- there is no guessed fallback. Looks until two placement points agree within 3 cm.
2. Plan everything before any motion, holding the hand's CURRENT orientation fixed on every
   2 cm step (the container stays level and does not turn through the opening): current ->
   pre-insert (container clear of the opening) -> inserted, above the spot -> lowered. Gates:
   IK, joint jump, J4/J6, wrap, and clearance to the open door (and to the microwave body until
   the opening). The container itself is not in the collision model.
3. Execute the approach + insert, then the final lowering (`lower_container`). The hand stays
   on the container.

`--release` (separate run, after checking the container sits right): open the gripper, back
straight out along -approach to the pre-insert point, then a door-checked move to park.

Run from the repo root (the sim config path is relative). See ../NOTES.md for the bring-up and env.
The same steps as a ROS 2 node (live SAM 3 overlay + plan/execute services): `microwave_place_node.py`.
Library use: `plan_placement` / `execute_placement`, `plan_release` / `execute_release`; they raise
`PlacementRefused` (with the reason) instead of exiting.
"""
import argparse, json, os, sys, time
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))               # microwave/: door_push, microwave_common
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))   # detect_handle_sam3
import detect_handle_sam3  # noqa: E402
from feeding_deployment.control.robot_controller.arm_client import ArmInterfaceClient  # noqa: E402
from feeding_deployment.control.robot_controller.command_interface import JointCommand, OpenGripperCommand  # noqa: E402

from door_push import DoorFrame, plan_to_park  # noqa: E402
from microwave_cavity import estimate_microwave_cavity, microwave_frame  # noqa: E402
from microwave_common import (BIG_MOVE_TIMEOUT_S, DOOR_FILE, MIN_CLEAR, clearance, execute_joint_plan,  # noqa: E402
                              make_sim, plan_cartesian, plan_straight_line, wait_for_joints)

# --- look inside ---
PROMPT = "inside of open microwave"   # SAM 3 text prompt; to try if it misses: "microwave interior",
                                      # "open microwave cavity"
MAX_LOOKS = 5              # keep looking until two placement points agree within DETECT_AGREE
MAX_EMPTY_RETRIES = 8      # per look: retries on a failed look (no mask / bad depth / implausible cavity)
DETECT_AGREE = detect_handle_sam3.DETECT_AGREE
MASK_ERODE_PX = 5          # drop mask-edge pixels, where depth bleeds between the rim and the interior
# Depth correction along the camera's optical axis. 0 like handle_detect.py (09-29 microwave grasps
# landed 0.0 cm without one); the SAM 3 fridge path uses +0.051 (measured 09-20). Re-measure if the
# placement is off along the look direction.
DEPTH_CORR_M = float(os.environ.get("PLACE_DEPTH_CORR", "0.0"))

# --- held container ---
CONTAINER_LENGTH = 0.152   # container reaches this far past the tool frame along the approach (6 in, the
                           # close task's --hold-offset-x); its centre is taken at half of it
MAX_APPROACH_YAW_DEG = 30.0   # the hand's approach must point into the microwave within this
MAX_APPROACH_TILT_DEG = 10.0  # ... and be this close to horizontal (container held level)

# --- placement ---
RELEASE_ABOVE_FLOOR = 0.01    # container bottom ends this far above the estimated floor: position control
                              # only (see lower_container), so never drive into a floor estimated too low
INSERT_CLEARANCE = 0.03       # carried this much higher while inserted, to clear the front lip
PRE_INSERT_STANDOFF = 0.10    # pre-insert: the container's far end this far in front of the opening
TOP_CLEARANCE = 0.03          # tool frame stays this far below the visible cavity top while inserted
MAX_REACH = 0.915             # same reach gate as the grasp script (tuned on this rig); plan_cartesian has none
PLACE_FILE = Path.home() / ".microwave_place.json"   # what --release backs out to
UNMOVED_TOL_DEG = 2.0         # execute_*: the arm must still be where the plan started


class PlacementRefused(RuntimeError):
    """A check failed; nothing (more) is commanded. str(e) says why."""


def look_inside(det, rs, tf, frame):
    """One look: SAM 3 interior mask -> depth points in arm_base_link -> cavity.

    Returns (cavity, message, overlay). cavity is None when the look is unusable and `message`
    says why; otherwise `message` summarises it. `overlay` is the colour frame (BGR) with the mask
    (green) and placement point (red) drawn on it, or None without a camera frame. `frame` None =
    no microwave axes (no door file): segment and draw the mask only, never a cavity.
    """
    d = rs.get_camera_data()
    bgr, depth, cam = d["rgb_image"], d["depth_image"], d["camera_info"]
    if bgr is None or depth is None or cam is None:
        return None, "no camera frame yet", None
    vis = bgr.copy()

    def done(cavity, msg):
        cv2.putText(vis, msg[:90], (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 255, 0) if cavity is not None else (0, 0, 255), 2)
        return cavity, msg, vis

    seg = det.segment(np.ascontiguousarray(bgr[:, :, ::-1]))
    if seg is None:
        return done(None, f"NO INTERIOR: SAM 3 found nothing for {det.prompt!r}")
    mask, score, box = seg
    k = 2 * MASK_ERODE_PX + 1
    core = cv2.erode(mask.astype(np.uint8), np.ones((k, k), np.uint8)) > 0
    contours, _ = cv2.findContours(core.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(vis, contours, -1, (0, 255, 0), 2)
    points, n_mask = detect_handle_sam3.mask_to_camera_points(core, depth, cam)
    if len(points) < detect_handle_sam3.MIN_VALID_DEPTH_FRAC * max(n_mask, 1):
        return done(None, f"BAD DEPTH: score {score:.2f}, only {len(points)}/{n_mask} interior pixels have depth")
    if frame is None:
        return done(None, f"score {score:.2f}  mask {n_mask} px -- no door file, mask only")
    points = points + np.array([0.0, 0.0, DEPTH_CORR_M])

    transform = tf.get_frame_to_frame_transform(cam)
    if transform is None:
        return done(None, "No arm_base_link<-camera transform (is calibration_tf up?)")
    base_from_cam = tf.make_homogeneous_transform(transform)
    try:
        cavity = estimate_microwave_cavity(points @ base_from_cam[:3, :3].T + base_from_cam[:3, 3], frame)
    except ValueError as e:
        return done(None, f"REJECTED: score {score:.2f}, {e}")

    cam_from_base = np.linalg.inv(base_from_cam)
    pc = cam_from_base[:3, :3] @ cavity["placement_point"] + cam_from_base[:3, 3] - [0.0, 0.0, DEPTH_CORR_M]
    if pc[2] > 0:
        u = int(cam.K[0] * pc[0] / pc[2] + cam.K[2])
        v = int(cam.K[4] * pc[1] / pc[2] + cam.K[5])
        cv2.circle(vis, (u, v), 8, (0, 0, 255), -1)
    return done(cavity, f"score {score:.2f}  mask {n_mask} px ({len(points)} with depth)  "
                        f"depth {(cavity['back'] - cavity['front']) * 100:.0f} x width "
                        f"{(cavity['left'] - cavity['right']) * 100:.0f} cm")


def detect_cavity(frame, detector=None):
    """Looks until two placement points agree. Returns the pair's mean cavity.
    `detector`: an already-built (det, rs, tf) to reuse (the node); None builds one (loads SAM 3).
    Every look's overlay is saved to /tmp/microwave_place/<timestamp>/."""
    log_dir = Path(f"/tmp/microwave_place/{time.strftime('%Y%m%d_%H%M%S')}")
    log_dir.mkdir(parents=True, exist_ok=True)
    if detector is None:
        detector = detect_handle_sam3.build_detector(prompt=PROMPT)[:3]
    det, rs, tf = detector

    def look(tag):
        for attempt in range(1, MAX_EMPTY_RETRIES + 1):
            cavity, msg, vis = look_inside(det, rs, tf, frame)
            if vis is not None:
                cv2.imwrite(str(log_dir / f"interior_{tag}_{attempt}.png"), vis)
            if cavity is not None:
                print(f"  look {tag}: {msg}  placement {np.round(cavity['placement_point'], 4)}")
                return cavity
            print(f"  look {tag}: {msg} (retry {attempt}/{MAX_EMPTY_RETRIES})")
            time.sleep(0.5)
        raise PlacementRefused(f"No usable look inside the microwave ({tag}) after {MAX_EMPTY_RETRIES} "
                               f"retries -- refusing. Overlays in {log_dir}")

    looks, pair = [], None
    for i in range(1, MAX_LOOKS + 1):
        looks.append(look(str(i)))
        pair = next((looks[j] for j in range(len(looks) - 1)
                     if np.linalg.norm(looks[j]["placement_point"] - looks[-1]["placement_point"]) <= DETECT_AGREE),
                    None)
        if pair is not None:
            break
    if pair is None:
        raise PlacementRefused(f"No two of {len(looks)} looks agreed within {DETECT_AGREE * 100:.0f} cm -- refusing.")
    a, b = pair, looks[-1]
    print(f"looks agree to {np.linalg.norm(a['placement_point'] - b['placement_point']) * 100:.1f} cm "
          f"after {len(looks)} look(s); overlays in {log_dir}")
    return {k: (np.asarray(a[k]) + np.asarray(b[k])) / 2.0 for k in a}


def placement_positions(cavity, frame, quat, drop):
    """Tool positions (pre-insert, above, place) for the container centre on the placement point.
    The hand keeps `quat`, so the container moves along its own (horizontal) approach axis."""
    approach = R.from_quat(quat).as_matrix()[:, 2]
    tilt = float(np.degrees(np.arcsin(np.clip(approach[2], -1.0, 1.0))))
    a = np.array([approach[0], approach[1], 0.0])
    a /= np.linalg.norm(a)
    fwd, up = frame[:, 2], np.array([0.0, 0.0, 1.0])
    yaw = float(np.degrees(np.arccos(np.clip(a @ fwd, -1.0, 1.0))))
    print(f"hand approach {np.round(approach, 3)}: tilt {tilt:+.1f} deg, {yaw:.1f} deg off the microwave's axis")
    if abs(tilt) > MAX_APPROACH_TILT_DEG:
        raise PlacementRefused(f"approach tilted {tilt:.1f} deg (max {MAX_APPROACH_TILT_DEG}) -- container not level? Refusing.")
    if yaw > MAX_APPROACH_YAW_DEG:
        raise PlacementRefused(f"approach {yaw:.1f} deg off the microwave's axis (max {MAX_APPROACH_YAW_DEG}) -- refusing.")

    centre = np.asarray(cavity["placement_point"], float)          # on the floor
    place = centre - a * CONTAINER_LENGTH / 2 + up * (drop + RELEASE_ABOVE_FLOOR)
    above = place + up * INSERT_CLEARANCE
    far_end = float((above + a * CONTAINER_LENGTH) @ fwd)
    pre = above - a * (far_end - (cavity["front"] - PRE_INSERT_STANDOFF)) / float(a @ fwd)

    near, far = float((place @ fwd)), float((place + a * CONTAINER_LENGTH) @ fwd)
    print(f"cavity along the axis: front {cavity['front']:.3f}  back {cavity['back']:.3f}; "
          f"container {near:.3f} -> {far:.3f}")
    print(f"floor z {cavity['floor']:.3f}  visible top z {cavity['top']:.3f}  tool z inserted {above[2]:.3f}")
    if near < cavity["front"] or far > cavity["back"]:
        raise PlacementRefused("container would not fit between the front and the back wall -- refusing.")
    reach = max(float(np.linalg.norm(x)) for x in (pre, above, place))
    if reach > MAX_REACH:
        raise PlacementRefused(f"placement needs {reach:.3f} m of reach (max {MAX_REACH}) -- the base is too far from the "
                 "microwave. Refusing.")
    if above[2] > cavity["top"] - TOP_CLEARANCE:
        raise PlacementRefused(f"inserted tool z {above[2]:.3f} is within {TOP_CLEARANCE * 100:.0f} cm of the visible cavity top "
                 f"{cavity['top']:.3f} -- refusing.")
    return pre, above, place


def lower_container(ai, leg):
    """The final lowering onto the cavity floor.

    TODO(impedance): the team decided this lowering should be impedance-controlled, so a few cm of
    perception error is absorbed instead of driven through. The only mechanism in the repo is the
    task compliant mode (`ArmInterfaceClient.switch_to_task_compliant_mode` + `compliant_set_ee_pose`),
    and it is not usable here as-is: `kinova.py` hardcodes `fix_joint_hack = True` (J6 frozen at
    -1.18 rad, a 6-DOF model -- placement runs J6 ~ +30..90 deg in sim), its gains (`compliant_controller.py`
    K_T_p / K_T_d) were tuned for bite transfer, and it has never run on this rig. Needed from
    Saisha/team: 7-DOF task control (fix_joint_hack optional), stiffness/damping for this task, and
    the collision-sensor policy during the lowering. Until then: the planned position-controlled
    steps, ending RELEASE_ABOVE_FLOOR above the estimated floor.
    """
    return execute_joint_plan(ai, leg, "lower")


def _check_unmoved(ai, q_plan):
    """Refuse to run a plan whose start is no longer where the arm is."""
    cur = np.asarray(ai.get_state()["position"], float)
    off = float(np.degrees(np.max(np.abs((cur - np.asarray(q_plan) + np.pi) % (2 * np.pi) - np.pi))))
    if off > UNMOVED_TOL_DEG:
        raise PlacementRefused(f"the arm moved {off:.1f} deg (max joint) since the plan -- re-plan.")


def plan_placement(ai, container_drop, detector=None):
    """Look inside, compute pre-insert/above/place and plan every leg in sim. No motion.
    Returns the plan dict `execute_placement` takes. Raises PlacementRefused."""
    if container_drop is None or container_drop <= 0:
        raise PlacementRefused("container_drop is required: how far the container's bottom is below the tool "
                               "frame (m), measured on the held container.")
    door = json.loads(DOOR_FILE.read_text()) if DOOR_FILE.exists() else {}
    if not all(k in door for k in ("closed_normal", "hinge", "closed_grasp_pos", "door_open_deg", "open_sign")):
        raise PlacementRefused(f"{DOOR_FILE} needs the open task's door geometry and door_open_deg/open_sign "
                               "-- refusing.")
    st = ai.get_state()
    ee0, q0 = np.asarray(st["ee_pos"], dtype=float), np.asarray(st["position"], dtype=float)
    quat = ee0[3:7]
    if float(st["gripper_pos"]) < 0.2:
        raise PlacementRefused("Gripper is OPEN -- nothing held. Refusing.")

    frame = microwave_frame(door["closed_normal"])
    cavity = detect_cavity(frame, detector)
    pre, above, place = placement_positions(cavity, frame, quat, container_drop)
    print(f"pre-insert {np.round(pre, 4)}  above {np.round(above, 4)}  place {np.round(place, 4)}")

    # --- plan everything before any motion, the hand's orientation fixed throughout ---
    scene, rb = make_sim()
    bodies = DoorFrame(door, door["open_sign"]).bodies(scene, rb, door["door_open_deg"])
    door_only = [b for b in bodies if b[1] == "door"]   # the body model is solid: inside it is the cavity
    print(f"door model: open {door['door_open_deg']:.1f} deg; current clearance "
          f"{clearance(rb, q0, bodies)[0] * 100:.1f} cm")
    legs = []
    for label, p0, p1, bods in (("to pre-insert", ee0[:3], pre, bodies),
                                ("insert", pre, above, door_only),
                                ("lower", above, place, door_only)):
        leg = plan_cartesian(scene, rb, p0, quat, p1, quat, legs[-1][1][-1] if legs else q0, label, bods)
        if leg is None:
            raise PlacementRefused(f"{label} fails a gate -- refusing before any motion.")
        legs.append((label, leg))
    return {"time": time.time(), "q0": q0, "quat": quat, "pre": pre, "above": above, "place": place,
            "cavity": cavity, "legs": legs}


def execute_placement(ai, plan):
    """Approach + insert, record the placement, lower. The hand stays on the container.
    Returns (ok, message). Raises PlacementRefused before any motion if the arm moved."""
    _check_unmoved(ai, plan["q0"])
    if float(ai.get_state()["gripper_pos"]) < 0.2:
        raise PlacementRefused("Gripper is OPEN -- nothing held. Refusing.")
    for label, leg in plan["legs"][:-1]:
        if not execute_joint_plan(ai, leg, label):
            return False, f"{label} stopped early -- still holding the container."
    PLACE_FILE.write_text(json.dumps({
        "time": time.strftime("%Y-%m-%d %H:%M:%S"), "pre_insert_pos": plan["pre"].tolist(),
        "above_pos": plan["above"].tolist(), "place_pos": plan["place"].tolist(), "quat": list(plan["quat"]),
        "cavity": {k: np.asarray(v).tolist() for k, v in plan["cavity"].items()}}, indent=1))
    ok = lower_container(ai, plan["legs"][-1][1])
    final = ai.get_state()
    return ok, (f"PLACE {'DONE' if ok else 'STOPPED SHORT'}: EE {np.round(final['ee_pos'][:3], 4)} "
                f"({np.linalg.norm(np.asarray(final['ee_pos'][:3]) - plan['place']) * 100:.1f} cm from the place "
                "point). Still holding -- check it, then release.")


def plan_release(ai):
    """Plan: back straight out along -approach to the recorded pre-insert point, then park.
    Not `release_and_back_off`: its park leg rebuilds the door model from the hand's position as if
    the hand were on the handle; here it's inside the microwave. Raises PlacementRefused."""
    if not PLACE_FILE.exists() or not DOOR_FILE.exists():
        raise PlacementRefused(f"Need {PLACE_FILE} (from a placement) and {DOOR_FILE} -- refusing.")
    rec, door = json.loads(PLACE_FILE.read_text()), json.loads(DOOR_FILE.read_text())
    st = ai.get_state()
    cur, quat = np.asarray(st["ee_pos"][:3], float), np.asarray(st["ee_pos"][3:7], float)
    q0 = np.asarray(st["position"], float)
    if float(st["gripper_pos"]) < 0.2:
        raise PlacementRefused("Gripper is already open -- refusing.")
    approach = R.from_quat(quat).as_matrix()[:, 2]
    back_m = float((cur - np.asarray(rec["pre_insert_pos"])) @ approach)
    print(f"placement ({rec['time']}): backing out {back_m * 100:.1f} cm to the pre-insert point")
    if not 0.0 < back_m < 0.6:
        raise PlacementRefused("the hand isn't where the recorded placement left it -- refusing.")

    scene, rb = make_sim()
    bodies = DoorFrame(door, door["open_sign"]).bodies(scene, rb, door["door_open_deg"])
    back = plan_straight_line(scene, rb, cur, quat, -approach, back_m, q0, "back out")
    if back is None:
        raise PlacementRefused("back-out fails a gate -- NOT releasing, arm untouched.")
    worst = min((clearance(rb, q, [b for b in bodies if b[1] == "door"]) for q in back), key=lambda w: w[0])
    print(f"  back-out worst clearance to the door {worst[0] * 100:.1f} cm ({worst[1]})")
    if worst[0] < MIN_CLEAR:
        raise PlacementRefused("back-out too close to the door -- NOT releasing, arm untouched.")
    park_leg, q_park = plan_to_park(scene, rb, cur - approach * back_m, quat, back[-1], bodies)
    if park_leg is None:
        # same open problem as after the open task's swing (NOTES.md): the straight leg to park can
        # hit J4's guard. Backing out is still safe -- stop there, return the arm by hand.
        print("  path to park fails a gate -- will stop after backing out (return the arm by hand).")
    return {"time": time.time(), "q0": q0, "back": back, "park_leg": park_leg, "q_park": q_park}


def execute_release(ai, plan):
    """Open the gripper, back out, park (if planned). Returns (ok, message)."""
    _check_unmoved(ai, plan["q0"])
    print("Releasing ...")
    ai.execute_command(OpenGripperCommand())
    time.sleep(1.0)
    ok = execute_joint_plan(ai, plan["back"], "back out")
    if ok and plan["park_leg"] is not None:
        ok = execute_joint_plan(ai, plan["park_leg"], "to park")
    if ok and plan["park_leg"] is not None and plan["q_park"] is not None:
        ai.execute_command(JointCommand(pos=plan["q_park"].tolist()))
        ok = wait_for_joints(ai, plan["q_park"], timeout_s=BIG_MOVE_TIMEOUT_S)
    final = ai.get_state()
    return ok, (f"RELEASE {'DONE' if ok else 'STOPPED EARLY'}"
                f"{'' if plan['park_leg'] is not None else ' (no park leg -- return the arm by hand)'}. "
                f"final EE {np.round(final['ee_pos'][:3], 4)}  gripper {float(final['gripper_pos']):.3f}")


def main():
    a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument("--execute", action="store_true", help="command the arm; omit for a dry run")
    a.add_argument("--release", action="store_true",
                   help="after placing: open the gripper, back out of the microwave, park")
    a.add_argument("--container-drop", type=float, default=None,
                   help="placing: how far the held container's bottom is below the tool frame (m) -- measure it")
    args = a.parse_args()
    ai = ArmInterfaceClient()
    try:
        if args.release:
            plan = plan_release(ai)
            if not args.execute:
                print(f"\nDRY RUN (release + back out{' + park' if plan['park_leg'] is not None else ''}) "
                      "-- nothing commanded.")
                return
            ok, msg = execute_release(ai, plan)
        else:
            plan = plan_placement(ai, args.container_drop)
            if not args.execute:
                print("\nDRY RUN -- would approach, insert and lower the container (hand stays on it). "
                      "Nothing commanded.")
                return
            ok, msg = execute_placement(ai, plan)
    except PlacementRefused as e:
        sys.exit(str(e))
    print(f"\n{msg}")
    if not ok:
        sys.exit(1)


if __name__ == "__main__":
    main()
