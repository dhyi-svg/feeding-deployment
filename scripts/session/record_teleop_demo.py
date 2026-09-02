"""READ-ONLY recorder for a human-teleoped demo: RGB-D + arm state + ground-truth poses.

Records a task being done by hand (Xbox controller plugged into the arm) so autonomous
perception can later be replayed against the exact same frames, with the human's own EE
poses as ground truth. It commands nothing.

**Why this does not fight teleop.** The camera half is pure ROS 2 subscription -- it
cannot touch the arm. The arm half deliberately does *not* go through ``arm_server.py``:
Xbox teleop faults that Kortex session (see JETSON_SETUP.md "After an e-stop or Xbox
teleop"), so anything reading through the arm RPC dies as soon as you pick up the
controller. Instead it opens its own read-only Kortex session
(:class:`~feeding_deployment.control.robot_controller.kortex_readonly.KortexReadOnlyFeedback`)
that only ever calls ``RefreshFeedback()`` -- no fault clearing, no servoing-mode change,
no ``/tmp/kinova.lock``. Prove that on your hardware with ``--check`` *while teleoping*
before recording anything you care about.

Bring-up (arm_server / stub base / bulldog are NOT needed and should be stopped)::

    ros2 launch launch/ros2/microwave_bringup.launch.py use_joint_state_bridge:=false
    $PY -u scripts/session/record_teleop_demo.py --check --tag pos_A     # while teleoping
    $PY -u scripts/session/record_teleop_demo.py --tag pos_A --note "door open, 2nd try"

``use_joint_state_bridge:=false`` because the launch's own bridge reads the arm RPC,
which is down; this script publishes ``/joint_states`` itself from its Kortex session so
``robot_state_publisher`` still builds the TF tree and every saved frame carries a real
``arm_base_link <- camera_color_optical_frame`` matrix.

While recording, the terminal is a tagging console:

    <Enter>       mark this instant  (label "mark")
    grasp<Enter>  mark it "grasp" -- any word works
    q<Enter>      stop     (Ctrl-C also stops cleanly)

Gripper open/close transitions and arm-state changes are tagged automatically, so the
grasp pose is captured even if you never touch the keyboard. Every tag also forces the
next frame to be saved, whether or not the arm is still.

Frames are saved while the arm is being driven and for ``--settle-sec`` after it stops --
the motion *and* the settled arrival -- then stop while it sits parked, because a hundred
copies of one stationary view is bytes, not data. Move the arm and it resumes on its own.
``--all-frames`` records regardless.

Output, one directory per demo. Measured on this rig's 640x480 frames: ~137 kB per saved
frame (86 kB rgb JPEG q95 + 51 kB depth PNG at compression 6), so 2 Hz while driving costs
~16 MB/min; ``--rgb-format png`` is ~48 MB/min. Arm state is ~0.7 MB/min at 30 Hz, and
``--no-camera`` records that alone::

    <out>/meta.json          run metadata, camera_info, calibration, git SHA, counts
    <out>/state.jsonl        every arm sample: joints, velocity, effort, EE pose, gripper,
                             gripper motor current, external force/torque at the tool
    <out>/events.jsonl       tags, with the arm state at that instant
    <out>/ground_truth.json  first grasp pose, in record_ground_truth.py's schema
    <out>/frames/000000/frame_rgb.jpg
                          /frame_depth.png            uint16 millimetres, aligned to colour
                          /frame_detection_inputs.json  camera_info + base<-camera matrix
                          /frame_state.json             arm state + image/pose sync age

Each frame directory is a self-contained capture in the format the replay tools already
read, so they work on teleop recordings unchanged::

    $PY -u scripts/session/replay_detection.py <out>/frames/*
"""

import argparse
import json
import os
import shutil
import signal
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np

# TFInterface retries a failed stamped lookup for this long before falling back to the
# newest transform. That fallback is right for a static detection and WRONG here -- the
# arm is being driven, so "newest" is a different pose. Kept short, and the lookups below
# are stamped and never use the fallback path anyway.
os.environ.setdefault("TF_LOOKUP_TIMEOUT_SEC", "1.0")

from feeding_deployment.control.robot_controller.kortex_readonly import (  # noqa: E402
    KortexReadOnlyFeedback,
)

DEFAULT_CALIB_PATH = Path.home() / ".ros2" / "easy_handeye2" / "calibrations" / "wrist_camera_calib.calib"
BASE_FRAME = "arm_base_link"
CAMERA_FRAME = "camera_color_optical_frame"
TF_KEY = f"{BASE_FRAME}__from__{CAMERA_FRAME}"

# Gripper reads 0 open, saturating near 1 closed. 0.2 is the threshold the repo's own
# scripts print "open"/"CLOSED" against; a transition across it is a grasp or a release.
GRIPPER_CLOSED_THRESHOLD = 0.2
# Below this the arm counts as parked rather than being driven. ~1.1 deg/s on the
# fastest joint.
STILL_JOINT_VEL_RAD_S = 0.02
# How long to keep saving after the arm stops. Covers the settle, and gets the sharp,
# unblurred frame of wherever you just drove to.
DEFAULT_SETTLE_SEC = 1.5
# Measured on this box's 640x480 depth frames: level 6 is 51 kB in 37 ms, against 70 kB
# in 14 ms at level 1 and 47 kB in 426 ms at level 9. 6 is where the curve turns.
DEFAULT_PNG_COMPRESSION = 6


def git_commit() -> str | None:
    """The repo state this recording was made against, or None outside a checkout."""
    import subprocess  # noqa: PLC0415 -- only needed once, at the end of a run

    try:
        return subprocess.run(
            ["git", "-C", str(Path(__file__).resolve().parent), "rev-parse", "--short", "HEAD"],
            capture_output=True, text=True, check=True, timeout=5,
        ).stdout.strip()
    except Exception:  # noqa: BLE001 -- metadata, never fatal
        return None


def jsonable(value):
    """Plain-Python, rounded floats -- state.jsonl is the file that gets big."""
    if isinstance(value, np.generic):
        value = value.item()
    if isinstance(value, float):
        return round(value, 6)
    if isinstance(value, np.ndarray):
        return [jsonable(v) for v in value.tolist()]
    if isinstance(value, dict):
        return {k: jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [jsonable(v) for v in value]
    return value


def state_record(state, now, still_vel=STILL_JOINT_VEL_RAD_S):
    """One arm sample as a flat dict."""
    ee = np.asarray(state["ee_pos"], dtype=float)
    velocity = np.asarray(state["velocity"], dtype=float)
    return jsonable(
        {
            "t": now,
            "joints": state["position"],
            "joint_vel": velocity,
            "joint_effort": state["effort"],
            "ee_pos": ee[:3],
            "ee_quat": ee[3:7],
            "gripper": state["gripper_pos"],
            # Elevated while the fingers stall against something -- unlike gripper_pos,
            # this distinguishes "closed on the handle" from "closed on air".
            "gripper_current": state.get("gripper_current"),
            "ee_force": state.get("ee_force"),
            "ee_torque": state.get("ee_torque"),
            "max_joint_vel": float(np.max(np.abs(velocity))) if velocity.size else None,
            "still": bool(velocity.size and np.max(np.abs(velocity)) < still_vel),
        }
    )


class JsonlWriter:
    """Append-only JSONL, flushed every line. Thread-safe."""

    def __init__(self, path: Path):
        self._lock = threading.Lock()
        self._file = path.open("a", encoding="utf-8")
        self.count = 0

    def write(self, record: dict) -> None:
        line = json.dumps(record)
        with self._lock:
            self._file.write(line + "\n")
            self._file.flush()
            self.count += 1

    def close(self) -> None:
        with self._lock:
            self._file.close()


class LatestState:
    """The most recent arm sample, shared between the poller and the joint-state bridge.

    The bridge polls at its own rate; handing it this instead of the Kortex client means
    one ``RefreshFeedback()`` per sample rather than two competing pollers on one session.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._state = None
        self._stamp = 0.0

    def set(self, state, stamp) -> None:
        with self._lock:
            self._state, self._stamp = state, stamp

    def get_state(self):  # the duck-typed contract JointStateBridge expects
        with self._lock:
            if self._state is None:
                raise RuntimeError("no arm sample yet")
            return self._state

    def get(self):
        with self._lock:
            return self._state, self._stamp


def open_camera(timeout_sec: float):
    from feeding_deployment.ros2.realsense_ros2_interface import RealSenseROS2Interface

    camera = RealSenseROS2Interface()
    if not camera.wait_for_frames(timeout_sec):
        raise SystemExit(
            "No synchronised RGB-D frames. Start the camera:\n"
            "  ros2 launch launch/ros2/microwave_bringup.launch.py use_joint_state_bridge:=false"
        )
    return camera


def lookup_base_from_camera(tf_interface, camera_info, timeout_sec: float):
    """The base<-camera transform AT THE FRAME'S STAMP, or None.

    Stamped on purpose: during teleop the arm is moving, so the newest transform belongs
    to a different pose than the frame. A frame whose transform cannot be resolved is
    unusable as ground truth and is dropped rather than mislabelled.
    """
    from rclpy.duration import Duration
    from rclpy.time import Time

    from feeding_deployment.ros2.compat import stamp_to_sec_nanosec

    sec, nanosec = stamp_to_sec_nanosec(camera_info.header.stamp)
    try:
        return tf_interface.tfBuffer.lookup_transform(
            BASE_FRAME,
            CAMERA_FRAME,
            Time(seconds=sec, nanoseconds=nanosec),
            timeout=Duration(seconds=timeout_sec),
        )
    except Exception:  # noqa: BLE001 -- tf2 raises several unrelated exception types
        return None


def start_joint_state_publishing(arm, latest: LatestState, rate_hz: float, stop: threading.Event):
    """Publish /joint_states from the read-only Kortex session, on a background thread.

    Nothing else can: the launch's own bridge reads ``arm_server.py``, whose Kortex
    session teleop has faulted. Without this, robot_state_publisher has no joint angles
    and there is no ``arm_base_link -> camera_color_optical_frame`` to stamp frames with.
    """
    from feeding_deployment.ros2.joint_state_bridge import JointStateBridge

    def poll():
        while not stop.is_set():
            try:
                latest.set(arm.get_state(), time.time())
            except Exception:  # noqa: BLE001 -- a blip must not kill the thread
                pass
            time.sleep(1.0 / rate_hz)

    threading.Thread(target=poll, name="arm-poll-check", daemon=True).start()
    return JointStateBridge(latest, rate_hz=min(rate_hz, 50.0))


def free_gb(path: Path) -> float:
    return shutil.disk_usage(path).free / 1e9


def dir_size_mb(path: Path) -> float:
    return sum(f.stat().st_size for f in path.rglob("*") if f.is_file()) / 1e6


def run_check(args, arm) -> int:
    """Prove the recorder can read everything it needs WITHOUT disturbing teleop."""
    print(f"arm state    : {arm.get_arm_state()}")
    state = arm.get_state()
    ee = np.asarray(state["ee_pos"], dtype=float)
    print(f"joints (deg) : {[round(float(np.degrees(v)), 1) for v in state['position']]}")
    print(f"EE           : {np.round(ee[:3], 4)}")
    print(f"gripper      : {state['gripper_pos']:.4f} "
          f"({'CLOSED' if state['gripper_pos'] > GRIPPER_CLOSED_THRESHOLD else 'open'})")

    print("\nsampling for 3 s -- drive the arm with the controller now; "
          "the numbers must move and nothing must error")
    samples, t_end = 0, time.time() + 3.0
    first, last = np.asarray(state["ee_pos"][:3], dtype=float), None
    while time.time() < t_end:
        last = np.asarray(arm.get_state()["ee_pos"][:3], dtype=float)
        samples += 1
        time.sleep(1.0 / args.state_hz)
    print(f"  {samples} samples, EE moved {np.linalg.norm(last - first) * 100:.1f} cm, "
          f"arm state still {arm.get_arm_state()}")

    if args.no_camera:
        return 0

    camera = open_camera(args.camera_timeout)
    from feeding_deployment.perception.tf_interface import TFInterface

    data = camera.get_camera_data()
    info = data["camera_info"]
    print(f"\ncamera       : rgb {data['rgb_image'].shape} depth {data['depth_image'].shape} "
          f"({info.width}x{info.height})")

    # The TF check is only meaningful with the same joint-state publishing a real
    # recording does -- robot_state_publisher has no other source while teleop holds
    # the arm and arm_server.py is down.
    stop = threading.Event()
    latest = LatestState()
    if not args.no_joint_states:
        start_joint_state_publishing(arm, latest, args.state_hz, stop)
        time.sleep(2.0)

    transform = lookup_base_from_camera(TFInterface(), camera.get_camera_data()["camera_info"],
                                        args.tf_timeout)
    stop.set()
    if transform is None:
        print(f"TF           : NO {BASE_FRAME} -> {CAMERA_FRAME} at the frame stamp.")
        print("               Check the launch is up (robot_state_publisher + the static")
        print("               hand-eye calibration) and that nothing else owns /joint_states.")
        return 1
    t = transform.transform.translation
    print(f"TF           : camera at {np.round([t.x, t.y, t.z], 3)} in {BASE_FRAME}")
    print("\nREADY -- teleop was not interrupted by any of the above.")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--out", type=Path, default=None,
                        help="run directory (default ~/captures/teleop_<timestamp>)")
    parser.add_argument("--tag", default="", help="appliance-position tag, e.g. pos_A")
    parser.add_argument("--note", default="", help="free text stored in meta.json")
    parser.add_argument("--frame-hz", type=float, default=2.0,
                        help="RGB-D frames saved per second (default 2 = ~60 MB/min; "
                             "the dominant cost by far)")
    parser.add_argument("--state-hz", type=float, default=30.0,
                        help="arm samples per second (default 30 = ~0.7 MB/min)")
    parser.add_argument("--rgb-format", choices=["png", "jpg"], default="jpg",
                        help="jpg q95 is ~5x smaller (86 kB vs 411 kB here) and lossy only "
                             "in the colour image -- depth is always lossless 16-bit PNG. "
                             "png for a pixel-exact colour record")
    parser.add_argument("--jpg-quality", type=int, default=95)
    parser.add_argument("--png-compression", type=int, default=DEFAULT_PNG_COMPRESSION,
                        choices=range(0, 10), metavar="0-9",
                        help=f"zlib level for the depth PNG, and for colour under "
                             f"--rgb-format png (default {DEFAULT_PNG_COMPRESSION}: depth "
                             f"51 kB / 37 ms per frame here, vs 70 kB / 14 ms at level 1. "
                             f"Level 9 saves 4 kB more and costs 426 ms -- not worth it)")
    parser.add_argument("--all-frames", action="store_true",
                        help="save frames even while the arm sits parked. Default is "
                             "motion-only: save while it is being driven and for "
                             "--settle-sec after it stops, then stop duplicating a "
                             "stationary view. Tagged instants are always saved")
    parser.add_argument("--settle-sec", type=float, default=DEFAULT_SETTLE_SEC,
                        help=f"keep saving this long after motion stops (default "
                             f"{DEFAULT_SETTLE_SEC}) -- the settled, unblurred frame of "
                             f"where you just drove to")
    parser.add_argument("--still-vel", type=float, default=STILL_JOINT_VEL_RAD_S,
                        help=f"joint speed (rad/s) below which the arm counts as parked "
                             f"(default {STILL_JOINT_VEL_RAD_S}, ~1.1 deg/s)")
    parser.add_argument("--no-camera", action="store_true",
                        help="arm state only -- no ROS, no images, tiny files")
    parser.add_argument("--no-joint-states", action="store_true",
                        help="do not publish /joint_states (use if something else already does)")
    parser.add_argument("--camera-timeout", type=float, default=30.0)
    parser.add_argument("--tf-timeout", type=float, default=0.3,
                        help="how long to wait for the transform at a frame's stamp")
    parser.add_argument("--min-free-gb", type=float, default=2.0,
                        help="stop recording when the disk falls below this")
    parser.add_argument("--max-minutes", type=float, default=30.0)
    parser.add_argument("--arm-ip", default="192.168.1.10")
    parser.add_argument("--check", action="store_true",
                        help="read everything once and exit -- run this WHILE teleoping")
    args = parser.parse_args()

    arm = KortexReadOnlyFeedback(ip=args.arm_ip)
    try:
        if args.check:
            return run_check(args, arm)
        return record(args, arm)
    finally:
        arm.close()


def record(args, arm) -> int:  # pylint: disable=too-many-branches,too-many-statements
    out = args.out or Path.home() / "captures" / time.strftime("teleop_%Y%m%d_%H%M%S")
    out = out.expanduser()
    frames_dir = out / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)

    latest = LatestState()
    states = JsonlWriter(out / "state.jsonl")
    events = JsonlWriter(out / "events.jsonl")
    stop = threading.Event()
    # Set by any tag, cleared once the frame thread has honoured it: a marked instant
    # (grasp, a demonstrated waypoint) is exactly the frame you cannot afford to have
    # skipped for being 1 deg/s too fast.
    force_frame = threading.Event()
    counters = {"frames": 0, "no_tf": 0, "idle": 0, "forced": 0, "write_ms": 0.0}
    # Last instant the arm was being driven. Maintained by the arm thread at --state-hz,
    # not by the frame thread: a quick nudge between two frames still counts as motion.
    motion = {"last_t": time.time()}

    def tag(label: str, **fields) -> dict:
        state, stamp = latest.get()
        record_ = {"label": label, "t": stamp, **fields}
        if state is not None:
            ee = np.asarray(state["ee_pos"], dtype=float)
            record_.update(jsonable({
                "ee_pos": ee[:3], "ee_quat": ee[3:7],
                "joints": state["position"], "gripper": state["gripper_pos"],
                "gripper_current": state.get("gripper_current"),
                "ee_force": state.get("ee_force"),
            }))
        events.write(record_)
        force_frame.set()
        print(f"  [{label}] {np.round(record_.get('ee_pos', []), 4)}")
        return record_

    # --- arm sampling: the only thread that talks to the arm -----------------------
    def poll_arm():
        was_closed = None
        last_arm_state = None
        next_arm_state_check = 0.0
        while not stop.is_set():
            loop_start = time.time()
            try:
                state = arm.get_state()
            except Exception as e:  # noqa: BLE001
                events.write({"label": "arm_read_failed", "t": time.time(), "error": str(e)})
                print(f"\n  ARM READ FAILED: {e}")
                stop.set()
                return
            now = time.time()
            latest.set(state, now)
            sample = state_record(state, now, args.still_vel)
            states.write(sample)
            if not sample["still"]:
                motion["last_t"] = now

            closed = state["gripper_pos"] > GRIPPER_CLOSED_THRESHOLD
            if was_closed is not None and closed != was_closed:
                tag("grasp" if closed else "release", auto=True)
            was_closed = closed

            # Cheap, and it is how a teleop takeover or a latched fault shows up.
            if now >= next_arm_state_check:
                next_arm_state_check = now + 2.0
                arm_state = arm.get_arm_state()
                if arm_state != last_arm_state:
                    if last_arm_state is not None:
                        tag("arm_state_change", auto=True, arm_state=arm_state)
                    last_arm_state = arm_state
                    counters["arm_state"] = arm_state

            time.sleep(max(0.0, 1.0 / args.state_hz - (time.time() - loop_start)))

    threading.Thread(target=poll_arm, name="arm-poll", daemon=True).start()
    deadline = time.time() + 2.0
    while latest.get()[0] is None and time.time() < deadline:
        time.sleep(0.05)
    if latest.get()[0] is None:
        print("No arm sample in 2 s -- the read-only Kortex session is not returning "
              "feedback. Run with --check for the diagnostics.")
        return 1

    camera = tf_interface = None
    camera_info_dict = None
    if not args.no_camera:
        camera = open_camera(args.camera_timeout)
        from feeding_deployment.perception.tf_interface import TFInterface

        if not args.no_joint_states:
            # Without this nothing feeds robot_state_publisher (the launch's own bridge
            # reads arm_server, which teleop has faulted), so tf2 has no arm chain.
            from feeding_deployment.ros2.joint_state_bridge import JointStateBridge

            JointStateBridge(latest, rate_hz=min(args.state_hz, 50.0))
            time.sleep(1.0)  # let the tree populate before the first lookup
        tf_interface = TFInterface()

    # --- frame saving --------------------------------------------------------------
    png_params = [cv2.IMWRITE_PNG_COMPRESSION, args.png_compression]

    def save_frames():
        last_header_stamp = None
        while not stop.is_set():
            loop_start = time.time()
            data = camera.get_camera_data()
            info, rgb, depth = data["camera_info"], data["rgb_image"], data["depth_image"]
            state, state_stamp = latest.get()
            header_stamp = (info.header.stamp.sec, info.header.stamp.nanosec)

            if rgb is None or depth is None or header_stamp == last_header_stamp:
                time.sleep(0.05)  # camera stalled or nothing new; do not save a duplicate
                continue

            sample = state_record(state, state_stamp, args.still_vel)
            forced = force_frame.is_set()
            # Record the drive and the arrival; stop once the view is just the same
            # parked scene over and over.
            idle_for = time.time() - motion["last_t"]
            if not args.all_frames and idle_for > args.settle_sec and not forced:
                counters["idle"] += 1
                time.sleep(max(0.0, 1.0 / args.frame_hz - (time.time() - loop_start)))
                continue

            transform = lookup_base_from_camera(tf_interface, info, args.tf_timeout)
            if transform is None:
                counters["no_tf"] += 1
                time.sleep(max(0.0, 1.0 / args.frame_hz - (time.time() - loop_start)))
                continue
            last_header_stamp = header_stamp
            force_frame.clear()
            counters["forced"] += int(forced)

            write_start = time.time()
            frame_dir = frames_dir / f"{counters['frames']:06d}"
            frame_dir.mkdir(parents=True, exist_ok=True)
            rgb_path = frame_dir / f"frame_rgb.{args.rgb_format}"
            if args.rgb_format == "jpg":
                cv2.imwrite(str(rgb_path), rgb, [cv2.IMWRITE_JPEG_QUALITY, args.jpg_quality])
            else:
                cv2.imwrite(str(rgb_path), rgb, png_params)
            # uint16 millimetres: what pixel2World and the replay tools expect. The
            # driver hands out float32 mm (unscaled), so this is a cast, not a rescale.
            cv2.imwrite(str(frame_dir / "frame_depth.png"),
                        np.nan_to_num(np.asarray(depth)).astype(np.uint16), png_params)
            (frame_dir / "frame_detection_inputs.json").write_text(json.dumps({
                "detector": "teleop_recording",
                "camera_info": tf_interface.camera_info_to_dict(info),
                "transforms": {TF_KEY: tf_interface.transform_to_dict(transform)},
                "params": {"source": "record_teleop_demo.py"},
            }, indent=2))
            # arm_sample_age_s is how stale the pose label is relative to the image.
            # At 30 Hz it is <33 ms, but it is the number that tells you whether a
            # disagreement between detector and label is a sync artefact.
            (frame_dir / "frame_state.json").write_text(json.dumps({
                **sample,
                "camera_stamp": header_stamp[0] + header_stamp[1] * 1e-9,
                "arm_sample_age_s": round(time.time() - state_stamp, 4),
                # Seconds since the arm was last driven. 0 means this frame was taken
                # mid-motion (expect blur, and a pose label with real sync error);
                # a settled frame is the one to trust as ground truth.
                "idle_for_s": round(idle_for, 3),
                "forced_by_tag": forced,
            }, indent=2))

            counters["frames"] += 1
            # Encoding runs inline, so if it ever exceeds the frame period the real
            # rate quietly drops below --frame-hz. Reported rather than hidden.
            counters["write_ms"] += (time.time() - write_start) * 1000
            if counters["frames"] % 10 == 0:
                print(f"  {counters['frames']} frames, {dir_size_mb(out):.0f} MB", flush=True)
            if free_gb(out) < args.min_free_gb:
                print(f"\n  STOPPING: free disk below {args.min_free_gb} GB")
                stop.set()
                return
            time.sleep(max(0.0, 1.0 / args.frame_hz - (time.time() - loop_start)))

    if camera is not None:
        camera_info_dict = tf_interface.camera_info_to_dict(camera.get_camera_data()["camera_info"])
        threading.Thread(target=save_frames, name="frame-save", daemon=True).start()

    # --- tagging console ------------------------------------------------------------
    started = time.time()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    print(f"\nrecording -> {out}")
    frame_note = "" if camera is None else f", frames {args.frame_hz:g} Hz ({args.rgb_format})"
    print(f"  arm {args.state_hz:g} Hz{frame_note}")
    print("  <Enter> mark   |   word<Enter> mark with a label   |   q<Enter> or Ctrl-C stop\n")
    tag("start")

    def watch_stdin():
        for line in sys.stdin:
            if stop.is_set():
                return
            label = line.strip()
            if label.lower() in ("q", "quit", "stop"):
                stop.set()
                return
            tag(label or "mark")

    threading.Thread(target=watch_stdin, name="stdin", daemon=True).start()

    while not stop.is_set():
        if time.time() - started > args.max_minutes * 60:
            print(f"\n  STOPPING: hit --max-minutes {args.max_minutes:g}")
            break
        time.sleep(0.2)
    stop.set()
    tag("stop")
    time.sleep(0.3)  # let the threads finish their current write

    # --- ground truth + metadata -----------------------------------------------------
    grasps = [json.loads(l) for l in (out / "events.jsonl").read_text().splitlines()]
    grasps = [e for e in grasps if e["label"] == "grasp"]
    if grasps:
        # record_ground_truth.py's schema, so the same tooling reads it. Written per-run
        # only -- the shared ~/captures/ground_truth.jsonl ledger is not touched here.
        (out / "ground_truth.json").write_text(json.dumps({
            "stamp": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(grasps[0]["t"])),
            "tag": args.tag or out.name,
            "ee_pos": grasps[0].get("ee_pos"),
            "ee_quat": grasps[0].get("ee_quat"),
            "joints": grasps[0].get("joints"),
            "gripper": grasps[0].get("gripper"),
            "note": args.note,
            "captures": [str(out)],
            "source": "record_teleop_demo.py (gripper-close during teleop)",
        }, indent=2))

    calib = DEFAULT_CALIB_PATH
    (out / "meta.json").write_text(json.dumps({
        "tag": args.tag,
        "note": args.note,
        "started": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(started)),
        "duration_sec": round(time.time() - started, 1),
        "args": {k: str(v) for k, v in vars(args).items()},
        "host": os.uname().nodename,
        "git_commit": git_commit(),
        "arm_state_last_seen": counters.get("arm_state"),
        "camera_info": camera_info_dict,
        "calibration_file": str(calib),
        "calibration": calib.read_text() if calib.is_file() else None,
        "counts": {
            "frames": counters["frames"],
            "states": states.count,
            "events": events.count,
            "frames_forced_by_tag": counters["forced"],
            "frames_dropped_no_tf": counters["no_tf"],
            "frames_skipped_idle": counters["idle"],
        },
        "mean_frame_write_ms": (round(counters["write_ms"] / counters["frames"], 1)
                                if counters["frames"] else None),
        "size_mb": round(dir_size_mb(out), 1),
    }, indent=2))

    states.close()
    events.close()
    print(f"\n{counters['frames']} frames, {states.count} arm samples, "
          f"{events.count} events, {dir_size_mb(out):.0f} MB")
    if counters["no_tf"]:
        print(f"  {counters['no_tf']} frames dropped: no {TF_KEY} at their stamp")
    if counters["idle"]:
        print(f"  {counters['idle']} frames skipped: arm parked for more than "
              f"{args.settle_sec:g} s")
    if counters["frames"]:
        mean_ms = counters["write_ms"] / counters["frames"]
        note = "" if mean_ms < 1000.0 / args.frame_hz else "  <- SLOWER than --frame-hz"
        print(f"  {mean_ms:.0f} ms mean encode+write per frame{note}")
    if camera is not None and counters["frames"] < 3:
        print(f"  ONLY {counters['frames']} frames saved. If the arm really was being "
              f"driven, lower --still-vel; --all-frames records regardless of motion.")
    print(f"  {out}")
    print(f"  replay: $PY -u scripts/session/replay_detection.py {out}/frames/*")
    return 0


if __name__ == "__main__":
    sys.exit(main())
