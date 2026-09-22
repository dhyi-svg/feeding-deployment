#!/usr/bin/env python3
"""Autonomous microwave button press: two-dots visual servo + force stop.

DRY RUN BY DEFAULT. Nothing moves without ``--execute``. With ``--execute`` the arm
moves in small, individually gated joint steps and stops on the first surprise.

How it works (pure translation -- the wrist orientation is never changed):

  The wrist camera is rigid to the gripper, so the LEFT fingertip is always at the same
  pixel (``/button_detector/claw_pixel``). If the button's pixel
  (``/button_detector/button_pixel``) sits on that pixel, the button lies on the
  fingertip's line of sight. Translating the wrist along that ray keeps it there until
  the fingertip touches the button. So:

    stage 1  lateral servo    move in the camera's image plane until |button - claw| < PX_TOL
    stage 2  approach         step along the fingertip ray; after EVERY step, with the arm at
                              rest, read the tool force from the press detector; contact when
                              |dF| > CONTACT_N; abort if > FORCE_ABORT_N or the travel cap
    stage 3  press            one more PRESS_TRAVEL step, hold, retract PRESS_RETRACT
    stage 4  retract          back to the stage-1 standoff (``--home``: joint-move to start)

  Lateral error closes on pixels; depth error closes on force. The only calibration used
  is the ROTATION arm_base_link <- camera (tf2), to turn camera-frame directions into
  base-frame directions -- a few degrees of error there just costs a servo iteration.

What has to be running (see TONIGHT_RUNBOOK.md "motion" section for the exact commands):
  arm_server.py + stub_base_server.py + bulldog_bypass.py, speed set LOW
  robot_state_publisher + joint_state_bridge + calibration_tf (tf chain; verify with
      ros2 run tf2_ros tf2_echo arm_base_link camera_color_optical_frame)
  realsense2_camera with align_depth
  button_detector_node.py  (-p target_button:=<name>)
  detect_button_press_force.py --publish   (baselined with the arm PARKED)

The travel cap (stage 2) needs ``--tip-dist``: the straight-line distance in metres from
the camera lens to the LEFT fingertip. Measure it with a ruler. Over-estimating it makes
the cap conservative (stops short); under-estimating relies on the force stop. The cap is
    L_max = (distance along the fingertip ray from the camera to the panel plane)
            - tip_dist + PLANE_OVERSHOOT_M
and ``--cap-override`` lowers it further for the first approach tests. On this rig
tip_dist = 0.154 (measured 2026-09-21 from the first contact).

Stopping it: the physical e-stop is the only stop proven on this rig. Killing
bulldog_bypass.py e-stops the arm within ~1 s. Ctrl-C here stops the NEXT step from being
sent -- the in-flight step (<= 1 cm, <= MAX_JOINT_STEP_DEG on any joint) completes. On any
abort the arm is left where it is and the way back is printed; nothing auto-retracts.

Usage ladder (one invocation per rung, a human at the e-stop for every --execute):
    $PY press_button_autonomous.py                              # dry run, all stages planned
    $PY press_button_autonomous.py --execute --stage 1          # lateral servo only
    $PY press_button_autonomous.py --execute --stage 2 --tip-dist 0.12 --cap-override 0.05
    $PY press_button_autonomous.py --execute --stage 2 --tip-dist 0.12   # until contact
    $PY press_button_autonomous.py --execute --tip-dist 0.12 --presses 1 # the real thing
Run with the *prepend* form PYTHONPATH=$PWD/src:$PYTHONPATH (PYTHONPATH=src drops rclpy).
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from collections import deque
from pathlib import Path

import os
import signal
import subprocess

import cv2
import numpy as np
import pybullet as p
import rclpy
import tf2_ros
from rclpy.duration import Duration
from rclpy.time import Time
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PolygonStamped, Vector3Stamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String

from feeding_deployment.control.robot_controller.arm_client import ArmInterfaceClient
from feeding_deployment.control.robot_controller.command_interface import JointCommand
from feeding_deployment.simulation.scene_description import create_scene_description_from_config
from feeding_deployment.simulation.simulator import FeedingDeploymentPyBulletSimulator

# ---- servo / approach ------------------------------------------------------------------
PX_TOL = 4.0                 # button pixel within this of the claw pixel counts as aligned
# Frames to median-filter the button pixel over before steering on it. A single frame is
# not safe: on 2026-09-21 the pixel sat at (372.7, 399.5) +-(2.2, 0.5) px while the node
# matched reference view 3, but ~1 frame in 30 matched a DIFFERENT view and placed the
# button 29 px away in x. The servo took instantaneous samples, swallowed those outliers
# and lurched (commanded zero x correction, measured +9 px of x change), never settling
# under PX_TOL. A median rejects them; measured p90 error vs the long-run mean was
# 3.31 px for 1 frame, 2.18 for 4, 1.65 for 8, 1.31 for 12.
PX_SAMPLES = 9
PX_SAMPLES_MIN = 3           # accept a shorter median rather than failing outright
SERVO_MAX_ITERS = 6          # from 32 cm the first correction is capped, so allow more rounds
SERVO_MAX_STEP_M = 0.03      # a single lateral correction is capped here
APPROACH_STEP_M = 0.010
FINE_STEP_M = 0.003
FINE_ZONE_M = 0.04           # switch to FINE_STEP_M this close to the travel cap
# Travel cap = (camera->plane along the ray) - tip_dist + PLANE_OVERSHOOT_M. The force stop
# is the primary terminator; the cap only guards against a gross depth/tip error, so it is
# allowed PAST the nominal plane (the dome sits ~5 mm proud of it and the depth has a few
# mm of bias this close). 2026-09-21: tip_dist measured 0.154 m on this rig (plane 17.0 cm
# along the ray, contact at 1.6 cm), so the cap from a 30 cm standoff is ~16 cm.
PLANE_OVERSHOOT_M = 0.015
RESERVO_EVERY_M = 0.03       # re-check lateral alignment this often during the approach
RESERVO_MAX_STEP_M = 0.01
# ---- far phase -------------------------------------------------------------------------
# The claw pixel is near the bottom of the frame, so putting the button ON it drags the
# panel behind the finger. From far away the panel is small and disappears entirely
# (2026-09-21, 32 cm standoff: locked -> "quad not convex"). So beyond CLOSE_STANDOFF_M the
# button is held HOLD_DY_PX above the claw pixel and the wrist advances along THAT ray; the
# proven servo-onto-claw only starts once the panel is close enough to stay visible.
HOLD_DY_PX = 110.0
CLOSE_STANDOFF_M = 0.20      # camera->panel along the hold ray at which the close phase starts
# 1.5 cm, not 2 cm: at the 2026-09-21 far-standoff posture the arm needs ~4.6 deg of joint
# motion per cm, so a 2 cm step plans 9.3 deg against the 10 deg MAX_JOINT_STEP_DEG gate --
# 0.7 deg of headroom, and one noisy solve aborts the run. 1.5 cm plans ~7 deg for the cost
# of ~2 extra steps. Shrink the step rather than open the gate.
FAR_STEP_M = 0.015
FAR_RESERVO_EVERY_M = 0.04
# ---- force -----------------------------------------------------------------------------
# Contact = BOTH of: the at-rest |dF| has risen CONTACT_N above its value at the start of
# the approach, AND it rose at least JUMP_N within the last single step. The second test
# is what separates a fingertip meeting a rigid panel (sharp rise inside one 3 mm step)
# from the slow ramps seen on 2026-09-21: Kinova's compensated wrench drifted ~2.6 N for a
# 2 cm move and another ~2.6 N over 9 mm of approach (0.5-0.75 N per step) with the
# fingertip verifiably touching nothing. An absolute threshold alone called that contact.
# Contact detection is only ARMED once contact is geometrically possible: within
# CONTACT_ARM_MARGIN_M of the expected fingertip-to-panel distance (s_panel - tip_dist).
# Outside that zone a force jump cannot be the button. On 2026-09-21 the approach declared
# CONTACT at 1.0 cm travel when the panel was 6.8 cm away -- a +3.39 N phantom jump from a
# single 1 cm step (4.4 deg of joint motion; Kinova's external-wrench ESTIMATE is strongly
# pose-dependent). The arm never touched anything and the "press" pressed air. The earlier
# successful presses never hit this because they started 1.6 cm out and stepped 2 mm at a
# time, which barely moves the joints. This is the same reasoning the far phase already
# uses ("no contact is geometrically possible there").
CONTACT_ARM_MARGIN_M = 0.03
# While unarmed, only a genuine collision should stop us. Phantom jumps observed at 3-5 N,
# a real fingertip-on-panel contact at 2.25-5.2 N, a hand push 20-50 N. 8 N is above the
# phantom band and well below anything that could damage the panel.
COARSE_ABORT_N = 8.0
# Contact is declared only on a MONOTONIC ramp, never on magnitude alone. Measured
# 2026-09-22 with the arm at rest and a fresh baseline, the wrench itself resolves to
# 0.074 +- 0.042 N (max excursion 0.277 N over 30 s, drift 0.003 N) -- it is 30-200x more
# precise than the contact forces we care about. The multi-newton "noise" we kept tripping
# on is not sensor noise at all but a POSE-DEPENDENT BIAS, which is why filtering does not
# help (1 s of averaging only takes std 0.075 -> 0.021 N; there is nothing high-frequency
# to remove). Magnitude cannot separate the two cases:
#     real contact   +7.80 N then +5.97 N   (rising, rising)
#     worst phantom  +6.30 N then -2.96 then +4.01 then -4.74   (oscillating)
# Bias wander reverses; a fingertip driven into a rigid panel never does. So a candidate
# contact must be CONFIRMED by one more fine step that rises again.
# The candidate test is a rise over a WINDOW of steps, not a single-step jump. Contact
# here builds gradually -- on 2026-09-22 the force climbed from the very first 1 mm step
# (0.12 -> 2.33 -> 2.28 -> 3.74 -> 4.75 -> 4.51 -> 4.49 -> 5.55 -> 7.58 -> 8.00 -> 11.11 N)
# so no SINGLE step jumped 3 N until 11 N, and confirming took it to 14.6 and the press to
# 16.4 N (abort). Summing over 3 steps sees the same ramp at 7.6 N instead, while still
# rejecting the phantom wander, which cancels itself over a window rather than accumulating.
CONTACT_WINDOW = 3           # steps to sum the rise over
CONTACT_N = 2.5              # rise over that window to become a candidate
JUMP_N = 3.0                 # kept: a single step this big is also a candidate
CONFIRM_RISE_N = 1.5         # the confirming step must add at least this much again
FORCE_ABORT_N = 15.0         # anything above this is not a button
FORCE_FREE_N = 1.0           # preflight: the tool must be this free
FORCE_STALE_S = 0.5          # press detector considered dead after this silence
FORCE_SETTLE_S = 0.3         # wait this long after a step before reading force
# Settle before re-taking the baseline. 2.8 s was not enough after a multi-cm move:
# on 2026-09-22 the only jog leg baselined straight after a 1.5 cm retract wandered
# 2-7.5 N, while every later leg (same 2 mm steps) stayed inside 0.6-2.8 N.
REBASELINE_SETTLE_S = 5.0
# ---- press -----------------------------------------------------------------------------
PRESS_TRAVEL_M = 0.003
PRESS_HOLD_S = 0.3
PRESS_RETRACT_M = 0.02
# Cap on a single commanded move ALONG A RAY. The joint-jump gate (MAX_JOINT_STEP_DEG)
# is in joint space, but these moves are specified in metres, and how many degrees a
# centimetre costs depends entirely on the posture: at the 2026-09-21 close standoff it
# was ~5 deg/cm, so the 2 cm press retract planned 10.1 deg and the gate refused AFTER
# the button had been pressed. Chunk every ray move to this size and the gate stops
# being reachable by a move that is merely long rather than wrong.
RAY_STEP_MAX_M = 0.01
# ---- motion gates (same family as the grasp scripts; tighter where the moves are smaller)
MAX_IK_ERR_M = 0.005
SEED_GOOD_M = 0.001          # posture seed is honoured unquestioned below this; see solve_translation
MAX_JOINT_STEP_DEG = 10.0
MAX_REACH_M = 0.91           # existing rig constant -- do not raise without asking
Z_RANGE_M = (0.25, 0.75)
TRACK_ABORT_M = 0.01
CONVERGE_TOL_DEG = 1.0
# ---- perception gates ------------------------------------------------------------------
MIN_INLIERS = 12             # to START a run (preflight); 6-8-inlier locks have picked the wrong dome
MIN_INLIERS_TRACK = 8        # to accept a re-servo correction mid-transit (a weak lock just skips it)
LOCK_HOLD_S = 2.0
FRESH_S = 1.0
PANEL_SAT_MIN = 100          # HSV saturation above which a pixel is red panel, not chrome
DEPTH_RANGE_M = (0.10, 1.5)
PLANE_Z_BAND_M = 0.05        # drop depths this far off the median (background seen through gaps)
PLANE_TRIM_M = 0.02          # refit after dropping points this far off the first fit
ARM = [1, 2, 3, 4, 5, 6, 7]
SCENE_CONFIG = "src/feeding_deployment/simulation/configs/vention.yaml"


class Abort(SystemExit):
    pass


# =============================================================================================
# Perception side: one rclpy node holding the latest of everything, spun in a thread.
# =============================================================================================
class Perception(Node):
    def __init__(self, ns: str, press_ns: str, arm_frame: str, cam_frame: str):
        super().__init__("press_button_autonomous")
        self.arm_frame, self.cam_frame = arm_frame, cam_frame
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest: dict[str, tuple[float, object]] = {}
        self.force_hist: deque[tuple[float, float]] = deque(maxlen=200)  # (t, |dF|)
        self.tfbuf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfbuf, self)

        def keep(name, conv=lambda m: m):
            def cb(msg):
                with self.lock:
                    self.latest[name] = (time.monotonic(), conv(msg))
            return cb

        self.create_subscription(PointStamped, f"{ns}/button_pixel",
                                 keep("button_px", lambda m: np.array([m.point.x, m.point.y])), 10)
        self.create_subscription(PointStamped, f"{ns}/claw_pixel",
                                 keep("claw_px", lambda m: np.array([m.point.x, m.point.y])), 10)
        self.create_subscription(String, f"{ns}/status", keep("status", lambda m: m.data), 10)
        self.create_subscription(PolygonStamped, f"{ns}/panel_quad",
                                 keep("quad", lambda m: np.array([[q.x, q.y] for q in m.polygon.points])), 10)
        self.create_subscription(CameraInfo, "/camera/color/camera_info", keep("info"), 10)
        self.create_subscription(Image, "/camera/color/image_raw", keep("color"), 1)
        self.create_subscription(Image, "/camera/aligned_depth_to_color/image_raw", keep("depth"), 1)
        self.create_subscription(Bool, f"{press_ns}/pressed", keep("pressed", lambda m: bool(m.data)), 10)

        def on_force(msg):
            v = np.array([msg.vector.x, msg.vector.y, msg.vector.z])
            with self.lock:
                self.latest["force"] = (time.monotonic(), v)
                self.force_hist.append((time.monotonic(), float(np.linalg.norm(v))))
        self.create_subscription(Vector3Stamped, f"{press_ns}/force_dev", on_force, 50)

        self._thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        self._thread.start()

    # -- accessors ---------------------------------------------------------------------------
    def get(self, name, max_age=None):
        with self.lock:
            item = self.latest.get(name)
        if item is None:
            return None
        t, v = item
        if max_age is not None and time.monotonic() - t > max_age:
            return None
        return v

    def age(self, name):
        with self.lock:
            item = self.latest.get(name)
        return None if item is None else time.monotonic() - item[0]

    def wait_after(self, name, t_after, timeout):
        """Latest value of `name` received strictly after monotonic time t_after, or None.

        Needed after a move: the detector node runs at ~5 Hz and a value up to
        FRESH_S old can predate the motion, which made the servo act on pre-move
        pixels (each iteration then only removed ~half the error, 2026-09-21).
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self.lock:
                item = self.latest.get(name)
            if item is not None and item[0] > t_after:
                return item[1]
            time.sleep(0.05)
        return None

    def median_after(self, name, t_after, n_samples=PX_SAMPLES, timeout=4.0):
        """Median of up to `n_samples` DISTINCT values of `name` received after t_after.

        Outlier-rejecting counterpart to wait_after(): see PX_SAMPLES for why a single
        frame is not safe to steer on. Returns None if fewer than PX_SAMPLES_MIN arrive
        before the timeout.
        """
        deadline = time.monotonic() + timeout
        seen = []
        last_t = t_after
        while time.monotonic() < deadline and len(seen) < n_samples:
            with self.lock:
                item = self.latest.get(name)
            if item is not None and item[0] > last_t:
                last_t = item[0]
                seen.append(item[1])
            else:
                time.sleep(0.02)
        if len(seen) < PX_SAMPLES_MIN:
            return None
        return np.median(np.stack(seen), axis=0)

    def wait(self, name, timeout, max_age=FRESH_S):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            v = self.get(name, max_age)
            if v is not None:
                return v
            time.sleep(0.05)
        return None

    def locked(self, min_inliers=MIN_INLIERS) -> tuple[bool, str]:
        st = self.get("status", FRESH_S)
        if st is None:
            return False, "no status (node not running?)"
        if not st.startswith("locked"):
            return False, st
        try:
            inl = int(st.split("inliers=")[1].split()[0])
        except (IndexError, ValueError):
            inl = 0
        return inl >= min_inliers, st

    def lock_target(self) -> str | None:
        st = self.get("status", FRESH_S)
        return st.split()[1] if st and st.startswith("locked") and len(st.split()) > 1 else None

    def force_at_rest(self, window_s=FORCE_SETTLE_S) -> float:
        """Median |dF| over the last window; NaN if the feed is stale."""
        if (self.age("force") or 1e9) > FORCE_STALE_S:
            return float("nan")
        now = time.monotonic()
        with self.lock:
            vals = [m for t, m in self.force_hist if now - t <= window_s]
        return float(np.median(vals)) if vals else float("nan")

    def intrinsics(self):
        info = self.get("info")
        if info is None:
            raise Abort("no camera_info")
        return info.k[0], info.k[4], info.k[2], info.k[5]

    def cam_rotation_in_base(self) -> np.ndarray:
        """3x3 rotation taking camera-frame directions into arm-base directions."""
        from scipy.spatial.transform import Rotation as R  # noqa: PLC0415
        try:
            tr = self.tfbuf.lookup_transform(self.arm_frame, self.cam_frame, Time(),
                                             timeout=Duration(seconds=2.0))
        except Exception as e:  # noqa: BLE001
            raise Abort(f"tf {self.arm_frame} <- {self.cam_frame} unavailable: {e}\n"
                        "  /joint_states silent? restart joint_state_bridge; verify with\n"
                        f"  ros2 run tf2_ros tf2_echo {self.arm_frame} {self.cam_frame}")
        q = tr.transform.rotation
        return R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

    def ray(self, px) -> np.ndarray:
        fx, fy, cx, cy = self.intrinsics()
        r = np.array([(px[0] - cx) / fx, (px[1] - cy) / fy, 1.0])
        return r / np.linalg.norm(r)

    def panel_plane(self):
        """Plane n.X = d (camera frame) fitted to the red panel inside the homography quad.

        Chrome domes (low saturation) are excluded: they are exactly where the depth
        sensor lies. Returns (n, d, median_depth, n_points).
        """
        quad = self.get("quad", FRESH_S)
        color = self.get("color", FRESH_S)
        depth = self.get("depth", FRESH_S)
        if quad is None or color is None or depth is None:
            raise Abort("panel plane: need fresh quad + color + aligned depth "
                        f"(quad {self.age('quad')}, color {self.age('color')}, depth {self.age('depth')})")
        bgr = self.bridge.imgmsg_to_cv2(color, "bgr8")
        d = self.bridge.imgmsg_to_cv2(depth, "passthrough").astype(np.float32)
        if depth.encoding in ("16UC1", "mono16"):
            d = d / 1000.0
        h, w = d.shape[:2]
        mask = np.zeros((h, w), np.uint8)
        cv2.fillConvexPoly(mask, quad.astype(np.int32), 255)
        sat = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)[..., 1]
        ok = (mask > 0) & (sat > PANEL_SAT_MIN) & np.isfinite(d) & (d > DEPTH_RANGE_M[0]) & (d < DEPTH_RANGE_M[1])
        vs, us = np.where(ok)
        if len(us) < 200:
            raise Abort(f"panel plane: only {len(us)} valid panel depth pixels inside the quad")
        fx, fy, cx, cy = self.intrinsics()
        z = d[vs, us]
        # Fit by ORDINARY least squares on the depth map, not total-least-squares (SVD) on the
        # 3D points. u,v are exact; all the noise is in z. A plane is exactly linear in inverse
        # depth: 1/z = m.[u',v',1] with m = n/d. TLS instead assumes isotropic noise, and on
        # 2026-09-21 at the 20 cm standoff that broke badly -- the vertical depth signal across
        # the panel was only 0.30 cm against 0.64 cm per-pixel noise, so SVD read the weak axis
        # as geometry and returned a 61 deg tilt ([0.11 0.88 -0.47]). OLS averages the noise
        # down and recovers [0.15 0.29 -0.95] (19 deg), which matches both the fits taken
        # further out and the quad's 1.019 keystone ratio. It worked at 29.8 cm only because
        # the panel filled more of the frame and carried more depth signal.
        up = (us - cx) / fx
        vp = (vs - cy) / fy
        zz = z
        # Background seen through gaps around the panel is only ~2% of the pixels but sits at
        # ~37 cm vs the panel's ~21 cm, and 1/z gives it enormous leverage. Drop it first.
        keep0 = np.abs(zz - np.median(zz)) <= PLANE_Z_BAND_M
        if keep0.sum() >= 200:
            up, vp, zz = up[keep0], vp[keep0], zz[keep0]
        m = None
        for _ in range(2):
            A = np.stack([up, vp, np.ones_like(up)], axis=1)
            m, *_ = np.linalg.lstsq(A, 1.0 / zz, rcond=None)
            pred = 1.0 / (A @ m)
            keep = np.abs(zz - pred) <= PLANE_TRIM_M
            if keep.sum() < 200 or bool(keep.all()):
                break
            up, vp, zz = up[keep], vp[keep], zz[keep]
        A = np.stack([up, vp, np.ones_like(up)], axis=1)
        resid = float(np.std(zz - 1.0 / (A @ m)))
        nrm = float(np.linalg.norm(m))
        n, d_plane = m / nrm, 1.0 / nrm
        if n[2] > 0:          # make the normal point back toward the camera (-z); d flips with it
            n, d_plane = -n, -d_plane
        if resid > 0.01:
            raise Abort(f"panel plane: residual {resid*100:.1f} cm -- not planar / bad depth")
        return n, d_plane, float(np.median(zz)), len(zz)


# =============================================================================================
# Arm side: seeded PyBullet IK, gated joint steps, convergence-checked execution.
# =============================================================================================
def _wait_converged(ai, q_cmd, tol_deg=CONVERGE_TOL_DEG, timeout_s=6.0):
    """Block until the arm's joints are within tol_deg of q_cmd and at rest.

    A plain "velocity ~ 0" wait is not enough: Kortex's blocking move returns on
    ACTION_END *or* ACTION_ABORT, so the next command can land while the arm is still
    moving and be rejected (ROBOT_MOVEMENT_IN_PROGRESS) -- that sub-step is silently
    skipped. Seen 9 times in one evening's arm log on this rig (2026-09-20).
    """
    q_cmd = np.asarray(q_cmd, dtype=float)
    deadline = time.time() + timeout_s
    derr = float("inf")
    while time.time() < deadline:
        time.sleep(0.12)
        st = ai.get_state()
        qa = np.asarray(st["position"], dtype=float)
        derr = float(np.degrees(np.max(np.abs((qa - q_cmd + np.pi) % (2 * np.pi) - np.pi))))
        vel = float(np.max(np.abs(np.asarray(st["velocity"], dtype=float))))
        if derr < tol_deg and vel < 1e-3:
            break
    return derr


class Arm:
    def __init__(self, execute: bool):
        self.execute = execute
        self.ai = ArmInterfaceClient()
        scene = create_scene_description_from_config(SCENE_CONFIG, "skewer")
        self.sim = FeedingDeploymentPyBulletSimulator(scene, use_gui=False)
        self.rb = self.sim.robot
        self.base_pos = np.asarray(scene.robot_base_pose.position, dtype=float)
        bq = np.asarray(scene.robot_base_pose.orientation, dtype=float)
        # The grasp scripts treat base-frame == world-frame directions (fk() subtracts only
        # the base position). Hold that assumption explicitly instead of inheriting it.
        if abs(abs(bq[3]) - 1.0) > 1e-3:
            raise Abort(f"scene robot_base_pose is rotated ({bq}); this script assumes identity")

    # -- state --------------------------------------------------------------------------------
    def state(self):
        return self.ai.get_state()

    def joints(self):
        return np.asarray(self.state()["position"], dtype=float)

    def ee_pos(self):
        return np.asarray(list(self.state()["ee_pos"])[:3], dtype=float)

    def arm_state_name(self):
        try:
            return self.ai._arm_interface.get_arm_state()["name"]  # noqa: SLF001
        except Exception as e:  # noqa: BLE001
            return f"unknown ({type(e).__name__})"

    # -- kinematics ---------------------------------------------------------------------------
    def _set_sim(self, q):
        for i, jj in enumerate(ARM):
            p.resetJointState(self.rb.robot_id, jj, float(q[i]), physicsClientId=self.rb.physics_client_id)

    def fk(self, q):
        """(world position, world quaternion xyzw) of the sim EE for joint vector q."""
        self._set_sim(q)
        ls = p.getLinkState(self.rb.robot_id, self.rb.end_effector_id, physicsClientId=self.rb.physics_client_id)
        return np.asarray(ls[4], dtype=float), np.asarray(ls[5], dtype=float)

    def solve_translation(self, q_cur, d_base, posture=None):
        """IK for 'current EE + d_base', same orientation.

        The target is relative to the sim's own FK of q_cur (not Kinova's ee_pos), so any
        constant sim-vs-Kortex tool-frame offset cancels instead of becoming a jump on
        step 1. The IK is SEEDED FROM `posture` (the run's start joints) rather than from
        q_cur: the Gen3 is redundant, and re-seeding from the current pose every step let
        the solution slide along the self-motion manifold -- on 2026-09-21 J1/J3
        counter-rotated 7 -> 10.6 deg per identical 2 cm step (30 deg total in 6 cm), which
        both tripped the joint-jump gate and shifted the wrench estimate by several N.
        Seeding from a fixed posture keeps every solution near one configuration. The anchor
        is honoured outright while it solves to better than SEED_GOOD_M; beyond that (a stale
        anchor, i.e. the arm has travelled away from it) the q_cur seed is solved too and the
        more accurate of the two wins. Run.reanchor_seed() refreshes the anchor per phase.
        Returns (q, ik_err_m, target_world, jump_deg, gate_failures).
        """
        pos, quat = self.fk(q_cur)
        target = pos + np.asarray(d_base, dtype=float)
        seeds = [posture, q_cur] if posture is not None else [q_cur]
        q = None
        err = None
        for seed in seeds:
            self._set_sim(seed)
            sol = p.calculateInverseKinematics(
                self.rb.robot_id, self.rb.end_effector_id, list(target), list(quat),
                maxNumIterations=400, residualThreshold=1e-5, physicsClientId=self.rb.physics_client_id)
            q_try = np.asarray(sol[:7], dtype=float)
            got, _ = self.fk(q_try)
            err_try = float(np.linalg.norm(got - target))
            if q is None or err_try < err:
                q, err = q_try, err_try
            # Take the posture seed the moment it is GOOD, not merely legal. Accepting the
            # first solution under MAX_IK_ERR_M (5 mm) is what broke the 2026-09-21 stage-1
            # servo: a stale anchor returned ~3 mm error, passed the gate, and the more
            # accurate q_cur seed was never tried -- while the corrections being asked for
            # were themselves only 1-4 mm. Below SEED_GOOD_M the anchor is honoured (no
            # null-space drift); above it, both seeds are solved and the closer one wins.
            if err <= SEED_GOOD_M:
                break
        bad = []
        tb = target - self.base_pos
        if np.linalg.norm(tb) > MAX_REACH_M:
            bad.append(f"reach {np.linalg.norm(tb):.3f} > {MAX_REACH_M}")
        if not (Z_RANGE_M[0] <= tb[2] <= Z_RANGE_M[1]):
            bad.append(f"z {tb[2]:.3f} outside {Z_RANGE_M}")
        if err > MAX_IK_ERR_M:
            bad.append(f"IK err {err*100:.2f} cm > {MAX_IK_ERR_M*100:.1f}")
        jump = float(np.degrees(np.max(np.abs((q - q_cur + np.pi) % (2 * np.pi) - np.pi))))
        if jump > MAX_JOINT_STEP_DEG:
            bad.append(f"joint jump {jump:.1f} deg > {MAX_JOINT_STEP_DEG}")
        return q, err, target, jump, bad

    # -- execution ----------------------------------------------------------------------------
    def step(self, d_base, name, log, posture=None):
        """Plan + gate + (if executing) move the EE by d_base. Returns the joint vector reached."""
        q0 = self.joints()
        ee0 = self.ee_pos()
        q, err, target, jump, bad = self.solve_translation(q0, d_base, posture=posture)
        rec = {"step": name, "d_base": list(map(float, d_base)), "ik_err_m": err, "jump_deg": jump,
               "gates": bad, "q": q.tolist(), "ee_before": ee0.tolist(), "executed": False}
        print(f"  {name:22s} d={np.round(d_base*100, 2)} cm  IK {err*100:.2f} cm  jump {jump:4.1f} deg"
              f"  {'FAIL: ' + '; '.join(bad) if bad else 'ok'}")
        if bad:
            log(rec)
            raise Abort(f"gate failed at {name}: {'; '.join(bad)} -- arm not commanded")
        if not self.execute:
            log(rec)
            return q
        self.ai.execute_command(JointCommand(pos=q.tolist()))
        derr = _wait_converged(self.ai, q)
        if derr >= CONVERGE_TOL_DEG:
            print(f"  {name}: settled {derr:.1f} deg short (Kortex likely dropped it) -- re-sending once")
            time.sleep(0.5)
            self.ai.execute_command(JointCommand(pos=q.tolist()))
            derr = _wait_converged(self.ai, q)
        ee1 = self.ee_pos()
        track = float(np.linalg.norm((ee1 - ee0) - np.asarray(d_base)))
        rec.update(executed=True, converge_deg=derr, ee_after=ee1.tolist(), track_err_m=track)
        log(rec)
        print(f"  {name:22s} moved {np.round((ee1-ee0)*100, 2)} cm  converge {derr:.2f} deg  track {track*100:.2f} cm")
        if derr >= CONVERGE_TOL_DEG:
            raise Abort(f"{name}: still {derr:.1f} deg off after retry -- HOLDING HERE")
        if track > TRACK_ABORT_M:
            raise Abort(f"{name}: tracking error {track*100:.1f} cm > {TRACK_ABORT_M*100:.0f} -- HOLDING HERE")
        return q


# =============================================================================================
# The stages
# =============================================================================================
class Run:
    def __init__(self, args):
        self.args = args
        self.log_dir = Path(args.log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self._log_f = open(self.log_dir / f"press_{time.strftime('%Y%m%d_%H%M%S')}.jsonl", "a")  # noqa: SIM115
        self.per = Perception(args.ns, args.press_ns, args.arm_frame, args.camera_frame)
        self.arm = Arm(args.execute)
        self.R_bc = None          # arm_base <- camera rotation
        self.ray_cam = None       # fingertip ray, camera frame, unit
        self.ray_base = None
        self.s_panel = None       # distance camera -> panel plane along the ray
        self.z_panel = None       # median camera-z of the panel
        self.travelled = 0.0      # along the fingertip ray, since the close standoff
        self.far_travelled = 0.0  # along the hold ray (far phase)
        self.far_ray_base = None
        self.start_joints = None
        # The IK seed is anchored to a posture to stop the redundant arm sliding along its
        # self-motion manifold (see Arm.solve_translation). It is NOT start_joints: --home and
        # the printed retreat path need those to stay the run's true origin. The anchor is
        # re-set at each phase boundary by reanchor_seed() -- see that method for why.
        self.seed_posture = None

    def step(self, d_base, name):
        """All EE moves go through here so every IK is seeded from the anchored posture."""
        return self.arm.step(d_base, name, self.log, posture=self.seed_posture)

    def move_along(self, unit, dist, name, far=False):
        """Translate `dist` metres along `unit`, in chunks of at most RAY_STEP_MAX_M.

        Keeps the travel bookkeeping in step with what has actually executed: if a chunk
        aborts, the chunks already done are already subtracted, so the printed "way back"
        is right. `unit` points in the direction of travel; `dist` is always positive.
        """
        remaining = float(dist)
        signed = -1.0 if np.dot(unit, self.far_ray_base if far else self.ray_base) < 0 else 1.0
        k = 0
        while remaining > 1e-4:
            stp = min(RAY_STEP_MAX_M, remaining)
            k += 1
            self.step(unit * stp, f"{name} {k} ({signed*stp*100:+.1f})")
            remaining -= stp
            if far:
                self.far_travelled += signed * stp
            else:
                self.travelled += signed * stp

    def reanchor_seed(self, why):
        """Re-anchor the IK seed to the current joints.

        A posture anchor is only a good IK seed while the arm is near it. On 2026-09-21 the
        far phase travelled 10.9 cm from the preflight posture; the stage-1 servo that followed
        then solved every correction from that stale seed and got ik_err 2.2-3.6 mm -- under
        MAX_IK_ERR_M (5 mm) so it was accepted, but the corrections themselves were only 1-4 mm,
        so each move injected more error than it removed. The button pixel oscillated
        (+2, -4, +12, +7, -4, -11 px) and never reached PX_TOL, aborting the run. Re-anchoring
        at each phase boundary keeps the anti-drift property within a phase while keeping the
        seed close enough to stay sub-millimetre.
        """
        print(f"  IK seed re-anchored to the current posture ({why})")
        if not self.execute:
            return   # nothing moved in a dry run, so the preflight anchor is still current
        self.seed_posture = self.arm.joints()

    def log(self, rec):
        rec = dict(rec, t=time.time())
        self._log_f.write(json.dumps(rec) + "\n")
        self._log_f.flush()

    def rebaseline_force(self, why):
        """Ask the running press detector (SIGUSR1) to re-take its rest baseline here.

        Kinova's compensated wrench shifts by several N with joint configuration (2026-09-21:
        +3.3 N for one 2 cm / 8.6 deg step, +2.6 N for a 2 cm servo move), and the detector
        stops adapting above 4 N, so after any sizeable move the approach would otherwise
        start from a biased, frozen reading. Arm must be at rest; nothing here moves it.
        """
        if not self.args.execute:
            return
        pids = subprocess.run(["pgrep", "-f", "^python3 -u scripts/scratch/detect_button_press_force"],
                              capture_output=True, text=True).stdout.split()
        if not pids:
            raise Abort("press detector process not found for re-baseline -- HOLDING HERE")
        print(f"  re-baselining the press detector ({why}); arm still for {REBASELINE_SETTLE_S:.1f} s ...")
        for pid in pids:
            os.kill(int(pid), signal.SIGUSR1)
        time.sleep(REBASELINE_SETTLE_S)
        f = self.force_now()
        print(f"  |dF| after re-baseline: {f:.2f} N")
        if f > FORCE_FREE_N:
            raise Abort(f"|dF| still {f:.2f} N after re-baseline -- something is loading the tool. HOLDING HERE")
        self.log({"rebaseline": why, "force_after": f})

    def force_now(self):
        f = self.per.force_at_rest()
        if math.isnan(f):
            raise Abort("press detector feed is stale/dead -- HOLDING HERE")
        return f

    # ---- stage 0 ------------------------------------------------------------------------------
    def preflight(self, require_lock=True, require_free=True, min_inliers=MIN_INLIERS):
        print("\n== stage 0: preflight ==")
        a = self.args
        st = self.arm.state()
        name = self.arm.arm_state_name()
        grip = float(st.get("gripper_pos", -1))
        speed = self.arm.ai.get_speed()
        print(f"  arm state     : {name}")
        print(f"  gripper_pos   : {grip:.3f}  ({'closed' if grip > 0.7 else 'NOT closed'})")
        print(f"  speed preset  : {speed}")
        problems = []
        if "SERVOING_READY" not in str(name) and not str(name).startswith("unknown"):
            problems.append(f"arm state {name}")
        if grip <= 0.7:
            problems.append("gripper must be CLOSED (this presses with the closed fingertips)")
        if str(speed).lower() != "low":
            problems.append(f"speed preset is {speed!r}, want 'low' (scripts/session/arm_set_speed.py low)")

        print("  waiting for camera_info / status / pixels ...")
        if self.per.wait("info", 5, None) is None:
            problems.append("no camera_info")
        # The node must hold a lock on the requested target for LOCK_HOLD_S.
        t0 = time.monotonic()
        held = 0.0
        last = ""
        while time.monotonic() - t0 < LOCK_HOLD_S + 6:
            ok, last = self.per.locked(min_inliers)
            if ok and (a.target is None or self.per.lock_target() == a.target):
                held += 0.1
                if held >= LOCK_HOLD_S:
                    break
            else:
                held = 0.0
            time.sleep(0.1)
        print(f"  button node   : {last}")
        if held < LOCK_HOLD_S and require_lock:
            problems.append(f"button node not locked on {a.target or 'any target'} for {LOCK_HOLD_S}s")
        if self.per.get("claw_px", FRESH_S) is None:
            problems.append("no claw_pixel")

        fa = self.per.age("force")
        f = self.per.force_at_rest()
        pressed = self.per.get("pressed", FORCE_STALE_S)
        print(f"  press detector: age {fa if fa is None else round(fa, 2)} s  |dF| {f:.2f} N  pressed={pressed}")
        if fa is None or fa > FORCE_STALE_S:
            problems.append("press detector not publishing (detect_button_press_force.py --publish)")
        elif require_free and f > FORCE_FREE_N:
            problems.append(f"tool force {f:.2f} N > {FORCE_FREE_N} -- arm touching something / bad baseline")
        elif require_free and pressed:
            problems.append("press detector reports PRESSED")

        try:
            self.R_bc = self.per.cam_rotation_in_base()
            print(f"  tf {a.arm_frame} <- {a.camera_frame}: ok (camera +z in base = {np.round(self.R_bc[:, 2], 3)})")
        except Abort as e:
            problems.append(str(e))

        self.start_joints = self.arm.joints()
        self.seed_posture = self.start_joints
        (self.log_dir / "start_joints.json").write_text(json.dumps({
            "joints": self.start_joints.tolist(), "ee_pos": list(map(float, st["ee_pos"])), "t": time.time()}))
        print(f"  start joints saved -> {self.log_dir / 'start_joints.json'}")
        self.log({"stage": 0, "arm_state": name, "gripper": grip, "speed": str(speed), "status": last,
                  "force": f, "problems": problems})
        if problems:
            for pr in problems:
                print(f"  PREFLIGHT FAIL: {pr}")
            raise Abort("preflight failed -- nothing commanded")
        print("  preflight OK")

    # ---- geometry shared by stages 1/2 --------------------------------------------------------
    def hold_px(self):
        claw = self.per.get("claw_px", FRESH_S)
        if claw is None:
            raise Abort("no claw_pixel")
        return claw + np.array([0.0, -HOLD_DY_PX])

    def measure_panel(self, px=None, label="fingertip ray"):
        """Plane + the ray through `px` (default: the claw pixel). Sets ray_*, s_panel, z_panel."""
        n, d, z_med, npts = self.per.panel_plane()
        if px is None:
            px = self.per.get("claw_px", FRESH_S)
            if px is None:
                raise Abort("no claw_pixel")
        self.ray_cam = self.per.ray(px)
        self.ray_base = self.R_bc @ self.ray_cam
        denom = float(n @ self.ray_cam)
        if abs(denom) < 0.3:
            raise Abort(f"{label} nearly parallel to the panel (n.r={denom:.2f})")
        self.s_panel = d / denom
        self.z_panel = z_med
        print(f"  panel plane   : normal(cam) {np.round(n, 3)}  median depth {z_med*100:.1f} cm  ({npts} px)")
        print(f"  {label:14s}: cam {np.round(self.ray_cam, 3)}  base {np.round(self.ray_base, 3)}"
              f"  camera->panel along ray {self.s_panel*100:.1f} cm")
        self.log({"panel_normal_cam": n.tolist(), "panel_d": d, "z_panel": z_med, "n_px": npts,
                  "ray_px": np.asarray(px).tolist(), "ray_cam": self.ray_cam.tolist(),
                  "ray_base": self.ray_base.tolist(), "s_panel": self.s_panel})

    def pixel_error(self, after=None, target=None):
        """button_px - target (default target: the claw pixel)."""
        b = self.per.median_after("button_px", after if after is not None else time.monotonic())
        c = self.per.get("claw_px", FRESH_S) if target is None else target
        if b is None or c is None:
            return None
        return b - c

    def lateral_correction(self, e_px, z, cap):
        """Camera-frame in-plane translation that moves the button pixel onto the claw pixel.

        Translating the camera +x makes a static point's u decrease, so the correction
        is +e (button right of claw -> move right). Scaled by the button's depth z.
        """
        fx, fy, _, _ = self.per.intrinsics()
        d_cam = np.array([e_px[0] * z / fx, e_px[1] * z / fy, 0.0])
        mag = float(np.linalg.norm(d_cam))
        if mag > cap:
            d_cam *= cap / mag
        return d_cam, self.R_bc @ d_cam

    # ---- far phase (before stage 1 when the panel is far) ------------------------------------
    def far_approach(self):
        """Hold the button HOLD_DY_PX above the claw pixel and close in along that ray until
        the panel is CLOSE_STANDOFF_M away. No contact is possible here (the fingertip is well
        short of the panel), so any force jump is an abort, not a contact."""
        print(f"\n== far phase: hold the button {HOLD_DY_PX:.0f} px above the claw, close to {CLOSE_STANDOFF_M*100:.0f} cm ==")
        self.far_travelled = 0.0
        since = 0.0
        f_prev = None
        # servo onto the hold pixel first
        moved_at = None
        for it in range(1, SERVO_MAX_ITERS + 1):
            e = self.pixel_error(after=moved_at, target=self.hold_px())
            if e is None:
                raise Abort("button node not locking in the far phase -- HOLDING HERE")
            err = float(np.linalg.norm(e))
            print(f"  hold-servo {it}: button - hold = ({e[0]:+.1f}, {e[1]:+.1f}) px  |e| {err:.1f}")
            if err < PX_TOL:
                break
            d_cam, d_base = self.lateral_correction(e, self.z_panel, SERVO_MAX_STEP_M)
            self.step(d_base, f"hold-servo {it}")
            if not self.args.execute:
                break
            moved_at = time.monotonic() + 0.15
        else:
            raise Abort("far phase: not aligned on the hold pixel -- HOLDING HERE")
        self.measure_panel(self.hold_px(), "hold ray")
        self.far_ray_base = self.ray_base.copy()
        if self.args.execute:
            time.sleep(FORCE_SETTLE_S)
            f_prev = self.force_now()
        step_i = 0
        while self.s_panel > CLOSE_STANDOFF_M:
            step = min(FAR_STEP_M, self.s_panel - CLOSE_STANDOFF_M)
            step_i += 1
            self.step(self.far_ray_base * step, f"far {step_i} (+{step*100:.1f})")
            self.far_travelled += step
            since += step
            self.s_panel -= step   # nominal; re-measured below
            if not self.args.execute:
                if step_i >= 2:
                    print("  (dry run: far steps continue until the panel is at the close standoff)")
                    return
                continue
            time.sleep(FORCE_SETTLE_S)
            f = self.force_now()
            jump = f - f_prev
            f_prev = f
            print(f"  far travelled {self.far_travelled*100:5.1f} cm  |dF| {f:5.2f} N ({jump:+.2f})  nominal panel {self.s_panel*100:.1f} cm")
            # No contact is possible out here, and 2 cm steps move joints several degrees,
            # which shifts the wrench estimate by up to ~3 N. Only a gross collision counts.
            if f > FORCE_ABORT_N:
                raise Abort(f"|dF| {f:.1f} N in the far phase -- collision? HOLDING HERE")
            if since >= FAR_RESERVO_EVERY_M:
                since = 0.0
                ok, st = self.per.locked(MIN_INLIERS_TRACK)
                if not ok:
                    # Between reference views the lock dips; the hold ray is cached and the
                    # panel distance keeps counting down nominally, so just skip this check.
                    print(f"  (weak/no lock in transit: {st} -- continuing on the cached ray)")
                    continue
                e = self.pixel_error(after=time.monotonic() - 0.2, target=self.hold_px())
                if e is not None and np.linalg.norm(e) > PX_TOL:
                    d_cam, d_base = self.lateral_correction(e, self.z_panel, RESERVO_MAX_STEP_M)
                    print(f"  hold re-servo: e=({e[0]:+.1f},{e[1]:+.1f}) px -> base {np.round(d_base*100, 2)} cm")
                    self.step(d_base, "hold re-servo")
                    time.sleep(FORCE_SETTLE_S)
                    f_prev = self.force_now()
                self.measure_panel(self.hold_px(), "hold ray")   # real distance, not nominal
                self.far_ray_base = self.ray_base.copy()
        print(f"  far phase done: panel {self.s_panel*100:.1f} cm along the hold ray after {self.far_travelled*100:.1f} cm")
        self.reanchor_seed("after the far phase")
        self.rebaseline_force("after the far phase")

    # ---- stage 1 ------------------------------------------------------------------------------
    def servo(self):
        print("\n== stage 1: lateral servo at standoff ==")
        self.measure_panel()
        moved_at = None
        for it in range(1, SERVO_MAX_ITERS + 1):
            e = self.pixel_error(after=moved_at)
            if e is None:
                raise Abort("button node stopped locking during the servo -- HOLDING HERE")
            err = float(np.linalg.norm(e))
            print(f"  iter {it}: button - claw = ({e[0]:+.1f}, {e[1]:+.1f}) px  |e| {err:.1f}")
            self.log({"stage": 1, "iter": it, "e_px": e.tolist()})
            if err < PX_TOL:
                print(f"  aligned (|e| < {PX_TOL} px)")
                break
            d_cam, d_base = self.lateral_correction(e, self.z_panel, SERVO_MAX_STEP_M)
            print(f"          correction cam {np.round(d_cam*100, 2)} cm -> base {np.round(d_base*100, 2)} cm")
            self.step(d_base, f"servo {it}")
            if not self.args.execute:
                print("  (dry run: cannot observe the effect of the correction; stopping the loop here)")
                break
            moved_at = time.monotonic() + 0.15   # only trust pixels from frames after the arm settled
        else:
            raise Abort(f"not aligned after {SERVO_MAX_ITERS} iterations -- HOLDING HERE")
        # The ray only depends on the claw pixel, but re-measure the plane now that we moved.
        if self.args.execute:
            self.measure_panel()

    # ---- stage 2 ------------------------------------------------------------------------------
    def approach(self) -> str:
        print("\n== stage 2: approach along the fingertip ray ==")
        a = self.args
        if a.tip_dist is None:
            if a.execute:
                raise Abort("--tip-dist (camera lens -> LEFT fingertip, metres, ruler) is required "
                            "to execute the approach. Over-estimate rather than under-estimate.")
            print("  (dry run without --tip-dist: cap shown for tip_dist = 0)")
            tip = 0.0
        else:
            tip = a.tip_dist
        L_max = self.s_panel - tip + PLANE_OVERSHOOT_M
        if a.cap_override is not None:
            L_max = min(L_max, a.cap_override)
        print(f"  travel cap L_max = s_panel {self.s_panel*100:.1f} - tip {tip*100:.1f} + overshoot {PLANE_OVERSHOOT_M*100:.1f}"
              f"{f' (override {a.cap_override*100:.1f})' if a.cap_override is not None else ''} = {L_max*100:.1f} cm")
        if L_max <= 0:
            raise Abort("travel cap <= 0: fingertip is already at/through the plane per these numbers")
        self.log({"stage": 2, "L_max": L_max, "tip_dist": tip, "cap_override": a.cap_override})

        self.travelled = 0.0
        since_servo = 0.0
        step_i = 0
        f_ref = f_prev = None
        f_hist = deque(maxlen=CONTACT_WINDOW + 1)
        if a.execute:
            self.rebaseline_force("before the approach")
            f_ref = f_prev = self.force_now()
            print(f"  force at rest before the approach: {f_ref:.2f} N (contact needs +{CONTACT_N} N total "
                  f"and +{JUMP_N} N within one step)")
            self.log({"stage": 2, "f_ref": f_ref})
        # Distance the fingertip must travel before it can possibly touch the panel.
        l_contact = self.s_panel - tip
        armed = False
        print(f"  contact arms at {max(0.0, l_contact - CONTACT_ARM_MARGIN_M)*100:.1f} cm travel "
              f"(panel is {l_contact*100:.1f} cm from the fingertip); below that a force jump is "
              f"posture drift, not the button (abort only above {COARSE_ABORT_N} N)")
        while True:
            remaining = L_max - self.travelled
            if remaining <= 1e-4:
                self.log({"stage": 2, "stop": "cap_reached", "travelled": self.travelled})
                print(f"  travel cap reached at {self.travelled*100:.1f} cm with no contact")
                return "cap_reached"
            if a.execute and not armed and self.travelled >= l_contact - CONTACT_ARM_MARGIN_M:
                armed = True
                # Re-zero the detector at the edge of the contact zone: every coarse step so far
                # has shifted the wrench bias, so the pre-approach reference is stale by now.
                self.rebaseline_force("entering the contact zone")
                f_ref = f_prev = self.force_now()
                f_hist.clear()
                f_hist.append(f_ref)
                print(f"  contact detection ARMED at {self.travelled*100:.1f} cm; rest force {f_ref:.2f} N")
                self.log({"stage": 2, "armed_at": self.travelled, "f_ref": f_ref})
            step = a.fine_step if (armed or remaining <= FINE_ZONE_M) else APPROACH_STEP_M
            step = min(step, remaining)
            step_i += 1
            self.step(self.ray_base * step, f"approach {step_i} (+{step*100:.1f})")
            self.travelled += step
            since_servo += step
            if not a.execute:
                if step_i >= 3:
                    print(f"  (dry run: {step_i} steps planned; the real loop continues to the cap or contact)")
                    return "dry_run"
                continue
            time.sleep(FORCE_SETTLE_S)
            f = self.force_now()
            rise, jump = f - f_ref, f - f_prev
            print(f"  travelled {self.travelled*100:5.1f} cm  |dF| at rest {f:5.2f} N"
                  f"  (+{rise:.2f} since start, {jump:+.2f} this step)")
            self.log({"stage": 2, "travelled": self.travelled, "force": f, "rise": rise, "jump": jump})
            if f > FORCE_ABORT_N:
                raise Abort(f"|dF| {f:.1f} N > {FORCE_ABORT_N} -- that is not a button. HOLDING HERE")
            if not armed and f > COARSE_ABORT_N:
                raise Abort(f"|dF| {f:.1f} N > {COARSE_ABORT_N} at {self.travelled*100:.1f} cm, "
                            f"{l_contact*100:.1f} cm from the panel -- hit something unexpected. HOLDING HERE")
            if not armed:
                # Too far out for contact: track the drifting bias instead of reading it as a
                # press. Still falls through to the lateral re-servo below.
                f_ref = f_prev = f
            f_hist.append(f)
            win_rise = f - f_hist[0] if len(f_hist) > 1 else 0.0
            if armed:
                print(f"      (rise over the last {len(f_hist)-1} step(s): {win_rise:+.2f} N)")
            if not armed:
                pass
            elif (win_rise > CONTACT_N and len(f_hist) > 1) or jump > JUMP_N:
                # Candidate only. Push one more fine step: a real contact keeps loading up,
                # a drifting bias does not. See CONTACT_N for the measurements behind this.
                print(f"  candidate contact at {self.travelled*100:.1f} cm ({f:.2f} N, +{jump:.2f} N"
                      f" in one step) -- confirming with one more {a.fine_step*1000:.0f} mm step")
                self.log({"stage": 2, "candidate": self.travelled, "force": f, "jump": jump})
                stp = min(a.fine_step, L_max - self.travelled)
                if stp <= 1e-4:
                    print("  travel cap reached before the candidate could be confirmed")
                    self.log({"stage": 2, "stop": "cap_before_confirm", "travelled": self.travelled})
                    return "cap_reached"
                self.step(self.ray_base * stp, f"confirm (+{stp*100:.2f})")
                self.travelled += stp
                since_servo += stp
                time.sleep(FORCE_SETTLE_S)
                f2 = self.force_now()
                print(f"    confirm: |dF| {f2:.2f} N ({f2 - f:+.2f} N since the candidate)")
                self.log({"stage": 2, "confirm": self.travelled, "force": f2, "delta": f2 - f})
                if f2 > FORCE_ABORT_N:
                    raise Abort(f"|dF| {f2:.1f} N while confirming -- HOLDING HERE")
                if f2 - f >= CONFIRM_RISE_N:
                    print(f"  CONTACT CONFIRMED at {self.travelled*100:.1f} cm "
                          f"({f2:.2f} N, rose {jump:+.2f} then {f2 - f:+.2f})")
                    self.log({"stage": 2, "stop": "contact", "travelled": self.travelled, "force": f2})
                    return "contact"
                print("  NOT confirmed -- the force did not keep rising, so that was posture "
                      "drift, not the button. Re-baselining and continuing.")
                self.rebaseline_force("rejected a phantom contact")
                f_ref = f_prev = self.force_now()
                f_hist.clear()          # the window must not straddle a re-baseline
                f_hist.append(f_ref)
            else:
                if rise > CONTACT_N:
                    print(f"  (level +{rise:.2f} N but no step jump -- treating as drift, continuing)")
                f_prev = f
            # Re-check lateral alignment while the node can still see the panel.
            if since_servo >= RESERVO_EVERY_M:
                since_servo = 0.0
                ok, _ = self.per.locked(MIN_INLIERS_TRACK)
                e = self.pixel_error(after=time.monotonic() - 0.2) if ok else None
                if e is not None and np.linalg.norm(e) > PX_TOL:
                    z_now = max(0.05, self.z_panel - self.travelled * self.ray_cam[2])
                    d_cam, d_base = self.lateral_correction(e, z_now, RESERVO_MAX_STEP_M)
                    print(f"  re-servo: e=({e[0]:+.1f},{e[1]:+.1f}) px -> base {np.round(d_base*100, 2)} cm")
                    self.step(d_base, "re-servo")
                    time.sleep(FORCE_SETTLE_S)
                    f_prev = self.force_now()   # a lateral move shifts the bias too; don't count it as a jump
                elif e is None:
                    print("  (node abstaining -- continuing on the cached ray)")

    # ---- stage 3 ------------------------------------------------------------------------------
    def press(self, n_presses: int):
        print(f"\n== stage 3: press x{n_presses} ==")
        for i in range(1, n_presses + 1):
            pt = self.args.press_travel
            self.step(self.ray_base * pt, f"press {i} (+{pt*100:.1f})")
            self.travelled += pt
            if self.args.execute:
                time.sleep(PRESS_HOLD_S)
                f = self.force_now()
                print(f"  press {i}: |dF| {f:.2f} N")
                self.log({"stage": 3, "press": i, "force": f})
                if f > FORCE_ABORT_N:
                    raise Abort(f"|dF| {f:.1f} N during press -- HOLDING HERE")
            self.move_along(-self.ray_base, PRESS_RETRACT_M, f"press {i} retract")
            if i < n_presses:
                if self.args.execute:
                    time.sleep(0.5)
                # Back to contact depth: re-advance what we retracted, minus the press travel.
                self.move_along(self.ray_base, PRESS_RETRACT_M - pt, f"press {i+1} re-approach")

    # ---- stage 4 ------------------------------------------------------------------------------
    def retract(self):
        print("\n== stage 4: retract to standoff ==")
        self.move_along(-self.ray_base, self.travelled, "retract")
        if self.far_ray_base is not None and self.far_travelled > 1e-4:
            print(f"  undoing the far phase ({self.far_travelled*100:.1f} cm along the hold ray)")
            self.move_along(-self.far_ray_base, self.far_travelled, "far retract", far=True)
        if self.args.home:
            q0 = self.start_joints
            q = self.arm.joints()
            jump = float(np.degrees(np.max(np.abs((q0 - q + np.pi) % (2 * np.pi) - np.pi))))
            print(f"  home: joint move back to start, max joint delta {jump:.1f} deg")
            if jump > 20.0:
                raise Abort(f"home move is {jump:.0f} deg on one joint -- use goto_preset.py deliberately instead")
            if self.args.execute:
                self.arm.ai.execute_command(JointCommand(pos=q0.tolist()))
                derr = _wait_converged(self.arm.ai, q0, timeout_s=10.0)
                print(f"  home: converged to {derr:.2f} deg")

    # ---- driver -------------------------------------------------------------------------------
    def run(self):
        a = self.args
        print(f"{'EXECUTE' if a.execute else 'DRY RUN'}  stages 0..{a.stage}  target={a.target or '(node default)'}"
              f"  presses={a.presses}  log={self.log_dir}")
        try:
            self.preflight()
            if a.stage >= 1:
                self.measure_panel(self.hold_px(), "hold ray")
                if self.s_panel > CLOSE_STANDOFF_M + 0.03:
                    self.far_approach()
                self.servo()
            if a.stage >= 2:
                outcome = self.approach()
                if a.stage >= 3 and outcome in ("contact", "dry_run"):
                    self.press(a.presses)
                elif a.stage >= 3:
                    print(f"  no contact ({outcome}) -- skipping the press")
                if a.stage >= 4 and outcome != "dry_run":
                    self.retract()
                elif a.stage >= 4:
                    print("\n== stage 4: (dry run) retract would reverse the approach travel ==")
        except KeyboardInterrupt:
            print("\nCtrl-C: no further steps sent. The in-flight step (if any) completes; arm HOLDS.")
            self._way_back()
            return 130
        except Abort as e:
            print(f"\nABORT: {e}")
            self._way_back()
            return 2
        print("\ndone.")
        return 0

    def _way_back(self):
        if self.start_joints is not None:
            print(f"  start joints are in {self.log_dir / 'start_joints.json'}; travelled along ray: "
                  f"{self.travelled*100:.1f} cm (ray base {np.round(self.ray_base, 3) if self.ray_base is not None else '?'})")
            print("  To retreat: re-run with --execute --stage 4 --resume-travel "
                  f"{self.travelled:.4f} (retract only), or goto_preset.py to a saved pose.")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--execute", action="store_true", help="actually move the arm (default: dry run)")
    ap.add_argument("--stage", type=int, default=4, choices=range(0, 5),
                    help="stop after this stage: 0 preflight, 1 servo, 2 approach, 3 press, 4 retract")
    ap.add_argument("--presses", type=int, default=1)
    ap.add_argument("--tip-dist", type=float, default=None,
                    help="camera lens -> LEFT fingertip distance in metres (ruler). Required to execute stage 2.")
    ap.add_argument("--cap-override", type=float, default=None,
                    help="limit the approach travel to this many metres (first tests: 0.05)")
    ap.add_argument("--press-travel", type=float, default=PRESS_TRAVEL_M,
                    help=f"extra travel (m) past first contact for the press itself (default {PRESS_TRAVEL_M})")
    ap.add_argument("--fine-step", type=float, default=FINE_STEP_M,
                    help=f"step size (m) for the last {FINE_ZONE_M*100:.0f} cm before the cap (default {FINE_STEP_M})")
    ap.add_argument("--target", default=None, help="require the node to be locked on this button name")
    ap.add_argument("--home", action="store_true", help="after retracting, joint-move back to the start joints")
    ap.add_argument("--goto-start", action="store_true",
                    help="only: joint-move back to the start joints saved by the previous run (<= 20 deg per joint)")
    ap.add_argument("--resume-far-travel", type=float, default=None,
                    help="with --stage 4 only: also undo this many metres of far-phase travel (hold ray)")
    ap.add_argument("--resume-travel", type=float, default=None,
                    help="with --stage 4 only: retract this many metres along the ray without re-approaching")
    ap.add_argument("--jog", type=float, default=None,
                    help="MEASUREMENT mode: advance this many metres along the fingertip ray in "
                         "fine steps and stop. No servo, no contact detection, no press. Used to "
                         "measure --tip-dist against a human-confirmed touch: at real contact the "
                         "fingertip is ON the panel, so tip_dist = (camera->panel along the ray at "
                         "the start) - (travel). Reports both after every step.")
    ap.add_argument("--log-dir", default=str(Path.home() / "press_logs"))
    ap.add_argument("--ns", default="/button_detector")
    ap.add_argument("--press-ns", default="/press_detector")
    ap.add_argument("--arm-frame", default="arm_base_link")
    ap.add_argument("--camera-frame", default="camera_color_optical_frame")
    args = ap.parse_args()

    rclpy.init()
    try:
        run = Run(args)
        if args.goto_start:
            saved = json.loads((run.log_dir / "start_joints.json").read_text())
            q0 = np.asarray(saved["joints"], dtype=float)
            run.preflight(require_lock=False, require_free=False)   # saves the CURRENT joints first
            q = run.arm.joints()
            jump = float(np.degrees(np.max(np.abs((q0 - q + np.pi) % (2 * np.pi) - np.pi))))
            print(f"\n== goto-start: max joint delta {jump:.1f} deg (saved {time.ctime(saved['t'])}) ==")
            if jump > 20.0:
                print("refusing: > 20 deg on one joint -- use goto_preset.py deliberately"); return 2
            if not args.execute:
                print("dry run -- add --execute to move"); return 0
            run.arm.ai.execute_command(JointCommand(pos=q0.tolist()))
            derr = _wait_converged(run.arm.ai, q0, timeout_s=15.0)
            print(f"converged to {derr:.2f} deg"); return 0 if derr < CONVERGE_TOL_DEG else 2
        if args.jog is not None:
            # Jog deliberately accepts a WEAKER lock than a press run. Its geometry is the
            # CLAW pixel (a fixed constant) plus the panel quad -- it never uses the button
            # position -- so a lock too weak to trust for "which dome is timer_clock" is
            # still fine for moving along the ray. The press path keeps the full MIN_INLIERS
            # gate, which exists because 6-8 inlier locks have picked the wrong dome.
            run.preflight(require_lock=True, require_free=True, min_inliers=MIN_INLIERS_TRACK)
            run.R_bc = run.per.cam_rotation_in_base()
            run.measure_panel()
            s0 = run.s_panel
            print(f"\n== JOG {args.jog*100:.1f} cm along the fingertip ray (measurement only) ==")
            print(f"  camera -> panel along the ray right now: {s0*100:.2f} cm")
            print(f"  at a human-confirmed touch:  tip_dist = {s0*100:.2f} cm - travel")
            print(f"  (the assumed tip_dist {(args.tip_dist or 0)*100:.1f} cm predicts a touch at "
                  f"{(s0 - (args.tip_dist or 0))*100:.2f} cm of travel)")
            if not args.execute:
                print("  dry run -- add --execute to move")
                return 0
            run.rebaseline_force("before the jog")
            remaining, k = args.jog, 0
            try:
                while remaining > 1e-4:
                    stp = min(args.fine_step, remaining)
                    k += 1
                    run.step(run.ray_base * stp, f"jog {k} (+{stp*100:.2f})")
                    run.travelled += stp
                    remaining -= stp
                    time.sleep(FORCE_SETTLE_S)
                    fo = run.force_now()
                    print(f"    travel {run.travelled*100:5.2f} cm | |dF| {fo:5.2f} N | "
                          f"implied tip_dist if touching NOW = {(s0 - run.travelled)*100:5.2f} cm")
                    run.log({"stage": "jog", "travelled": run.travelled, "force": fo,
                             "implied_tip_dist": s0 - run.travelled})
                    if fo > FORCE_ABORT_N:
                        raise Abort(f"|dF| {fo:.1f} N during the jog -- HOLDING HERE")
            except (Abort, KeyboardInterrupt) as e:
                print(f"\nSTOPPED: {e}")
            print(f"\n  jog done: travelled {run.travelled*100:.2f} cm; "
                  f"if the fingertip is touching now, --tip-dist {(s0 - run.travelled):.4f}")
            print(f"  to back out: --execute --stage 4 --resume-travel {run.travelled:.4f}")
            return 0
        if args.resume_travel is not None or args.resume_far_travel is not None:
            # Retreat-only mode after an abort: preflight (minus the lock requirement being
            # meaningful), recompute the ray(s) from the claw pixel + tf, then retract.
            # Retreating is exactly what you do while the tool is still loaded, so
            # neither the lock nor the free-tool check applies here.
            run.preflight(require_lock=False, require_free=False)
            run.R_bc = run.per.cam_rotation_in_base()
            run.ray_cam = run.per.ray(run.per.get("claw_px"))
            run.ray_base = run.R_bc @ run.ray_cam
            run.travelled = args.resume_travel or 0.0
            if args.resume_far_travel:
                run.far_ray_base = run.R_bc @ run.per.ray(run.hold_px())
                run.far_travelled = args.resume_far_travel
            try:
                run.retract()
            except Abort as e:
                print(f"\nABORT: {e}")
                return 2
            return 0
        return run.run()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
