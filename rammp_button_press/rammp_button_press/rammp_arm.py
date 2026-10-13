"""`RammpArm`: the button press's arm on RAMMP's kinova_gen3_node instead of the Cornell arm RPC.

Same methods ``press_button`` used on the old ``Arm`` (state, joints, ee_pose, precheck_poses,
smooth_poses), mapped onto the driver (kinova-gen3-ros2 docs/interface.md):

    joints / state["position"]  <- /joint_states (best effort; joint_1..7, wrapped to +-pi like Kortex)
    ee_pose / state["ee_pos"]   <- /ee_state + TOOL_Z along the flange z  (see TOOL_Z)
    state["gripper_pos"]        <- /gripper_state.position (0 open .. 1 closed)
    status()                    <- /control_status (estopped, arbitration, owner) + /diagnostics
    smooth_poses(path)          -> ONE execute_joint_trajectory goal through the joint solutions
                                   kinematics.precheck_poses computed for that path
    (Kortex speed preset)       -> no equivalent: speed is set here by the trajectory timing

The old stack sent a Cartesian trajectory and Kortex ran its own IK, so the sim check was a
reachability proxy. Here the arm follows exactly the joint path that passed the gates. The
goals are dense (DT) with velocities, so the driver interpolates cubically, timed by a
minimum-jerk profile over the whole leg; ``path_tolerance`` makes the driver abort if any joint
lags its reference by more than PATH_TOL_RAD. Ctrl-C while a goal runs cancels it (the arm
stops) and re-raises. Trajectory code adapted from microwave/sheppy_arm.py (microwave-wip),
which ran attended on this arm 2026-10-07.

Never commands the gripper. Never calls go_to_ee_pose (cuRobo-planned: not a straight line).
"""
from __future__ import annotations

import threading
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.msg import JointTolerance
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from rammp_arm_interfaces.action import ExecuteJointTrajectory
from rammp_arm_interfaces.msg import EeState, GripperState, PoseSetpoint
from rammp_arm_interfaces.srv import CloseStream, OpenStream
from rammp_curobo_interfaces.action import PlanToPose
from rammp_common_interfaces.msg import ControlStatus
from rammp_common_interfaces.srv import AcquireControl, ReleaseControl

from rammp_button_press import Abort
from rammp_button_press.geometry import max_joint_delta_deg
from rammp_button_press.kinematics import (JOINTS, MAX_IK_ERR_M, MAX_IK_ROT_ERR_DEG, MAX_JOINT_STEP_DEG, MAX_REACH_M,
                                           Z_RANGE_M, Kinematics)

# /ee_state is the flange (end_effector_link): checked 2026-10-07, tf2 base_link -> end_effector_link
# == /ee_state, and sim tool_frame FK at the real joints == /ee_state + 0.120 m along the flange z
# to 0.1 mm. The button press's PREPRESS_EE_OFFSET_M was measured in Kinova's tool_frame (= the
# Kortex tool pose = flange + 0.120 m), so ee_pose() reports that. preflight re-checks it live
# (TOOL_FRAME_CHECK_M): if the driver's /ee_state ever changes frame, the run refuses.
TOOL_Z = 0.120
TOOL_FRAME_CHECK_M = 0.005
FRESH_STATE_S = 0.5           # /joint_states and /ee_state run at ~100 Hz; older than this = driver down
CONTINUOUS = [0, 2, 4, 6]     # J1/J3/J5/J7 spin freely
DT = 0.02                     # trajectory sample spacing (cuRobo's interpolation_dt)
# Speed: the old stack ran with Kortex's low/medium preset. These are peaks of the min-jerk profile,
# the values microwave/sheppy_arm.py ran attended on this arm before "a little faster" (2026-10-07).
CART_SPEED = 0.06             # m/s
JOINT_SPEED = 0.35            # rad/s, on any joint, whatever the Cartesian length says
ROT_EQUIV_M = 0.15            # a 1 rad wrist turn counts as 15 cm of travel when timing a leg
MIN_DURATION_S = 1.0
PATH_TOL_RAD = 0.15           # ~8.6 deg tracking guard per joint (driver-side abort), = sheppy_arm.py
TRACK_ABORT_M = 0.01          # final tool position this far from the target -> hold (= old arm.py)
START_DRIFT_DEG = 0.5         # the arm must still be where a leg was planned from when it is sent
SENDER_ID = "rammp_button_press"
ERRORS = {0: "SUCCESSFUL", -1: "INVALID_GOAL", -4: "PATH_TOLERANCE_VIOLATED",
          -5: "GOAL_TOLERANCE_VIOLATED", -6: "PREEMPTED", -8: "NOT_AUTHORIZED", -9: "HALTED"}
DIAG_LEVELS = {0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"}
# ---- long moves planned by RAMMP's cuRobo (curobo_move): staging, back to start, bail. The
# planner's own path is gated here point by point (reach, z band, joint limits, joint step,
# final pose) and retimed to CART_SPEED / JOINT_SPEED, then sent as-is through
# execute_joint_trajectory -- so what moves is exactly what passed. (go_to_ee_pose would
# re-plan inside the driver, unseen.) Its world (gen3_real.yaml: pedestal + table) does NOT
# contain the microwave, counter, wheelchair or user.
CUROBO_ACTION = "/rammp_curobo/plan_to_pose"
CUROBO_TIMEOUT_S = 30.0
# ---- impedance press (press_impedance): the press stroke only, streamed to the driver's
# ee_pose_impedance controller (JointImpedanceMode, in-loop IK; driver defaults Kq 80/30 N.m/rad).
# RAMMP marks that mode's closed-loop behaviour unvalidated (kinova-gen3-driver #6), its gravity
# model misses the 0.44 kg wrist camera mount and torque holds "droop and drift" (#5, #40): so the
# session first holds the spot and refuses to press if the arm sags. Closing a session latches a
# hold at the measured q (no slew back to the last setpoint).
IMP_CONTROLLER = "ee_pose_impedance"
IMP_TIMEOUT_S = 0.1           # driver stops the session if setpoints stop for this long
IMP_RATE_HZ = 100.0           # setpoint rate (10x the timeout)
IMP_SETTLE_S = 0.6            # hold the spot after opening: the spring ramps in over 0.5 s
IMP_STROKE_S = 1.0            # each way, min-jerk
IMP_END_HOLD_S = 0.3          # back at the spot, before closing
IMP_HOLD_ABORT_M = 0.005      # tool more than this off the held spot (sag / drift) -> close and hold
IMP_LATERAL_ABORT_M = 0.01    # sideways off the press line during the stroke -> close and hold
IMP_OVERSHOOT_ABORT_M = 0.005  # deeper than the commanded depth by this -> close and hold
IMP_START_CHECK_M = 0.002     # the arm must be at the pre-press spot before a session opens


def wrap(q):
    q = np.asarray(q, dtype=float).copy()
    q[CONTINUOUS] = (q[CONTINUOUS] + np.pi) % (2 * np.pi) - np.pi
    return q


def unwrap_from(q_start, knots):
    """Knots made continuous from `q_start`: each free joint steps the short way from the previous knot."""
    out, prev = [], np.asarray(q_start, dtype=float)
    for k in knots:
        d = np.asarray(k, dtype=float) - prev
        d[CONTINUOUS] = (d[CONTINUOUS] + np.pi) % (2 * np.pi) - np.pi
        prev = prev + d
        out.append(prev.copy())
    return out


def impedance_profile(depth, settle_s=IMP_SETTLE_S, stroke_s=IMP_STROKE_S, hold_s=0.0,
                      end_s=IMP_END_HOLD_S, rate_hz=IMP_RATE_HZ):
    """(t, depth_at_t, phase) for an impedance press: hold, min-jerk in to `depth`, hold, back out,
    hold. Phases: 'settle', 'in', 'bottom', 'out', 'end'. depth 0 = hold test only."""
    out = []

    def mj(u):
        return 10 * u**3 - 15 * u**4 + 6 * u**5

    t0 = 0.0
    phases = [("settle", settle_s, lambda u: 0.0)]
    if depth > 0:
        phases += [("in", stroke_s, lambda u: depth * mj(u)), ("bottom", hold_s, lambda u: depth),
                   ("out", stroke_s, lambda u: depth * (1.0 - mj(u)))]
    phases.append(("end", end_s, lambda u: 0.0))
    dt = 1.0 / rate_hz
    for name, dur, f in phases:
        n = int(round(dur * rate_hz))
        for k in range(n):
            out.append((t0 + k * dt, float(f((k + 1) / n)), name))
        t0 += n * dt
    return out


def min_jerk_samples(path, s_knots, duration):
    """Positions + velocities every DT along a piecewise-linear joint path (knots at path
    parameter `s_knots`, 0..L), timed by a minimum-jerk s(t) over `duration`."""
    path, length = np.asarray(path), float(s_knots[-1])
    n = max(2, int(np.ceil(duration / DT)) + 1)
    t = np.linspace(0.0, duration, n)
    u = t / duration
    s = length * (10 * u**3 - 15 * u**4 + 6 * u**5)
    q = np.stack([np.interp(s, s_knots, path[:, j]) for j in range(path.shape[1])], axis=1)
    qd = np.gradient(q, t, axis=0)
    qd[0] = qd[-1] = 0.0
    return t, q, qd


def leg_timing(q0, knots, poses, p0, R0):
    """(joint path from q0, path parameter at each knot, duration) for a leg of tool `poses`
    starting at tool pose (p0, R0). Timed in Cartesian metres at CART_SPEED (a wrist turn counted
    via ROT_EQUIV_M), never faster than JOINT_SPEED on any joint. min-jerk peak = 1.875 x average."""
    path = [np.asarray(q0, dtype=float)] + unwrap_from(q0, knots)
    seg, prev_p, prev_R = [], np.asarray(p0, dtype=float), np.asarray(R0, dtype=float)
    for pt, Rt in poses:
        turn = Rotation.from_matrix(np.asarray(Rt) @ prev_R.T).magnitude()
        seg.append(max(1e-4, float(np.linalg.norm(np.asarray(pt) - prev_p)), turn * ROT_EQUIV_M))
        prev_p, prev_R = np.asarray(pt, dtype=float), np.asarray(Rt, dtype=float)
    s_knots = np.concatenate([[0.0], np.cumsum(seg)])
    joint_travel = float(np.sum([np.max(np.abs(b - a)) for a, b in zip(path[:-1], path[1:])]))
    duration = max(MIN_DURATION_S, 1.875 * s_knots[-1] / CART_SPEED, 1.875 * joint_travel / JOINT_SPEED)
    return np.array(path), s_knots, duration


class RammpArm:
    def __init__(self, execute: bool, acquire_control: bool = False):
        self.execute = execute
        self.acquire = acquire_control
        self.node, self._exec, self._spin = spin_node("rammp_button_press_arm")
        node = self.node
        self.kin = Kinematics()
        self._lock = threading.Lock()
        self._js = self._ee = self._grip = self._status = None
        self._js_names, self._js_idx = None, None
        self._diag: dict[str, tuple[int, str]] = {}
        self._token = bytes(16)
        self._active_gh = None
        self.cancel_requested = False   # set by cancel_motion(); a caller clears it per task
        self.sent_goals = 0             # trajectories the driver accepted (press_server: "did it move")
        node.create_subscription(JointState, "/joint_states", self._on_js, qos_profile_sensor_data)
        node.create_subscription(EeState, "/ee_state", self._on_ee, qos_profile_sensor_data)
        node.create_subscription(GripperState, "/gripper_state", self._on_grip, qos_profile_sensor_data)
        node.create_subscription(ControlStatus, "/control_status", self._on_status, QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        node.create_subscription(DiagnosticArray, "/diagnostics", self._on_diag, 10)
        self._traj = ActionClient(node, ExecuteJointTrajectory, "/execute_joint_trajectory")
        self._acq = node.create_client(AcquireControl, "/acquire_control")
        self._rel = node.create_client(ReleaseControl, "/release_control")
        # Impedance press: the publisher exists from the start, so DDS discovery has settled
        # long before a session opens (the driver doc's "create your publisher BEFORE you open").
        self._sp_pub = node.create_publisher(PoseSetpoint, "/setpoint/pose", qos_profile_sensor_data)
        self._plan = ActionClient(node, PlanToPose, CUROBO_ACTION)
        self._open_stream = node.create_client(OpenStream, "/open_stream")
        self._close_stream = node.create_client(CloseStream, "/close_stream")
        t0 = time.time()
        while (self._js is None or self._ee is None or self._grip is None) and time.time() - t0 < 10.0:
            time.sleep(0.05)
        if self._js is None or self._ee is None or self._grip is None:
            missing = [n for n, v in (("/joint_states", self._js), ("/ee_state", self._ee),
                                      ("/gripper_state", self._grip)) if v is None]
            raise Abort(f"no {', '.join(missing)} in 10 s -- is the sheppy `arm` node up (and the arm powered, "
                        "eno1 linked)? ROS_DOMAIN_ID=0 / RMW_IMPLEMENTATION=rmw_cyclonedds_cpp exported?")

    # -- subscriptions ------------------------------------------------------------------------
    # Callbacks run at ~100 Hz each and share the GIL with dome detection (2026-10-12: with the
    # old per-message SciPy rotation on a MultiThreadedExecutor, a 0.5 s re-detect took 1.5 s).
    # So they only store raw values; state() does the maths when someone asks.
    def _on_js(self, m):
        names = tuple(m.name)
        if names != self._js_names:
            self._js_idx = [names.index(n) for n in JOINTS] if all(n in names for n in JOINTS) else None
            self._js_names = names
        if self._js_idx is not None:
            pos = m.position
            q = [pos[i] for i in self._js_idx]
            with self._lock:
                self._js = (time.monotonic(), q)

    def _on_ee(self, m):
        p, o = m.pose.position, m.pose.orientation
        with self._lock:
            self._ee = (time.monotonic(), (p.x, p.y, p.z, o.x, o.y, o.z, o.w))

    def _on_grip(self, m):
        with self._lock:
            self._grip = (time.monotonic(), float(m.position), bool(m.present))

    def _on_status(self, m):
        with self._lock:
            self._status = m

    def _on_diag(self, m):
        with self._lock:
            for s in m.status:
                self._diag[s.name] = (int.from_bytes(s.level, "little") if isinstance(s.level, bytes) else int(s.level),
                                      s.message)

    def close(self):
        self.release_control()
        self._exec.shutdown(timeout_sec=1.0)
        self._spin.join(timeout=2.0)       # no callback may still be running when the node goes
        self.node.destroy_node()
        self.kin.close()

    # -- state --------------------------------------------------------------------------------
    def _fresh(self, item, what):
        if item is None or time.monotonic() - item[0] > FRESH_STATE_S:
            raise Abort(f"{what} is stale (> {FRESH_STATE_S} s) -- arm driver down?")
        return item

    def state(self):
        """The old ArmInterfaceClient.get_state() dict (position, ee_pos [tool xyz + quat xyzw], gripper_pos)."""
        with self._lock:
            js, ee, grip = self._js, self._ee, self._grip
        js, ee = self._fresh(js, "/joint_states"), self._fresh(ee, "/ee_state")
        flange, quat = np.array(ee[1][:3]), np.array(ee[1][3:])
        tool = flange + Rotation.from_quat(quat).apply([0.0, 0.0, TOOL_Z])
        return {"position": wrap(np.array(js[1], dtype=float)), "ee_pos": np.concatenate([tool, quat]),
                "gripper_pos": grip[1]}

    def joints(self):
        return np.asarray(self.state()["position"], dtype=float)

    def raw_joints(self):
        """Measured joints as the driver reports them (not wrapped): the trajectory start."""
        with self._lock:
            js = self._js
        return np.array(self._fresh(js, "/joint_states")[1], dtype=float)

    def ee_pos(self):
        return np.asarray(self.state()["ee_pos"][:3], dtype=float)

    def ee_pose(self):
        """(xyz, 3x3 rotation) of the Kinova tool frame in the arm base frame."""
        ee = self.state()["ee_pos"]
        return np.asarray(ee[:3], dtype=float), Rotation.from_quat(ee[3:7]).as_matrix()

    def gripper_present(self):
        with self._lock:
            return self._grip is not None and self._grip[2]

    def status(self):
        """(problems, info lines) from /control_status, /diagnostics and the live tool-frame check."""
        with self._lock:
            st, diag = self._status, dict(self._diag)
        problems, info = [], []
        if st is None:
            problems.append("no /control_status (latched) -- arm driver not up?")
        else:
            info.append(f"control_status : estopped={st.estopped} arbitration={'enforced' if st.arbitration_enabled else 'disabled'}"
                        f" owned={st.owned}{' by ' + repr(st.owner_id) if st.owned else ''} rejected={st.rejected_count}")
            if st.estopped:
                problems.append("arm is E-STOPPED (latched) -- clear it first")
            if st.arbitration_enabled and not self.acquire:
                problems.append("arbitration is ENFORCED: goals need a token. Rerun with --acquire-control "
                                "(takes control only if nobody holds the arm)")
            if st.arbitration_enabled and st.owned and st.owner_id != SENDER_ID:
                problems.append(f"the arm is owned by {st.owner_id!r} -- acquiring would SEIZE it and halt their "
                                "motion; refusing")
        if not diag:
            info.append("diagnostics    : none yet (1 Hz)")
        for name, (level, msg) in sorted(diag.items()):
            info.append(f"diagnostics    : {name}: {DIAG_LEVELS.get(level, level)} {msg}")
            if level >= 2 and "Arm" in name:
                problems.append(f"diagnostics {name} {DIAG_LEVELS.get(level, level)}: {msg}")
        # Live tool-frame check: our sim FK at the measured joints vs the driver's pose + TOOL_Z.
        try:
            s = self.state()
            p_fk, q_fk = self.kin.fk(s["position"])
            dp = float(np.linalg.norm(p_fk - s["ee_pos"][:3]))
            dr = float(np.degrees((Rotation.from_quat(q_fk) * Rotation.from_quat(s["ee_pos"][3:7]).inv()).magnitude()))
            info.append(f"tool frame     : sim FK vs /ee_state+{TOOL_Z} m: {dp*1000:.1f} mm, {dr:.2f} deg")
            if dp > TOOL_FRAME_CHECK_M or dr > 1.0:
                problems.append(f"tool frame mismatch: sim tool_frame FK is {dp*1000:.1f} mm / {dr:.1f} deg from "
                                f"/ee_state + {TOOL_Z} m -- /ee_state is not the flange any more? The pre-press "
                                "offset would be wrong; refusing")
        except Abort as e:
            problems.append(str(e))
        return problems, info

    # -- arbitration --------------------------------------------------------------------------
    def take_control(self):
        """Under enforced arbitration, acquire a token -- only if nobody holds the arm (acquire SEIZES)."""
        with self._lock:
            st = self._status
        if st is None or not st.arbitration_enabled:
            return
        if st.owned and st.owner_id != SENDER_ID:
            raise Abort(f"arm owned by {st.owner_id!r}; not seizing it")
        if not self._acq.wait_for_service(timeout_sec=3.0):
            raise Abort("/acquire_control unavailable")
        res = self._call(self._acq, AcquireControl.Request(owner_id=SENDER_ID))
        if res is None or not res.accepted:
            raise Abort(f"/acquire_control refused: {getattr(res, 'message', 'no answer')}")
        self._token = bytes(res.token)
        print(f"  control acquired (generation {res.generation})")

    def release_control(self):
        if self._token == bytes(16):
            return
        res = self._call(self._rel, ReleaseControl.Request(token=list(self._token)))
        print(f"  control released: {getattr(res, 'released', False)} {getattr(res, 'message', '')}")
        self._token = bytes(16)

    def _call(self, client, req, timeout_s=5.0):
        return self._wait(client.call_async(req), timeout_s)

    @staticmethod
    def _wait(fut, timeout_s):
        t0 = time.time()
        while not fut.done():
            if time.time() - t0 > timeout_s:
                return None
            time.sleep(0.01)
        return fut.result()

    # -- planning (pure sim) ------------------------------------------------------------------
    def precheck_poses(self, poses, name, log, posture=None, start=None):
        """Gate a path of (xyz, R) tool poses in the sim (kinematics.precheck_poses), chained from
        `start` = (q, xyz, R) -- the arm's current joints and tool pose by default. Returns (worst IK
        err m, worst joint step deg, end state, joint solution of every point)."""
        if start is None:
            p, R = self.ee_pose()
            start = (self.joints(), p, R)
        return self.kin.precheck_poses(poses, name, log, start, posture=posture)

    # -- execution ----------------------------------------------------------------------------
    def smooth_poses(self, poses, knots, start, name, log, track_abort_m=TRACK_ABORT_M):
        """Send a pre-checked path as ONE joint trajectory through `knots` (its joint solutions from
        precheck_poses, planned from `start` = the arm's (joints, tool xyz, R) at the time), then
        verify the final tool position. Dry run: logs and returns the current xyz."""
        target = np.asarray(poses[-1][0], dtype=float)
        rec = {"step": name, "smooth": True, "target": target.tolist(), "n_points": len(poses), "executed": False}
        if not self.execute:
            log(rec)
            return self.ee_pos()
        q_now = self.raw_joints()
        drift = max_joint_delta_deg(q_now, start[0])
        if drift > START_DRIFT_DEG:
            raise Abort(f"{name}: the arm moved {drift:.1f} deg since this leg was planned -- nothing sent")
        path, s_knots, duration = leg_timing(q_now, knots, poses, start[1], start[2])
        t, q, qd = min_jerk_samples(path, s_knots, duration)
        code, msg = self._send(t, q, qd)
        fin = self._wait_still(target)
        err = float(np.linalg.norm(fin - target))
        rec.update(executed=True, returned=code, error=msg, duration_s=duration, ee_after=fin.tolist(), final_err_m=err)
        log(rec)
        print(f"  {name:22s} reached {np.round(fin, 3)}  ({err*1000:.1f} mm from target, {duration:.1f} s)")
        if code != 0:
            raise Abort(f"{name}: trajectory ended {ERRORS.get(code, code)} {msg} -- HOLDING HERE")
        if err > track_abort_m:
            raise Abort(f"{name}: ended {err*100:.1f} cm from the target -- HOLDING HERE")
        return fin

    def _wait_still(self, target, timeout_s=5.0, settle_reads=4, tol_m=5e-4):
        """The goal's result arrives when the reference ends; wait for the tool to stop."""
        deadline, prev, steady = time.time() + timeout_s, None, 0
        cur = self.ee_pos()
        while time.time() < deadline and steady < settle_reads:
            time.sleep(0.05)
            cur = self.ee_pos()
            steady = steady + 1 if prev is not None and np.linalg.norm(cur - prev) < tol_m else 0
            prev = cur
        return cur

    # -- long moves through RAMMP's cuRobo planner ---------------------------------------------
    def plan_curobo(self, p_goal, R_goal, start_q, name):
        """(t, q, qd) from cuRobo for the tool pose (p_goal, R_goal), starting at joints start_q
        (driver convention, not wrapped). Plans only. Raises Abort if there is no plan."""
        if not self._plan.wait_for_server(timeout_sec=5.0):
            raise Abort(f"{name}: no cuRobo planner ({CUROBO_ACTION}) -- sheppy `planner` node up? Arm not commanded")
        g = PlanToPose.Goal()
        g.target.position.x, g.target.position.y, g.target.position.z = map(float, p_goal)
        q = Rotation.from_matrix(R_goal).as_quat()
        g.target.orientation.x, g.target.orientation.y, g.target.orientation.z, g.target.orientation.w = map(float, q)
        g.start_joints = list(map(float, start_q))
        gh = self._wait(self._plan.send_goal_async(g), 10.0)
        if gh is None or not gh.accepted:
            raise Abort(f"{name}: cuRobo refused the request -- arm not commanded")
        res = self._wait(gh.get_result_async(), CUROBO_TIMEOUT_S)
        if res is None:
            gh.cancel_goal_async()
            raise Abort(f"{name}: cuRobo did not answer in {CUROBO_TIMEOUT_S:.0f} s -- arm not commanded")
        r = res.result
        if not r.success:
            raise Abort(f"{name}: cuRobo found no path ({r.message}) -- arm not commanded")
        names = list(r.trajectory.joint_names)
        idx = [names.index(j) for j in JOINTS]
        pts = r.trajectory.points
        t = np.array([pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9 for pt in pts])
        qs = np.array(unwrap_from(start_q, [np.array(pt.positions)[idx] for pt in pts]))
        return t, qs

    def gate_joint_path(self, qs, p_goal, R_goal, name, log):
        """The same gates as kinematics.precheck_poses, on a joint path that was not built from
        per-point IK: every point's FK within reach and the z band, joints off their limits, no
        step over MAX_JOINT_STEP_DEG; the end within MAX_IK_ERR_M / MAX_IK_ROT_ERR_DEG of the goal.
        Returns (end tool error m, worst step deg)."""
        worst_step = 0.0
        prev = qs[0]
        for i, q in enumerate(qs):
            p, _ = self.kin.fk(q)
            bad = []
            if np.linalg.norm(p - self.kin.base_pos) > MAX_REACH_M:
                bad.append(f"reach {np.linalg.norm(p - self.kin.base_pos):.3f} > {MAX_REACH_M}")
            if not Z_RANGE_M[0] <= p[2] - self.kin.base_pos[2] <= Z_RANGE_M[1]:
                bad.append(f"z {p[2]:.3f} outside {Z_RANGE_M}")
            step = max_joint_delta_deg(q, prev)
            worst_step = max(worst_step, step)
            if step > MAX_JOINT_STEP_DEG:
                bad.append(f"joint jump {step:.1f} deg > {MAX_JOINT_STEP_DEG}")
            bad += self.kin.limit_failures(wrap(q))
            if bad:
                log({"step": name, "precheck_failed_at": i + 1, "n_points": len(qs), "gates": bad, "planner": "curobo"})
                raise Abort(f"{name}: cuRobo path fails the gates at point {i + 1}/{len(qs)}: {'; '.join(bad)} "
                            "-- arm not commanded")
            prev = q
        p_end, q_end = self.kin.fk(qs[-1])
        err = float(np.linalg.norm(p_end - p_goal))
        rerr = float(np.degrees((Rotation.from_quat(q_end) * Rotation.from_matrix(R_goal).inv()).magnitude()))
        if err > MAX_IK_ERR_M or rerr > MAX_IK_ROT_ERR_DEG:
            raise Abort(f"{name}: cuRobo path ends {err*1000:.1f} mm / {rerr:.1f} deg off the goal -- arm not commanded")
        return err, worst_step

    def retime(self, t, qs):
        """Slow a cuRobo trajectory (planned at full speed) down to CART_SPEED / JOINT_SPEED.
        Shape unchanged; positions resampled every DT with velocities by finite differences."""
        dq = np.abs(np.diff(qs, axis=0)) / np.maximum(np.diff(t), 1e-6)[:, None]
        tool = np.array([self.kin.fk(q)[0] for q in qs])
        dv = np.linalg.norm(np.diff(tool, axis=0), axis=1) / np.maximum(np.diff(t), 1e-6)
        k = max(1.0, float(dq.max()) / JOINT_SPEED, float(dv.max()) / CART_SPEED)
        T = max(MIN_DURATION_S, float(t[-1]) * k)
        tt = np.linspace(0.0, T, max(2, int(np.ceil(T / DT)) + 1))
        q = np.stack([np.interp(tt, t * (T / t[-1]), qs[:, j]) for j in range(qs.shape[1])], axis=1)
        qd = np.gradient(q, tt, axis=0)
        qd[0] = qd[-1] = 0.0
        return tt, q, qd

    def curobo_move(self, p_goal, R_goal, name, log, track_abort_m=TRACK_ABORT_M):
        """Plan (p_goal, R_goal) with RAMMP's cuRobo from the arm's current joints, gate the plan,
        slow it to the speed caps and execute exactly that. Dry run: plan + gates only (that is
        the check). Returns the tool position reached (dry run: the current one)."""
        q0 = self.raw_joints()
        t, qs = self.plan_curobo(p_goal, R_goal, q0, name)
        err, step = self.gate_joint_path(qs, p_goal, R_goal, name, log)
        tt, q, qd = self.retime(t, qs)
        travel = float(np.linalg.norm(self.kin.fk(qs[-1])[0] - self.kin.fk(qs[0])[0]))
        print(f"  {name:16s} cuRobo {len(qs):3d} pts  end {err*1000:.1f} mm off  joint step <= {step:.1f} deg  "
              f"ok ({travel*100:.0f} cm apart, {tt[-1]:.1f} s at the speed caps)")
        target = np.asarray(p_goal, dtype=float)
        rec = {"step": name, "planner": "curobo", "target": target.tolist(), "n_points": len(qs),
               "duration_s": float(tt[-1]), "executed": False}
        if not self.execute:
            log(rec)
            return self.ee_pos()
        drift = max_joint_delta_deg(self.raw_joints(), q0)
        if drift > START_DRIFT_DEG:
            raise Abort(f"{name}: the arm moved {drift:.1f} deg since this leg was planned -- nothing sent")
        code, msg = self._send(tt, q, qd)
        fin = self._wait_still(target)
        e = float(np.linalg.norm(fin - target))
        rec.update(executed=True, returned=code, error=msg, ee_after=fin.tolist(), final_err_m=e)
        log(rec)
        print(f"  {name:22s} reached {np.round(fin, 3)}  ({e*1000:.1f} mm from target, {tt[-1]:.1f} s)")
        if code != 0:
            raise Abort(f"{name}: trajectory ended {ERRORS.get(code, code)} {msg} -- HOLDING HERE")
        if e > track_abort_m:
            raise Abort(f"{name}: ended {e*100:.1f} cm from the target -- HOLDING HERE")
        return fin

    def press_impedance(self, p_spot, n_out, depth, name, log, hold_s=0.0):
        """Press `depth` m along -n_out from the pre-press spot with the driver's compliant
        ee_pose_impedance controller, and back. The arm must be standing at the spot (position
        mode, no goal running). Holds the spot first and refuses to press if the arm sags; any
        abort closes the session at once, which latches a hold where the arm is. depth 0 = only
        the hold test. Returns {sag_m, max_depth_m, max_lateral_m}. Dry run: logs only."""
        rec = {"step": name, "impedance": True, "depth": depth, "executed": False}
        if self.cancel_requested:
            raise Abort(f"{name}: cancelled -- no session opened, HOLDING HERE")
        if not self.execute:
            log(rec)
            return None
        n_out = np.asarray(n_out, dtype=float) / np.linalg.norm(n_out)
        p0, R0 = self.ee_pose()
        if np.linalg.norm(p0 - np.asarray(p_spot)) > IMP_START_CHECK_M:
            raise Abort(f"{name}: arm is {np.linalg.norm(p0 - p_spot)*1000:.1f} mm from the pre-press spot "
                        "-- no session opened")
        quat = Rotation.from_matrix(R0).as_quat()       # hold the orientation the arm actually has
        flange_off = R0[:, 2] * TOOL_Z
        profile = impedance_profile(depth, hold_s=hold_s)

        def setpoint(p_tool):
            m = PoseSetpoint()
            f = p_tool - flange_off
            m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, f)
            m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = map(float, quat)
            m.token = list(self._token)
            return m

        for _ in range(5):                               # prime: the session must find a target waiting
            self._sp_pub.publish(setpoint(p0))
        if not self._open_stream.wait_for_service(timeout_sec=3.0):
            raise Abort("/open_stream unavailable -- nothing moved")
        res = self._call(self._open_stream, OpenStream.Request(controller=IMP_CONTROLLER, timeout_s=IMP_TIMEOUT_S,
                                                               token=list(self._token)))
        if res is not None and res.accepted:
            self.sent_goals += 1
        if res is None or not res.accepted:
            raise Abort(f"{name}: /open_stream refused: {getattr(res, 'message', 'no answer')} -- nothing moved")
        if "/setpoint/pose" not in list(res.channels):
            self._close()
            raise Abort(f"{name}: {IMP_CONTROLLER} reads {list(res.channels)}, not /setpoint/pose -- closed")
        sag = max_depth = max_lat = 0.0
        why = None
        t_start = time.monotonic()
        try:
            for t, d, phase in profile:
                while time.monotonic() - t_start < t:
                    time.sleep(0.001)
                target = p0 - d * n_out
                self._sp_pub.publish(setpoint(target))
                p = self.ee_pos()                        # raises Abort if the state goes stale
                rel = p - p0
                along = -float(rel @ n_out)              # + = into the panel
                lat = float(np.linalg.norm(rel + along * n_out))
                max_depth, max_lat = max(max_depth, along), max(max_lat, lat)
                off = float(np.linalg.norm(p - target))
                if phase == "settle":
                    sag = max(sag, off)
                if phase in ("settle", "end") and off > IMP_HOLD_ABORT_M:
                    why = f"arm drifted {off*1000:.1f} mm off the held spot ({phase}; limit {IMP_HOLD_ABORT_M*1000:.0f})"
                elif lat > IMP_LATERAL_ABORT_M:
                    why = f"arm moved {lat*1000:.1f} mm sideways off the press line (limit {IMP_LATERAL_ABORT_M*1000:.0f})"
                elif along > depth + IMP_OVERSHOOT_ABORT_M:
                    why = f"arm went {along*1000:.1f} mm in, past the commanded {depth*1000:.0f} mm"
                if why:
                    break
        except KeyboardInterrupt:
            self._close()
            print("  RammpArm: Ctrl-C -- impedance session closed, holding")
            raise
        except Abort as e:
            why = str(e)
        self._close()
        fin = self.ee_pos()
        rec.update(executed=True, sag_m=sag, max_depth_m=max_depth, max_lateral_m=max_lat,
                   ee_after=fin.tolist(), aborted=why)
        log(rec)
        print(f"  {name:22s} impedance: sag {sag*1000:.1f} mm, deepest {max_depth*1000:.1f} mm "
              f"(commanded {depth*1000:.0f}), sideways <= {max_lat*1000:.1f} mm, "
              f"ended {np.linalg.norm(fin - p0)*1000:.1f} mm from the spot")
        if why:
            raise Abort(f"{name}: {why} -- session closed, HOLDING HERE")
        return {"sag_m": sag, "max_depth_m": max_depth, "max_lateral_m": max_lat}

    def cancel_motion(self):
        """Stop the trajectory in flight, if any (the driver then holds where the arm is), and
        make every later leg of this task refuse to start. Safe from another thread."""
        self.cancel_requested = True
        gh = self._active_gh
        if gh is not None:
            gh.cancel_goal_async()

    def _close(self):
        """Close the stream session (the driver then holds at the measured q)."""
        res = self._call(self._close_stream, CloseStream.Request(token=list(self._token)), timeout_s=2.0)
        if res is None or not res.closed:
            print(f"  WARNING: /close_stream: {getattr(res, 'message', 'no answer')} "
                  f"(the session also ends {IMP_TIMEOUT_S:.1f} s after the last setpoint)")

    def _send(self, t, q, qd):
        if self.cancel_requested:
            raise Abort("cancelled -- nothing sent, HOLDING HERE")
        if not self._traj.wait_for_server(timeout_sec=5.0):
            raise Abort("no /execute_joint_trajectory action server -- nothing sent")
        g = ExecuteJointTrajectory.Goal()
        g.trajectory.joint_names = JOINTS
        for ti, qi, qdi in zip(t, q, qd):
            pt = JointTrajectoryPoint()
            pt.positions, pt.velocities = qi.tolist(), qdi.tolist()
            sec = int(ti)
            pt.time_from_start = Duration(sec=sec, nanosec=int(round((ti - sec) * 1e9)))
            g.trajectory.points.append(pt)
        tol = []
        for n in JOINTS:
            jt = JointTolerance()
            jt.name, jt.position = n, PATH_TOL_RAD
            tol.append(jt)
        g.path_tolerance = tol
        g.control_mode = 0        # POSITION
        g.preemption = 0          # QUEUE
        g.sender_id = SENDER_ID
        g.token = list(self._token)
        gh = self._wait(self._traj.send_goal_async(g), 5.0)
        if gh is None or not gh.accepted:
            raise Abort("trajectory goal REJECTED -- nothing moved")
        self._active_gh = gh
        self.sent_goals += 1
        try:
            if self.cancel_requested:                  # cancelled while this leg was being sent
                gh.cancel_goal_async()
            res = self._wait(gh.get_result_async(), float(t[-1]) + 30.0)
        except KeyboardInterrupt:
            gh.cancel_goal_async()
            time.sleep(0.2)   # let the cancel go out before the exception unwinds the process
            print("  RammpArm: Ctrl-C -- goal cancelled")
            raise
        finally:
            self._active_gh = None
        if res is None:
            gh.cancel_goal_async()
            return -100, "no result in time -- goal cancelled"
        return int(res.result.error_code), res.result.error_string


def spin_node(name: str) -> tuple[Node, SingleThreadedExecutor, threading.Thread]:
    """A node on its own executor, spun on a daemon thread (Perception spins the global one)."""
    if not rclpy.ok():
        rclpy.init()
    node = Node(name)
    ex = SingleThreadedExecutor()      # main thread only polls futures; one thread is cheaper
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    return node, ex, th
