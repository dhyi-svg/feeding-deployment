"""`SheppyArm` -- the arm for the microwave scripts under sheppy: a drop-in for
`ArmInterfaceClient`'s two methods (`get_state()`, `execute_command(cmd)`), backed by
`kinova_gen3_node` (~/kinova-gen3-ros2) instead of the old `arm_server.py` RPC.

    get_state()  -> {"position", "velocity", "effort", "ee_pos" [x y z qx qy qz qw], "gripper_pos"}
                    from /joint_states, /ee_state, /gripper_state (joints wrapped to +-pi, like Kortex).
                    `ee_pos` is Kinova's `tool_frame` (flange + TOOL_Z along the flange z), like the
                    Kortex tool pose the scripts were tuned on -- /ee_state itself is the flange
                    (`end_effector_link`).
    execute_command(JointCommand)              -> one `execute_joint_trajectory` goal, current -> target
    execute_command(CartesianTrajectoryCommand)-> the poses IK'd in PyBullet (chained from the real
                                                  joints, the same `solve_ik` the sim gates use), then
                                                  ONE `execute_joint_trajectory` goal through them
    execute_command(Open/CloseGripperCommand)  -> /setpoint/gripper streamed until it settles

All blocking; each returns True on success. The driver has no Cartesian-trajectory action, so
the arm now follows the exact joint path the scripts' sim gates checked, not Kortex's own
Cartesian interpolation. Trajectories are dense (20 ms) with velocities, so the driver does
cubic Hermite (smooth), not linear (a velocity step at every knot); speed follows a minimum-
jerk profile over the whole path. Free-spinning joints go the short way round (what Kortex did).

`path_tolerance` is set on every goal: the driver aborts (-4) if any joint lags its reference
by more than PATH_TOL_RAD -- the tracking-abort the old scripts did by polling.
Ctrl-C while a goal runs cancels it (the arm stops) and re-raises.

Needs the hybrid env (NOTES.md): /opt/ros/humble + ~/ros2_ws sourced, ROS_DOMAIN_ID=0,
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp, sheppy profile real-arm-camera up.
"""
import threading, time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.msg import JointTolerance
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from scipy.spatial.transform import Rotation as R

from rammp_arm_interfaces.action import ExecuteJointTrajectory
from rammp_arm_interfaces.msg import EeState, GripperSetpoint, GripperState, JointImpedanceGains

from feeding_deployment.control.robot_controller.command_interface import (
    CartesianTrajectoryCommand, CloseGripperCommand, JointCommand, OpenGripperCommand)

JOINTS = [f"joint_{i}" for i in range(1, 8)]
CONTINUOUS = [0, 2, 4, 6]           # J1/J3/J5/J7 spin freely
TOOL_Z = 0.120                      # end_effector_link -> tool_frame (Kinova's 2F-85 tool, robot_controller/urdfs)
DT = 0.02                           # trajectory sample spacing (cuRobo's interpolation_dt)
JOINT_SPEED = 0.85                  # rad/s peak for joint moves (~49 deg/s); was 0.35, 0.5 (10-07), 0.65 then 0.85 (10-24,
                                    # "everything felt slow")
CART_SPEED = 0.16                   # m/s peak for Cartesian paths; was 0.06, 0.09 (10-07), 0.12 then 0.16 (10-24)
# Contact moves (the push-open into the door, the push-close push) CAN'T HANDLE MORE SPEED than the 10-07
# values below -- keep them there however fast the rest goes: at 0.65 rad/s / 0.12 m/s the push-open's
# last leg (door pushing back + a 68 deg hand turn) aborted PATH_TOLERANCE_VIOLATED (10-24); it ran clean
# three times at these.
CONTACT_JOINT_SPEED = 0.5
CONTACT_CART_SPEED = 0.09


def contact_speed_scale():
    """speed_scale for move_joints that keeps a contact move at CONTACT_*_SPEED however fast the rest is."""
    return min(1.0, CONTACT_CART_SPEED / CART_SPEED, CONTACT_JOINT_SPEED / JOINT_SPEED)


MIN_DURATION_S = 1.0
CART_STEP_M = 0.02                  # Cartesian segments are cut into steps this long ...
CART_STEP_DEG = 5.0                 # ... and turning at most this much (= plan_cartesian)
ROT_EQUIV_M = 0.15                  # a turn of 1 rad counts as 15 cm of travel (6 cm/s -> ~23 deg/s)
MAX_KNOT_JUMP_DEG = 20.0            # refuse a step that moves any joint more than this (= MAX_STEP_JUMP_DEG)
PATH_TOL_RAD = 0.15                # ~8.6 deg tracking guard per joint (driver-side abort)
GRIPPER_FORCE = 0.5
GRIPPER_SPEED = 0.5
# Joint impedance for compliant moves (the door swing): the driver's documented teleop starting point
# (kinova-gen3-driver docs/guide/control-modes.md). In impedance mode lag ~ force (kq x lag), so the path
# tolerance doubles as a force stop. NOT USED for the door swing (scrapped 10-21): 0.08 rad aborted just after
# the latch; 0.30 rad let the arm swing wildly once the handle slipped out of the fingers (e-stopped).
IMPEDANCE = {"kq": (80.0, 80.0, 80.0, 80.0, 30.0, 30.0, 30.0), "zeta": 0.5,
             "torque_limit": (39.0, 39.0, 39.0, 39.0, 9.0, 9.0, 9.0), "path_tol": 0.08}
ERRORS = {0: "SUCCESSFUL", -1: "INVALID_GOAL", -4: "PATH_TOLERANCE_VIOLATED",
          -5: "GOAL_TOLERANCE_VIOLATED", -6: "PREEMPTED", -8: "NOT_AUTHORIZED", -9: "HALTED"}


def _wrap(q):
    q = np.asarray(q, dtype=float).copy()
    q[CONTINUOUS] = (q[CONTINUOUS] + np.pi) % (2 * np.pi) - np.pi
    return q


def _unwrap_from(q_start, knots):
    """Knots made continuous from `q_start`: each free joint steps the short way from the previous knot."""
    out, prev = [], np.asarray(q_start, dtype=float)
    for k in knots:
        d = np.asarray(k, dtype=float) - prev
        d[CONTINUOUS] = (d[CONTINUOUS] + np.pi) % (2 * np.pi) - np.pi
        prev = prev + d
        out.append(prev.copy())
    return out


def _min_jerk_samples(path, s_knots, duration):
    """Positions + velocities every DT along a piecewise-linear joint path `path` (knots at path
    parameter `s_knots`, 0..L), timed by a minimum-jerk s(t) over `duration`."""
    path, L = np.asarray(path), float(s_knots[-1])
    n = max(2, int(np.ceil(duration / DT)) + 1)
    t = np.linspace(0.0, duration, n)
    u = t / duration
    s = L * (10 * u**3 - 15 * u**4 + 6 * u**5)
    q = np.stack([np.interp(s, s_knots, path[:, j]) for j in range(7)], axis=1)
    qd = np.gradient(q, t, axis=0)
    qd[0] = qd[-1] = 0.0
    return t, q, qd


def cartesian_knots(sim, ee_start, q_start, traj):
    """Joint knots for `traj` from tool pose `ee_start` / joints `q_start`: straight lines with
    slerped orientation between consecutive poses, split into <= CART_STEP_M / CART_STEP_DEG steps,
    each IK'd from the previous (same re-solve as `plan_cartesian`). Returns (knots, segment
    lengths in m), or (None, None) if a step misses the IK tolerance or jumps a joint more than
    MAX_KNOT_JUMP_DEG. Pure sim -- used by `move_cartesian`, and testable offline."""
    from scipy.spatial.transform import Slerp
    from microwave_common import J4_GUARD_DEG, J6_GUARD_DEG, MAX_IK_ERR, solve_ik
    scene, rb = sim
    q, prev_p, prev_q = np.asarray(q_start, float), np.asarray(ee_start[:3], float), np.asarray(ee_start[3:7], float)
    knots, lengths = [], []
    for k, (pos, quat) in enumerate(traj):
        pos, quat = np.asarray(pos, float), np.asarray(quat, float)
        rots = R.from_quat([prev_q, quat])
        ang = float(np.degrees((rots[0].inv() * rots[1]).magnitude()))
        dist = float(np.linalg.norm(pos - prev_p))
        n = max(1, int(np.ceil(max(dist / CART_STEP_M, ang / CART_STEP_DEG))))
        slerp = Slerp([0, 1], rots)
        for i in range(1, n + 1):
            f = i / n
            tp, tq = prev_p + (pos - prev_p) * f, slerp([f]).as_quat()[0]
            nq, err = solve_ik(scene, rb, tp, tq, q)
            for _ in range(3):
                if err <= MAX_IK_ERR:
                    break
                nq, err = solve_ik(scene, rb, tp, tq, nq)
            d = nq - q
            d[CONTINUOUS] = (d[CONTINUOUS] + np.pi) % (2 * np.pi) - np.pi
            jump = float(np.degrees(np.max(np.abs(d))))
            j4, j6 = abs(np.degrees(nq[3])), abs(np.degrees(nq[5]))
            if err > MAX_IK_ERR or jump > MAX_KNOT_JUMP_DEG or j4 > J4_GUARD_DEG or j6 > J6_GUARD_DEG:
                print(f"  SheppyArm: waypoint {k + 1}/{len(traj)} step {i}/{n}: IK err {err * 100:.1f} cm, "
                      f"jump {jump:.1f} deg, J4 {j4:.1f}, J6 {j6:.1f} -- nothing sent.")
                return None, None
            knots.append(nq)
            lengths.append(max(1e-4, dist / n, np.radians(ang / n) * ROT_EQUIV_M))
            q = nq
        prev_p, prev_q = pos, quat
    return knots, lengths


class SheppyArm:
    def __init__(self, node_name="microwave_sheppy_arm"):
        if not rclpy.ok():
            rclpy.init()
        self.node = Node(node_name)
        self._lock = threading.Lock()
        self._js = self._ee = self._grip = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, qos_profile_sensor_data)
        self.node.create_subscription(EeState, "/ee_state", self._on_ee, qos_profile_sensor_data)
        self.node.create_subscription(GripperState, "/gripper_state", self._on_grip, qos_profile_sensor_data)
        self._traj = ActionClient(self.node, ExecuteJointTrajectory, "/execute_joint_trajectory")
        self._grip_pub = self.node.create_publisher(
            GripperSetpoint, "/setpoint/gripper",
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1))
        self._exec = MultiThreadedExecutor()
        self._exec.add_node(self.node)
        threading.Thread(target=self._exec.spin, daemon=True).start()
        self._sim = None
        self.last_path = []
        t0 = time.time()
        while (self._js is None or self._ee is None or self._grip is None) and time.time() - t0 < 10.0:
            time.sleep(0.05)
        if self._js is None or self._ee is None or self._grip is None:
            raise RuntimeError("no /joint_states, /ee_state or /gripper_state -- is the sheppy arm node up, "
                               "and ROS_DOMAIN_ID / RMW_IMPLEMENTATION exported?")
        if not self._traj.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("no /execute_joint_trajectory action server")

    # ---- state -------------------------------------------------------------------------------
    def _on_js(self, m):
        if len(m.position) >= 7:
            idx = [list(m.name).index(n) for n in JOINTS]
            with self._lock:
                self._js = (np.array([m.position[i] for i in idx]), np.array([m.velocity[i] for i in idx]),
                            np.array([m.effort[i] for i in idx]))

    def _on_ee(self, m):
        p, o = m.pose.position, m.pose.orientation
        quat = np.array([o.x, o.y, o.z, o.w])
        tool = np.array([p.x, p.y, p.z]) + R.from_quat(quat).apply([0.0, 0.0, TOOL_Z])
        with self._lock:
            self._ee = np.concatenate([tool, quat])

    def _on_grip(self, m):
        with self._lock:
            self._grip = float(m.position)

    def get_state(self):
        with self._lock:
            q, dq, tau = self._js
            return {"position": _wrap(q), "velocity": dq.copy(), "effort": tau.copy(),
                    "ee_pos": self._ee.copy(), "gripper_pos": self._grip}

    # ---- commands ----------------------------------------------------------------------------
    def execute_command(self, cmd):
        if isinstance(cmd, JointCommand):
            return self.move_joints([np.asarray(cmd.pos, dtype=float)])
        if isinstance(cmd, CartesianTrajectoryCommand):
            return self.move_cartesian(cmd.traj)
        if isinstance(cmd, OpenGripperCommand):
            return self.gripper(0.0)
        if isinstance(cmd, CloseGripperCommand):
            return self.gripper(1.0)
        raise TypeError(f"SheppyArm: unsupported command {type(cmd).__name__}")

    def move_joints(self, knots, cart_lengths=None, speed_scale=1.0, impedance=None, monitor=None):
        """Through joint `knots` (wrapped or not) from the measured joints. `cart_lengths`: the
        Cartesian distance of each segment -- given, the path is timed in metres at CART_SPEED,
        otherwise in joint-space radians at JOINT_SPEED. `impedance`: a dict like IMPEDANCE -> run
        compliant (joint impedance) instead of stiff position control. `monitor`: polled at ~100 Hz
        while the trajectory runs; when it returns True the goal is cancelled (the driver stops the
        arm and holds), `monitor_stopped` is set and the call returns True."""
        q0 = np.array(self._js[0])
        path = [q0] + _unwrap_from(q0, knots)
        seg = (np.asarray(cart_lengths, dtype=float) if cart_lengths is not None
               else np.array([np.max(np.abs(b - a)) for a, b in zip(path[:-1], path[1:])]))
        s_knots = np.concatenate([[0.0], np.cumsum(seg)])
        if s_knots[-1] < 1e-6:
            return True
        # min-jerk peak speed is 1.875 x the average
        speed = (CART_SPEED if cart_lengths is not None else JOINT_SPEED) * speed_scale
        # never faster than JOINT_SPEED on any joint either, whatever the Cartesian length says
        joint_travel = float(np.sum([np.max(np.abs(b - a)) for a, b in zip(path[:-1], path[1:])]))
        duration = max(MIN_DURATION_S, 1.875 * s_knots[-1] / speed, 1.875 * joint_travel / (JOINT_SPEED * speed_scale))
        t, q, qd = _min_jerk_samples(path, s_knots, duration)
        import microwave_common
        if microwave_common.KEEP_OUT is not None:     # fridge runs only: the wheelchair keep-out, every 2nd sample + end
            rb = self._sim_get()[1]
            for qi in list(q[::2]) + [q[-1]]:
                msg = microwave_common.KEEP_OUT(rb, qi)
                if msg is not None:
                    print(f"  SheppyArm: the trajectory enters the keep-out zone ({msg}) -- nothing sent.")
                    return False
        return self._send(t, q, qd, impedance, monitor)

    def move_cartesian(self, traj, impedance=None, speed_scale=1.0):
        """`traj`: [(pos, quat xyzw), ...] in the base frame, like CartesianTrajectoryCommand. Each
        segment (from the current pose) is a straight line with slerped orientation -- what Kortex
        did -- cut into <= CART_STEP_M / CART_STEP_DEG steps before IK, the same density as
        `plan_cartesian`, so the joint path is the one the sim gates checked."""
        knots, lengths = cartesian_knots(self._sim_get(), np.array(self._ee), np.array(self._js[0]), traj)
        if knots is None:
            return False
        return self.move_joints(knots, cart_lengths=lengths, impedance=impedance, speed_scale=speed_scale)

    def _sim_get(self):
        if self._sim is None:
            from microwave_common import make_sim
            self._sim = make_sim()
        return self._sim

    def _send(self, t, q, qd, impedance=None, monitor=None):
        g = ExecuteJointTrajectory.Goal()
        g.trajectory.joint_names = JOINTS
        for ti, qi, qdi in zip(t, q, qd):
            pt = JointTrajectoryPoint()
            pt.positions, pt.velocities = qi.tolist(), qdi.tolist()
            sec = int(ti)
            pt.time_from_start = Duration(sec=sec, nanosec=int(round((ti - sec) * 1e9)))
            g.trajectory.points.append(pt)
        tol = []
        for name in JOINTS:
            jt = JointTolerance()
            jt.name, jt.position = name, (impedance["path_tol"] if impedance else PATH_TOL_RAD)
            tol.append(jt)
        g.path_tolerance = tol
        g.control_mode = 0                 # POSITION
        if impedance:
            g.control_mode = 1             # IMPEDANCE
            g.gains = JointImpedanceGains(kq=list(impedance["kq"]), zeta=float(impedance["zeta"]),
                                          torque_limit=list(impedance["torque_limit"]))
        g.preemption = 0                   # QUEUE
        g.sender_id = "microwave"
        fut = self._traj.send_goal_async(g)
        gh = self._wait(fut, 5.0)
        if gh is None or not gh.accepted:
            print("  SheppyArm: trajectory goal REJECTED")
            return False
        # record the measured tool path while the goal runs (`last_path`: [(t, x, y, z)]) -- in
        # impedance mode this is where the door actually let the hand go
        self.last_path, rec_stop = [], threading.Event()

        def _rec(t0=time.time()):
            while not rec_stop.is_set():
                e = self._ee
                self.last_path.append((time.time() - t0, float(e[0]), float(e[1]), float(e[2])))
                time.sleep(0.02)
        threading.Thread(target=_rec, daemon=True).start()
        self.monitor_stopped = False
        try:
            rf, t_end, res = gh.get_result_async(), time.time() + t[-1] + 30.0, None
            while not rf.done() and time.time() < t_end:
                if monitor is not None and not self.monitor_stopped and monitor():
                    self.monitor_stopped = True
                    gh.cancel_goal_async()          # the driver stops the arm, then settles PREEMPTED
                time.sleep(0.01)
            if rf.done():
                res = rf.result()
        except KeyboardInterrupt:
            gh.cancel_goal_async()
            print("  SheppyArm: Ctrl-C -- goal cancelled")
            raise
        finally:
            rec_stop.set()
        if res is None:
            gh.cancel_goal_async()
            print("  SheppyArm: no result in time -- goal cancelled")
            return False
        code = res.result.error_code
        if self.monitor_stopped and code == -6:
            return True
        if code != 0:
            print(f"  SheppyArm: trajectory ended {ERRORS.get(code, code)} {res.result.error_string}")
            fe = list(res.result.final_error.positions)
            if fe:
                print(f"  SheppyArm: final joint error (ref - actual) deg J1-J7 {np.round(np.degrees(fe), 1)}")
        return code == 0

    @staticmethod
    def _wait(fut, timeout_s):
        t0 = time.time()
        while not fut.done():
            if time.time() - t0 > timeout_s:
                return None
            time.sleep(0.01)
        return fut.result()

    def gripper(self, position, timeout_s=4.0, force=None):
        """Stream /setpoint/gripper at 50 Hz until the fingers stop moving (or `timeout_s`). The
        2F-85 latches the last command, so it holds after the stream stops."""
        m = GripperSetpoint()
        m.position, m.speed = float(position), GRIPPER_SPEED
        m.force = float(GRIPPER_FORCE if force is None else force)   # a current ceiling: lower = looser hold
        t0, last, still_since = time.time(), self._grip, None
        while time.time() - t0 < timeout_s:
            self._grip_pub.publish(m)
            time.sleep(0.02)
            g = self._grip
            if abs(g - last) > 0.002:
                last, still_since = g, None
            elif time.time() - t0 > 0.5:
                still_since = still_since or time.time()
                if time.time() - still_since > 0.3:
                    break
        return True

    def close(self):
        self._exec.shutdown()
        self.node.destroy_node()
