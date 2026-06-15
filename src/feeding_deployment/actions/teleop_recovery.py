"""Manual teleoperation recovery session.

This is the robot-side consumer for the iPad teleop screen
(webapp/vue-ros-demo/src/views/Teleop.vue). When an autonomous manipulation
skill fails (e.g. opening the fridge/microwave), the executive can hand control
to the user, who jogs the arm with bounded relative steps and then returns
control to autonomy.

Message protocol (mirrors the webapp):

  webapp -> robot, on /WebAppComm (parsed by WebInterface into its receive queue):
      {"state": "teleop", "status": "command",
       "control": "move.up" | "rotate.roll_right" | "joint.4.pos" |
                  "gripper.open" | "gripper.close" | "retract",
       "step_size": "fine"|"medium"|"coarse", "value": <m or rad>, "cmd_id": <int>}
      {"state": "teleop", "status": "halt", "cmd_id": <int>}      # Stop the Retract move
      {"state": "teleop", "status": "done"}                       # exit manual mode

  robot -> webapp, on /ServerComm (via WebInterface._send_message):
      {"state": "teleop", "status": "motion_complete",
       "cmd_id": <int>, "control": <str>, "commanded": <float>, "achieved": <float>}
      {"state": "teleop", "status": "motion_aborted",
       "cmd_id": <int>, "control": <str>, "reason": <str>}

The webapp gates input: after a tap it disables every button until it receives a
matching motion_complete/motion_aborted (matched by cmd_id), so commands arrive
strictly one at a time. The only long, interruptible motion is "retract".

CONVENTIONS THAT MUST BE VERIFIED ON HARDWARE (see TELEOP_INTEGRATION.md):
  * Tool-frame axis indices (which column of the rotation matrix is the gripper's
    approach / lateral axis) and the left/right + rotation sign conventions.
  * Whether stop_action() can preempt a blocking move issued on the same RPC
    connection (the Retract -> Stop feature).
"""

from __future__ import annotations

import json
import threading
import time
from pathlib import Path
from typing import Any, Optional

import numpy as np
from scipy.spatial.transform import Rotation as R


# --- Tool-frame conventions (VERIFY ON HARDWARE) -------------------------------
# get_state()["ee_pos"] = [x, y, z, qx, qy, qz, qw] in the arm base frame.
# The orientation's rotation matrix columns are the tool-frame axes expressed in
# the base frame.
#
# EVERYTHING is relative to the gripper tip — picture the user driving a car that
# sits on the tip of the gripper. "Reach out" = forward along the direction the
# gripper points; "Up/Down" = the gripper's own up/down (NOT world-vertical);
# "Left/Right" = the gripper's sides. The frame is LIVE: after the user rotates
# the gripper, "forward" follows the new heading, exactly like turning a car.
#
# We assume the Kinova tool frame's +z is the approach axis (pointing out of the
# gripper), +x is the gripper's lateral ("left") axis, and +y is its up axis.
APPROACH_AXIS_INDEX = 2  # tool +z -> "reach out" / "pull back" (forward/back)
LATERAL_AXIS_INDEX = 0   # tool +x -> "left" / "right"
UPDOWN_AXIS_INDEX = 1    # tool +y -> "up" / "down" (gripper-relative) and yaw axis

# Flip these if a button drives the arm the wrong way on hardware.
LATERAL_SIGN = +1.0
UPDOWN_SIGN = +1.0

# Rotation: control -> (local tool-frame axis index, sign). Applied as a local
# rotation R_new = R_cur * R_delta(axis, sign*value).
ROTATION_SPEC = {
    "rotate.roll_right": (APPROACH_AXIS_INDEX, +1.0),
    "rotate.roll_left": (APPROACH_AXIS_INDEX, -1.0),
    "rotate.tilt_up": (LATERAL_AXIS_INDEX, -1.0),
    "rotate.tilt_down": (LATERAL_AXIS_INDEX, +1.0),
    "rotate.turn_left": (UPDOWN_AXIS_INDEX, +1.0),
    "rotate.turn_right": (UPDOWN_AXIS_INDEX, -1.0),
}

# Rotate about a point this far (metres) along the tool approach axis from the
# EE-frame origin. 0.0 => rotate about the EE origin. Set to the fingertip/TCP
# offset to pivot in place at a grasped handle (spec recommendation).
TCP_OFFSET_M = 0.0

# Defensive clamps: a single bounded step should never exceed these, regardless
# of what the webapp sends. Larger requests are rejected (motion_aborted).
MAX_TRANSLATION_STEP_M = 0.10
MAX_ROTATION_STEP_RAD = 0.60
MAX_JOINT_STEP_RAD = 0.60

# Exit the session if no message (including the iPad heartbeat) arrives for this
# long -> the tablet disconnected; don't strand the executive in teleop. Must be
# comfortably larger than the webapp HEARTBEAT_MS (currently 3 s).
SESSION_TIMEOUT_S = 10.0


class TeleopRecoverySession:
    """Drives one manual-recovery session against the arm and the web interface.

    Constructed with the same interfaces a HighLevelAction already holds, so it
    can be launched from any skill via HighLevelAction.run_manual_teleop_recovery.
    """

    def __init__(
        self,
        robot_interface,
        web_interface,
        retract_joint_config,
        joint_lower_limits,
        joint_upper_limits,
        log_dir: Optional[Path] = None,
        failure_context: Optional[str] = None,
        session_id: Optional[str] = None,
        teleop_speed_preset: str = "low",
    ) -> None:
        self.robot_interface = robot_interface
        self.web_interface = web_interface
        self.retract_joint_config = list(retract_joint_config)
        self.joint_lower_limits = np.asarray(joint_lower_limits, dtype=float)
        self.joint_upper_limits = np.asarray(joint_upper_limits, dtype=float)
        self.failure_context = failure_context
        self.session_id = session_id
        self.teleop_speed_preset = teleop_speed_preset

        self.log_path = None
        if log_dir is not None:
            self.log_path = Path(log_dir) / "teleop_intervention_log.jsonl"

        self._restore_speed = None

        # What the executive should do after this session, chosen by the user via
        # the Done button on a mid-skill takeover: "redo" the interrupted skill or
        # continue to the "next" skill. Defaults to "next" (also used for idle /
        # between-task teleop, where there is no skill to redo).
        self.post_teleop_action = "next"

    # -- public entry point ----------------------------------------------------

    def run(self) -> str:
        """Run the manual recovery loop until the user taps Done.

        Blocks the caller (the failed skill) until the user finishes recovery.
        Returns the user's post-teleop choice ("redo" or "next").
        """
        if self.robot_interface is None or self.web_interface is None:
            raise RuntimeError(
                "TeleopRecoverySession requires both robot_interface and web_interface"
            )

        self._enter()
        try:
            self._loop()
        finally:
            self._exit()
        return self.post_teleop_action

    # -- setup / teardown ------------------------------------------------------

    def _enter(self) -> None:
        # Slow the arm down for manual jogging; remember the old preset.
        try:
            self._restore_speed = self.robot_interface.get_speed()
        except Exception:
            self._restore_speed = None
        try:
            self.robot_interface.set_speed(self.teleop_speed_preset)
        except Exception as exc:  # non-fatal; keep going at the current speed
            print(f"[teleop] could not set speed preset: {exc}")

        # Clear stale messages, route the webapp to the teleop screen.
        self.web_interface.clear_received_messages()
        self.web_interface.current_page = "teleop"
        self.web_interface._send_message({"state": "teleop", "status": "jump"})
        self._log({"event": "session_start", "failure_context": self.failure_context})
        print("[teleop] manual recovery session started")

    def _exit(self) -> None:
        if self._restore_speed in ("low", "medium", "high"):
            try:
                self.robot_interface.set_speed(self._restore_speed)
            except Exception as exc:
                print(f"[teleop] could not restore speed preset: {exc}")
        self._log({"event": "session_end"})
        print("[teleop] manual recovery session ended")

    # -- main loop -------------------------------------------------------------

    def _loop(self) -> None:
        active = True
        while active:
            msg = self._next_teleop_message()
            if msg is None:
                # task_selection jump or interface shutting down: bail out.
                break

            status = msg.get("status")
            if status == "done":
                # "post_action" (redo/next) rides on the Done message from a
                # mid-skill takeover; absent for plain Done (defaults to "next").
                self.post_teleop_action = msg.get("post_action") or "next"
                self._log({"event": "done", "post_action": self.post_teleop_action})
                active = False
            elif status == "halt":
                # Only meaningful while a Retract move is running; handled inside
                # _handle_retract. A stray halt here is a no-op.
                self._log({"event": "halt_ignored", "cmd_id": msg.get("cmd_id")})
            elif status == "command":
                self._handle_command(msg)
            else:
                print(f"[teleop] ignoring unexpected message: {msg}")

    def _next_teleop_message(self, poll_dt: float = 0.05) -> Optional[dict[str, Any]]:
        """Block until the next actionable teleop message arrives, or None to exit.

        Returns None to exit the session on: task-selection jump, interface
        shutdown, or loss of the iPad heartbeat (SESSION_TIMEOUT_S with no message
        at all -> assume the tablet disconnected, so the executive isn't stranded).
        Heartbeat messages are consumed as keep-alive and never returned.
        """
        q = self.web_interface.received_web_interface_messages
        last_seen = time.time()
        while getattr(self.web_interface, "active", True):
            if getattr(self.web_interface, "task_selection_jump", False):
                return None
            if time.time() - last_seen > SESSION_TIMEOUT_S:
                self._log({"event": "session_timeout_no_heartbeat"})
                print(f"[teleop] no iPad heartbeat for {SESSION_TIMEOUT_S}s; exiting session")
                return None
            try:
                msg = q.get_nowait()
            except Exception:  # queue.Empty
                time.sleep(poll_dt)
                continue
            if isinstance(msg, dict) and msg.get("state") == "teleop":
                last_seen = time.time()  # any teleop message is a sign of life
                if msg.get("status") == "heartbeat":
                    continue  # keep-alive only; not an actionable command
                return msg
            # Non-teleop message while in recovery: drop it.
        return None

    # -- command handling ------------------------------------------------------

    def _handle_command(self, msg: dict[str, Any]) -> None:
        control = msg.get("control", "")
        cmd_id = msg.get("cmd_id")
        value = msg.get("value")

        self._log(
            {
                "event": "command_received",
                "control": control,
                "cmd_id": cmd_id,
                "value": value,
                "step_size": msg.get("step_size"),
            }
        )

        try:
            if control == "retract":
                self._handle_retract(cmd_id)
                return
            if control in ("gripper.open", "gripper.close"):
                self._handle_gripper(control, cmd_id)
                return
            if control.startswith("move."):
                self._handle_translation(control, value, cmd_id)
                return
            if control.startswith("rotate."):
                self._handle_rotation(control, value, cmd_id)
                return
            if control.startswith("joint."):
                self._handle_joint(control, value, cmd_id)
                return
            self._abort(cmd_id, control, f"unknown_control:{control}")
        except Exception as exc:  # any controller / RPC / safety failure
            # A force-threshold breach trips gravity-comp / emergency stop on the
            # arm, which surfaces here as an exception from execute_command.
            self._abort(cmd_id, control, self._classify_exception(exc))

    def _handle_translation(self, control: str, value: Any, cmd_id) -> None:
        from feeding_deployment.control.robot_controller.command_interface import (
            CartesianCommand,
        )

        step = self._validate_step(value, MAX_TRANSLATION_STEP_M)
        if step is None:
            self._abort(cmd_id, control, "step_too_large_or_invalid")
            return

        # Live gripper-tip frame: directions are read from the CURRENT gripper
        # orientation every tap (like driving a car that sits on the tip), so
        # "forward" follows the gripper after it rotates.
        cur_pos, cur_quat = self._current_pose()
        rot = R.from_quat(cur_quat).as_matrix()
        forward = rot[:, APPROACH_AXIS_INDEX]  # direction the gripper points
        lateral = rot[:, LATERAL_AXIS_INDEX]
        updown = rot[:, UPDOWN_AXIS_INDEX]

        dir_map = {
            "move.away": forward,                       # "Reach out"
            "move.towards": -forward,                   # "Pull back"
            "move.left": LATERAL_SIGN * lateral,
            "move.right": -LATERAL_SIGN * lateral,
            "move.up": UPDOWN_SIGN * updown,            # gripper's own up
            "move.down": -UPDOWN_SIGN * updown,
        }
        if control not in dir_map:
            self._abort(cmd_id, control, f"unknown_control:{control}")
            return

        direction = dir_map[control]
        direction = direction / (np.linalg.norm(direction) + 1e-12)
        target_pos = cur_pos + step * direction
        # Translation never changes orientation (only moves along the tip frame).
        self.robot_interface.execute_command(
            CartesianCommand(pos=target_pos, quat=cur_quat)
        )
        achieved = self._achieved_translation(cur_pos)
        self._complete(cmd_id, control, commanded=step, achieved=achieved)

    def _handle_rotation(self, control: str, value: Any, cmd_id) -> None:
        from feeding_deployment.control.robot_controller.command_interface import (
            CartesianCommand,
        )

        step = self._validate_step(value, MAX_ROTATION_STEP_RAD)
        if step is None or control not in ROTATION_SPEC:
            self._abort(cmd_id, control, "step_too_large_or_invalid")
            return

        axis_index, sign = ROTATION_SPEC[control]
        cur_pos, cur_quat = self._current_pose()
        rot_cur = R.from_quat(cur_quat)

        local_axis = np.zeros(3)
        local_axis[axis_index] = 1.0
        delta = R.from_rotvec(sign * step * local_axis)
        rot_new = rot_cur * delta  # local-frame rotation

        # Keep the TCP fixed (pivot in place at the configured tool point).
        tcp_local = np.zeros(3)
        tcp_local[APPROACH_AXIS_INDEX] = TCP_OFFSET_M
        target_pos = cur_pos + rot_cur.apply(tcp_local) - rot_new.apply(tcp_local)

        self.robot_interface.execute_command(
            CartesianCommand(pos=target_pos, quat=rot_new.as_quat())
        )
        achieved = self._achieved_rotation(rot_cur)
        self._complete(cmd_id, control, commanded=step, achieved=achieved)

    def _handle_joint(self, control: str, value: Any, cmd_id) -> None:
        from feeding_deployment.control.robot_controller.command_interface import (
            JointCommand,
        )

        # control == "joint.<N>.pos" | "joint.<N>.neg", N in 1..7
        parts = control.split(".")
        if len(parts) != 3:
            self._abort(cmd_id, control, f"bad_joint_control:{control}")
            return
        try:
            joint_num = int(parts[1])
        except ValueError:
            self._abort(cmd_id, control, f"bad_joint_index:{control}")
            return
        sign = +1.0 if parts[2] == "pos" else -1.0
        idx = joint_num - 1
        if not (0 <= idx < 7):
            self._abort(cmd_id, control, f"joint_out_of_range:{control}")
            return

        step = self._validate_step(value, MAX_JOINT_STEP_RAD)
        if step is None:
            self._abort(cmd_id, control, "step_too_large_or_invalid")
            return

        q = np.asarray(self._current_joints(), dtype=float).copy()
        before = q[idx]
        q[idx] = float(
            np.clip(
                q[idx] + sign * step,
                self.joint_lower_limits[idx],
                self.joint_upper_limits[idx],
            )
        )
        self.robot_interface.execute_command(JointCommand(pos=q))
        achieved = abs(self._current_joints()[idx] - before)
        self._complete(cmd_id, control, commanded=step, achieved=achieved)

    def _handle_gripper(self, control: str, cmd_id) -> None:
        from feeding_deployment.control.robot_controller.command_interface import (
            CloseGripperCommand,
            OpenGripperCommand,
        )

        cmd = OpenGripperCommand() if control == "gripper.open" else CloseGripperCommand()
        self.robot_interface.execute_command(cmd)
        self._complete(cmd_id, control, commanded=None, achieved=None)

    def _handle_retract(self, cmd_id) -> None:
        """Move to the safe retract joint config. Interruptible via a 'halt'.

        The move runs in a worker thread so the main loop can keep reading the
        queue; a 'halt' message calls stop_action() to preempt it.
        """
        from feeding_deployment.control.robot_controller.command_interface import (
            JointCommand,
        )

        result: dict[str, Any] = {}

        def _worker():
            try:
                self.robot_interface.execute_command(
                    JointCommand(pos=self.retract_joint_config)
                )
                result["ok"] = True
            except Exception as exc:  # noqa: BLE001
                result["error"] = self._classify_exception(exc)

        worker = threading.Thread(target=_worker, daemon=True)
        worker.start()

        halted = False
        q = self.web_interface.received_web_interface_messages
        while worker.is_alive():
            try:
                peek = q.get_nowait()
            except Exception:
                time.sleep(0.05)
                continue
            if (
                isinstance(peek, dict)
                and peek.get("state") == "teleop"
                and peek.get("status") == "halt"
            ):
                halted = True
                self._log({"event": "retract_halt_requested", "cmd_id": cmd_id})
                try:
                    self.robot_interface.stop_action()
                except Exception as exc:
                    print(f"[teleop] stop_action failed: {exc}")
                # Keep waiting for the worker to unwind after the abort.
            # Drop any other message received mid-retract.

        worker.join()

        if halted or "error" in result:
            reason = result.get("error", "halted")
            self._abort(cmd_id, "retract", reason)
        else:
            self._complete(cmd_id, "retract", commanded=None, achieved=None)

    # -- responses + logging ---------------------------------------------------

    def _complete(self, cmd_id, control, commanded, achieved) -> None:
        payload = {"state": "teleop", "status": "motion_complete", "cmd_id": cmd_id,
                   "control": control}
        if commanded is not None:
            payload["commanded"] = float(commanded)
        if achieved is not None:
            payload["achieved"] = float(achieved)
        self.web_interface._send_message(payload)
        self._log({"event": "motion_complete", "control": control, "cmd_id": cmd_id,
                   "commanded": commanded, "achieved": achieved})

    def _abort(self, cmd_id, control, reason) -> None:
        payload = {"state": "teleop", "status": "motion_aborted", "cmd_id": cmd_id,
                   "control": control, "reason": reason}
        self.web_interface._send_message(payload)
        self._log({"event": "motion_aborted", "control": control, "cmd_id": cmd_id,
                   "reason": reason})
        print(f"[teleop] motion aborted ({control}): {reason}")

    def _log(self, entry: dict[str, Any]) -> None:
        entry = dict(entry)
        entry.setdefault("screen", "teleop")
        entry["session_id"] = self.session_id
        entry["failure_context"] = self.failure_context
        entry["t"] = time.strftime("%Y-%m-%dT%H:%M:%S")
        if self.log_path is not None:
            try:
                with open(self.log_path, "a") as f:
                    f.write(json.dumps(entry) + "\n")
            except Exception as exc:
                print(f"[teleop] could not write log: {exc}")

    # -- arm-state helpers -----------------------------------------------------

    def _current_pose(self):
        ee = np.asarray(self.robot_interface.get_state()["ee_pos"], dtype=float)
        return ee[:3].copy(), ee[3:].copy()

    def _current_joints(self):
        return np.asarray(self.robot_interface.get_state()["position"], dtype=float)

    def _achieved_translation(self, start_pos) -> float:
        cur_pos, _ = self._current_pose()
        return float(np.linalg.norm(cur_pos - start_pos))

    def _achieved_rotation(self, start_rot: "R") -> float:
        _, quat = self._current_pose()
        delta = start_rot.inv() * R.from_quat(quat)
        return float(np.linalg.norm(delta.as_rotvec()))

    @staticmethod
    def _validate_step(value: Any, max_step: float) -> Optional[float]:
        try:
            step = float(value)
        except (TypeError, ValueError):
            return None
        if step <= 0.0 or step > max_step:
            return None
        return step

    @staticmethod
    def _classify_exception(exc: Exception) -> str:
        text = str(exc).lower()
        if "emergency" in text or "gravity" in text:
            return "arm_emergency_stop_active"
        if "compliant" in text:
            return "arm_in_compliant_mode"
        return f"controller_error:{exc}"
