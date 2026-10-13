"""`/press_button` action server: the button press as a node RAMMP's stack can call.

Same detection, planning and gates as `press_button` (it runs `press_button.press_button` per
goal), but the camera, arm interface and kinematics stay loaded between goals, and progress
streams back as feedback (one line per stage, as the script prints them).

SAFETY: the arm moves only if the node was started with `allow_execute:=true` AND the goal asks
for `execute`. Otherwise the goal runs as a dry run (detect + plan + gate every leg; nothing
moves) and the result says so. Someone stands at the e-stop whenever allow_execute is on.
Cancelling a goal stops the trajectory in flight; the driver holds the arm where it is, and the
goal ends there (no automatic return). One goal at a time: a second one is rejected.

    ros2 run rammp_button_press press_server --ros-args -p allow_execute:=false
    ros2 run rammp_button_press press_client --target start_30s            # dry run
"""
from __future__ import annotations

import contextlib
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from rammp_button_press import Abort, dome_pattern
from rammp_button_press.press_button import MAX_PRESS_IN_M, PRESS_IN_M, Run, build_parser, press_button
from rammp_button_press_interfaces.action import PressButton

MAX_PRESSES = 10


class _FeedbackLines:
    """stdout replacement for one goal: every complete line goes to the real stdout and out as
    action feedback."""

    def __init__(self, goal_handle, real):
        self.gh, self.real, self.buf = goal_handle, real, ""

    def write(self, s):
        self.real.write(s)
        self.buf += s
        while "\n" in self.buf:
            line, self.buf = self.buf.split("\n", 1)
            if line.strip():
                fb = PressButton.Feedback()
                fb.stage = line.rstrip()
                self.gh.publish_feedback(fb)
        return len(s)

    def flush(self):
        self.real.flush()


class PressServer(Node):
    def __init__(self):
        super().__init__("button_press")
        self.allow_execute = bool(self.declare_parameter("allow_execute", False).value)
        argv = ["--arm-frame", self.declare_parameter("arm_frame", "base_link").value,
                "--camera-frame", self.declare_parameter("camera_frame", "camera_color_optical_frame").value,
                "--camera-ns", self.declare_parameter("camera_ns", "/wrist_camera").value,
                "--long-moves", self.declare_parameter("long_moves", "curobo").value,
                "--log-dir", self.declare_parameter("log_dir", build_parser().get_default("log_dir")).value]
        if self.declare_parameter("acquire_control", False).value:
            argv.append("--acquire-control")
        if self.allow_execute:
            argv.append("--execute")
        self.base_argv = argv
        # One Run for the node's life: camera, arm interface and kinematics stay loaded.
        self.run = Run(build_parser().parse_args(argv))
        self.pub = self.run.per.create_publisher(PoseStamped, "/button_press/target_pose", 1)
        self.busy = threading.Lock()
        group = ReentrantCallbackGroup()   # cancel requests must get through while a goal runs
        self.server = ActionServer(self, PressButton, "press_button", execute_callback=self.execute,
                                   goal_callback=self.on_goal, cancel_callback=self.on_cancel,
                                   callback_group=group)
        self.get_logger().info(f"/press_button ready; allow_execute={self.allow_execute} "
                               f"({'the arm WILL move on execute goals' if self.allow_execute else 'every goal is a dry run'})")

    def on_goal(self, goal):
        if self.busy.locked():
            self.get_logger().warn("rejected: a press is already running")
            return GoalResponse.REJECT
        if goal.target not in dome_pattern.NAMES:
            self.get_logger().warn(f"rejected: unknown target {goal.target!r} (known: {dome_pattern.NAMES})")
            return GoalResponse.REJECT
        if goal.presses > MAX_PRESSES or not 0.0 <= goal.press_in_m <= MAX_PRESS_IN_M:
            self.get_logger().warn(f"rejected: presses {goal.presses} (max {MAX_PRESSES}) / press_in_m "
                                   f"{goal.press_in_m} (0 = default, max {MAX_PRESS_IN_M})")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def on_cancel(self, _goal_handle):
        self.get_logger().warn("cancel requested: stopping the arm where it is")
        self.run.arm.cancel_motion()
        return CancelResponse.ACCEPT

    def execute(self, gh):
        g = gh.request
        res = PressButton.Result()
        with self.busy:
            execute = bool(g.execute and self.allow_execute)
            argv = [a for a in self.base_argv if a != "--execute"]
            argv += ["--target", g.target, "--presses", str(g.presses),
                     "--press-in", str(g.press_in_m or PRESS_IN_M)]
            if execute:
                argv.append("--execute")
            if g.no_return:
                argv.append("--no-return")
            a = build_parser().parse_args(argv)
            run = self.run
            run.args, run.arm.execute = a, execute
            run.arm.cancel_requested = False
            run.start_log()
            res.log_file = run.log_path
            sent0 = run.arm.sent_goals
            note = "" if execute or not g.execute else "allow_execute is false: ran as a DRY RUN -- "
            self.get_logger().info(f"goal: {g.target} x{g.presses} {'EXECUTE' if execute else 'dry run'}")
            out = _FeedbackLines(gh, sys.stdout)
            try:
                with contextlib.redirect_stdout(out):
                    code = press_button(run, a, self.pub)
                res.success = code == 0
                res.message = note + ("done" if execute else "dry run passed every gate; nothing moved")
            except Abort as e:
                res.success, res.message = False, note + str(e)
            except Exception as e:  # noqa: BLE001 -- report, never kill the server mid-goal
                res.success, res.message = False, f"{type(e).__name__}: {e}"
                self.get_logger().error(res.message)
            res.moved = run.arm.sent_goals > sent0
            if gh.is_cancel_requested:
                gh.canceled()
                res.message = "cancelled -- " + res.message
            elif res.success:
                gh.succeed()
            else:
                gh.abort()
            self.get_logger().info(f"result: success={res.success} moved={res.moved} {res.message}")
            return res

    def close(self):
        self.run.arm.close()


def main(argv=None) -> int:
    rclpy.init(args=argv)
    node = None
    try:
        node = PressServer()
        ex = MultiThreadedExecutor()
        ex.add_node(node)
        ex.spin()
    except (KeyboardInterrupt, Abort) as e:
        if isinstance(e, Abort):
            print(f"press_server: {e}")
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    time.sleep(0.1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
