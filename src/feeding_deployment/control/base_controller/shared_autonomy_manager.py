#!/usr/bin/env python3
"""
shared_autonomy_manager.py

The "manager" that sits in front of move_base for shared autonomy.

It exposes a `navigate` action server (move_base_msgs/MoveBaseAction) that looks
identical to move_base to its caller (NavigateHLA in actions/navigate.py, which
now connects to "navigate" instead of "move_base"). It forwards each goal to the
real move_base and relays the outcome, with one twist:

  * AUTONOMOUS (default): forward the goal to move_base; relay whatever move_base
    decides (SUCCEEDED -> succeeded, anything terminal-else -> aborted).
  * On /shared_autonomy/takeover: the human is taking over. Cancel the move_base
    goal (so it stops planning/driving and is no longer fighting the human) and
    switch to TELEOP. Localization (Cartographer / VIO) is NOT touched, so TF
    stays valid throughout.
  * On /shared_autonomy/done while in TELEOP: the human has parked the robot.
    Report SUCCEEDED to the caller ourselves ("blind success") -- move_base
    cannot, because it was cancelled.
  * On /shared_autonomy/resume while in TELEOP: the human has freed the robot but
    wants autonomy to finish the trip. Re-send the ORIGINAL goal to move_base and
    switch back to AUTONOMOUS. Because the goal is an absolute map-frame pose and
    localization was never dropped, move_base simply replans from the robot's
    current pose to the goal -- i.e. "navigate from here to the goal." The
    takeover/resume cycle can repeat if it gets stuck again.
  * On /shared_autonomy/cancel while in TELEOP: the human ended the takeover
    WITHOUT parking at the goal (e.g. a base-driving detour during an unrelated
    skill, where there is no navigation goal to "complete"). Report ABORTED, not
    SUCCEEDED -- there is no goal-reached to claim. (If no goal is active, the
    flag is simply cleared when the next goal starts.)
  * On /nav_safety_hold (ZED-divergence interlock) while AUTONOMOUS: the cmd_vel
    bridge is zeroing the motors, but move_base doesn't know -- TEB keeps
    optimizing against a robot that is secretly frozen, trips its oscillation
    detector, and resets. So PAUSE: cancel the move_base goal and wait. When the
    hold releases, re-send the original goal (same replan-from-current-pose
    mechanism as resume). Holds during TELEOP need no handling here: the goal is
    already cancelled and the bridge stops the human's commands on its own.

This is why the manager exists: an action client only believes the terminal
status from the server it is connected to, so when a human (not move_base)
finishes the goal, *something* must be the server that can legitimately say
SUCCEEDED. That something is this node.
"""

import threading

# TODO(ros2): actionlib (ROS1) has no rclpy equivalent package; using
# rclpy.action instead (ActionClient/ActionServer). This is a real
# architectural change -- ROS2 actions are natively async/futures-based
# (goal handles + async send/cancel/result) rather than the blocking
# SimpleActionClient/SimpleActionServer model this file was written against.
# See TODO(ros2) markers below at each meaningfully-changed call site.
import rclpy.action
from feeding_deployment.ros2_utils import node_handle
from feeding_deployment.ros2_utils import rospy_compat as rospy
# TODO(ros2): actionlib_msgs/GoalStatus (ROS1) does not exist in ROS2; ROS2
# actions report status via action_msgs/msg/GoalStatus, whose constants are
# STATUS_{UNKNOWN,ACCEPTED,EXECUTING,CANCELING,SUCCEEDED,CANCELED,ABORTED} --
# not the same names/values as the ROS1 actionlib_msgs constants used below
# (PREEMPTED, ABORTED, REJECTED, RECALLED, LOST, ACTIVE, SUCCEEDED). This
# import and every _TERMINAL_FAILURE / mb_state comparison below is left
# structurally in place but is NOT correct against ROS2 action status codes
# without a real rework; not guessing at the remap here.
from actionlib_msgs.msg import GoalStatus
# TODO(ros2): move_base_msgs (ROS1 .action package with a move_base_msgs
# python module) has no known ROS2 port available to this migration; ROS2
# navigation (nav2) uses nav2_msgs/action/NavigateToPose instead, which has a
# different goal/result shape (PoseStamped goal vs MoveBaseGoal, no
# MoveBaseResult). Left as-is (import will fail until this is resolved) since
# guessing at a nav2 message substitution would silently change behavior.
from move_base_msgs.msg import MoveBaseAction, MoveBaseResult
from std_msgs.msg import Bool, Empty, String

# State labels
AUTONOMOUS = "AUTONOMOUS"
TELEOP = "TELEOP"

_TERMINAL_FAILURE = (
    GoalStatus.PREEMPTED,
    GoalStatus.ABORTED,
    GoalStatus.REJECTED,
    GoalStatus.RECALLED,
    GoalStatus.LOST,
)


class SharedAutonomyManager:
    def __init__(self) -> None:
        # TODO(ros2): "~name" (rospy private param) has no exact rclpy
        # equivalent; declared as plain parameter names on this node instead.
        _node = node_handle.get_node()
        self.navigate_action = _node.declare_parameter("navigate_action", "navigate").value
        self.move_base_action = _node.declare_parameter("move_base_action", "move_base").value
        self.takeover_topic = _node.declare_parameter(
            "takeover_topic", "/shared_autonomy/takeover"
        ).value
        self.done_topic = _node.declare_parameter("done_topic", "/shared_autonomy/done").value
        self.resume_topic = _node.declare_parameter(
            "resume_topic", "/shared_autonomy/resume"
        ).value
        self.cancel_topic = _node.declare_parameter(
            "cancel_topic", "/shared_autonomy/cancel"
        ).value
        self.safety_hold_topic = _node.declare_parameter(
            "safety_hold_topic", "/nav_safety_hold"
        ).value
        self.safety_hold_reason_topic = _node.declare_parameter(
            "safety_hold_reason_topic", self.safety_hold_topic + "_reason"
        ).value
        self.loop_hz = float(_node.declare_parameter("loop_hz", 20.0).value)
        self.move_base_wait_s = float(_node.declare_parameter("move_base_wait_s", 30.0).value)

        # Edge flags set by topic callbacks, consumed in the execute loop.
        self._lock = threading.Lock()
        self._takeover_req = False
        self._done_req = False
        self._resume_req = False
        self._cancel_req = False

        # Level (not edge): mirrors the latched /nav_safety_hold Bool.
        self._hold = False
        # Latest reason string from the monitor ("" while clear); cited in the
        # pause/resume log lines so the run log records WHY navigation paused.
        self._hold_reason = ""

        # Client to the real move_base.
        # TODO(ros2): actionlib.SimpleActionClient -> rclpy.action.ActionClient.
        # wait_for_server/send_goal/cancel_goal/get_state/get_result below are
        # all blocking-call sites in the ROS1 code; rclpy's ActionClient is
        # async-only (goal handles + futures), so those calls needed real
        # restructuring, not a 1:1 swap -- see per-call TODOs below.
        self.mb_client = rclpy.action.ActionClient(
            _node, MoveBaseAction, self.move_base_action
        )
        rospy.loginfo(
            "shared_autonomy_manager: waiting for '%s' action server...",
            self.move_base_action,
        )
        if not self.mb_client.wait_for_server(timeout_sec=self.move_base_wait_s):
            raise RuntimeError(
                f"Timed out waiting for move_base action server "
                f"'{self.move_base_action}'"
            )

        _node.create_subscription(Empty, self.takeover_topic, self._on_takeover, 1)
        _node.create_subscription(Empty, self.done_topic, self._on_done, 1)
        _node.create_subscription(Empty, self.resume_topic, self._on_resume, 1)
        _node.create_subscription(Empty, self.cancel_topic, self._on_cancel, 1)
        _node.create_subscription(Bool, self.safety_hold_topic, self._on_hold, 1)
        _node.create_subscription(
            String, self.safety_hold_reason_topic, self._on_hold_reason, 1
        )

        # Our own action server.
        # TODO(ros2): actionlib.SimpleActionServer(execute_cb=..., auto_start=False)
        # -> rclpy.action.ActionServer(execute_callback=...). rclpy's
        # ActionServer has no auto_start concept -- it starts accepting goals
        # as soon as it's constructed, so the explicit .start() call below is
        # dropped. More importantly, rclpy's execute_callback signature takes
        # a `goal_handle` (not the plain `goal` this file's self._execute
        # expects), and success/abort/preempt are reported by calling
        # goal_handle.succeed()/abort()/canceled() + returning a Result from
        # within the callback -- NOT via self.server.set_succeeded/
        # set_aborted/set_preempted(...) as the body of _execute below still
        # does. This is flagged, not silently rewritten: _execute's control
        # flow (server.is_preempt_requested(), server.set_succeeded(), etc.)
        # needs a real per-call rework against goal_handle semantics before
        # this will function under rclpy. See TODOs inside _execute.
        self.server = rclpy.action.ActionServer(
            _node,
            MoveBaseAction,
            self.navigate_action,
            execute_callback=self._execute,
        )
        rospy.loginfo(
            "shared_autonomy_manager ready. Serving '%s', forwarding to '%s'.",
            self.navigate_action,
            self.move_base_action,
        )

    # ------------------------------------------------------------------ #
    # Topic callbacks
    # ------------------------------------------------------------------ #
    def _on_takeover(self, _msg: Empty) -> None:
        with self._lock:
            self._takeover_req = True

    def _on_done(self, _msg: Empty) -> None:
        with self._lock:
            self._done_req = True

    def _on_resume(self, _msg: Empty) -> None:
        with self._lock:
            self._resume_req = True

    def _on_cancel(self, _msg: Empty) -> None:
        with self._lock:
            self._cancel_req = True

    def _on_hold(self, msg: Bool) -> None:
        self._hold = bool(msg.data)

    def _on_hold_reason(self, msg: String) -> None:
        self._hold_reason = msg.data

    def _consume_flags(self):
        with self._lock:
            takeover, done, resume, cancel = (
                self._takeover_req,
                self._done_req,
                self._resume_req,
                self._cancel_req,
            )
            self._takeover_req = False
            self._done_req = False
            self._resume_req = False
            self._cancel_req = False
        return takeover, done, resume, cancel

    # ------------------------------------------------------------------ #
    # Action execution
    # ------------------------------------------------------------------ #
    def _execute(self, goal) -> None:
        # Clear any stale intents from before this goal started.
        self._consume_flags()
        state = AUTONOMOUS
        # True while we have cancelled move_base because of a safety hold and
        # are waiting for the hold to release. Only meaningful in AUTONOMOUS.
        paused = False
        paused_reason = ""

        rospy.loginfo("shared_autonomy_manager: new goal -> forwarding to move_base.")
        # TODO(ros2): actionlib SimpleActionClient.send_goal(goal) was
        # fire-and-forget (state polled via get_state() below).
        # rclpy.action.ActionClient has no send_goal() -- only
        # send_goal_async(goal), which returns a Future[ClientGoalHandle] that
        # must itself be awaited/spun before the goal is even accepted, and a
        # SEPARATE get_result_async() future for the terminal result. This
        # whole method's poll loop (get_state()/get_result() below) assumes
        # the blocking SimpleActionClient model and needs a real rework to a
        # future/callback-based one; not guessing at that rework here.
        self.mb_client.send_goal(goal)
        # Don't trust move_base's terminal status until the goal we just sent has
        # actually gone ACTIVE. Right after send_goal (initial send AND every
        # resume) get_state() can briefly report the PREVIOUS goal's terminal
        # code (e.g. PREEMPTED from the takeover cancel), which we'd otherwise
        # misread as a failure. The latch is armed on every send_goal below.
        seen_active = False

        rate = rospy.Rate(self.loop_hz)
        while not rospy.is_shutdown():
            # Upstream caller cancelled (e.g. NavigateHLA timed out and cancelled).
            # TODO(ros2): SimpleActionServer.is_preempt_requested() has no
            # rclpy.action.ActionServer equivalent -- preemption/cancellation
            # in ROS2 actions is delivered via a cancel_callback on the
            # ActionServer and goal_handle.is_cancel_requested inside
            # execute_callback, not polled here. set_preempted()/set_succeeded()/
            # set_aborted() below are similarly SimpleActionServer-only; ROS2
            # equivalents are goal_handle.canceled()/succeed()/abort() called
            # with a Result, from within execute_callback(goal_handle).
            if self.server.is_preempt_requested():
                self.mb_client.cancel_goal()
                rospy.logwarn("shared_autonomy_manager: caller preempted the goal.")
                self.server.set_preempted()
                return

            takeover, done, resume, cancel = self._consume_flags()

            if takeover and state == AUTONOMOUS:
                state = TELEOP
                paused = False
                # TODO(ros2): actionlib cancel_goal() had no rclpy.action
                # equivalent (ActionClient has no cancel_goal(); cancelling
                # requires the ClientGoalHandle returned by send_goal_async(),
                # via goal_handle.cancel_goal_async()). Needs runtime
                # verification once send_goal above is reworked to async.
                self.mb_client.cancel_goal()
                rospy.loginfo(
                    "shared_autonomy_manager: TAKEOVER -> move_base cancelled, "
                    "human in control. Waiting for 'done' or 'resume'."
                )

            if done:
                if state == TELEOP:
                    rospy.loginfo(
                        "shared_autonomy_manager: DONE -> reporting SUCCEEDED "
                        "(human-completed)."
                    )
                    # TODO(ros2): SimpleActionServer.set_succeeded(result, text)
                    # has no rclpy.action.ActionServer equivalent -- reporting
                    # a terminal result in ROS2 is done by calling
                    # goal_handle.succeed() then returning the Result from
                    # execute_callback(goal_handle), not by calling a method
                    # on self.server from arbitrary code.
                    self.server.set_succeeded(
                        MoveBaseResult(), "Goal completed by human teleoperation."
                    )
                    return
                # DONE while AUTONOMOUS: the takeover that should have preceded it
                # never registered, so move_base still owns the goal and we cannot
                # honestly claim a human completion. Don't fail the goal (the human
                # may well have parked it fine), but surface the dropped takeover
                # instead of swallowing DONE silently -- the caller (navigate.py)
                # latches the Done on its own side and skips the confirm re-drive.
                rospy.logwarn(
                    "shared_autonomy_manager: DONE received while AUTONOMOUS -- no "
                    "takeover was registered, so move_base still owns the goal. The "
                    "takeover was likely dropped; ignoring this DONE."
                )

            if cancel and state == TELEOP:
                # Human ended the takeover without reaching the goal. Report
                # ABORTED so the caller does NOT treat it as goal-reached.
                rospy.loginfo(
                    "shared_autonomy_manager: CANCEL -> reporting ABORTED "
                    "(takeover ended without reaching the goal)."
                )
                # TODO(ros2): see set_succeeded TODO above -- same
                # goal_handle.abort()-from-within-execute_callback rework
                # needed for set_aborted().
                self.server.set_aborted(
                    MoveBaseResult(), "Teleop takeover cancelled by human."
                )
                return

            if resume and state == TELEOP:
                # Hand control back to autonomy: replan from the robot's current
                # pose to the same (absolute, map-frame) goal. Refresh the stamp
                # so move_base doesn't reject it as stale; re-arm the latch so the
                # newly-sent goal's status is what we act on.
                state = AUTONOMOUS
                goal.target_pose.header.stamp = rospy.now().to_msg()
                self.mb_client.send_goal(goal)
                seen_active = False
                rospy.loginfo(
                    "shared_autonomy_manager: RESUME -> re-sent goal, autonomy "
                    "driving from current pose to the goal."
                )

            if state == AUTONOMOUS:
                # Safety hold (ZED-divergence interlock): the bridge is zeroing
                # the motors, so cancel the move_base goal rather than let TEB
                # optimize against a frozen robot (oscillation false-positives,
                # timediff<=0 resets). Re-send when the hold releases -- same
                # replan-from-current-pose mechanism as resume.
                if self._hold and not paused:
                    paused = True
                    paused_reason = self._hold_reason or "reason not yet received"
                    self.mb_client.cancel_goal()
                    rospy.logwarn(
                        "shared_autonomy_manager: safety HOLD [%s] -> goal paused "
                        "(move_base cancelled); will re-send on recovery.",
                        paused_reason,
                    )
                elif not self._hold and paused:
                    paused = False
                    goal.target_pose.header.stamp = rospy.now().to_msg()
                    self.mb_client.send_goal(goal)
                    seen_active = False
                    rospy.loginfo(
                        "shared_autonomy_manager: HOLD released (was: %s) -> goal "
                        "re-sent, autonomy resuming from current pose.",
                        paused_reason,
                    )
                if paused:
                    rate.sleep()
                    continue

                # TODO(ros2): SimpleActionClient.get_state() (synchronous,
                # returns the latest actionlib_msgs/GoalStatus int) has no
                # rclpy.action.ActionClient equivalent -- status lives on the
                # ClientGoalHandle from send_goal_async(), and is only
                # observable via that handle's status field or a
                # goal_handle.status callback, not polled from the client
                # object itself.
                mb_state = self.mb_client.get_state()
                if mb_state == GoalStatus.ACTIVE:
                    seen_active = True
                if mb_state == GoalStatus.SUCCEEDED:
                    # A cancelled goal reports PREEMPTED, never SUCCEEDED, so a
                    # SUCCEEDED here always belongs to the goal we sent -- safe to
                    # honor without the ACTIVE latch.
                    rospy.loginfo(
                        "shared_autonomy_manager: move_base SUCCEEDED -> relaying."
                    )
                    # TODO(ros2): get_result() (synchronous) has no
                    # rclpy.action.ActionClient equivalent -- only
                    # get_result_async() on a ClientGoalHandle, which itself
                    # requires the goal to already be accepted/tracked via
                    # the send_goal_async() future.
                    result = self.mb_client.get_result() or MoveBaseResult()
                    self.server.set_succeeded(result, "move_base reached the goal.")
                    return
                # Only honor a FAILURE once the current goal has gone ACTIVE --
                # right after a (re)send, get_state() can briefly return the prior
                # goal's PREEMPTED (from the takeover cancel), which we must not
                # misread as this goal failing.
                if seen_active and mb_state in _TERMINAL_FAILURE:
                    rospy.logwarn(
                        "shared_autonomy_manager: move_base ended in state %d "
                        "without a takeover -> aborting.",
                        mb_state,
                    )
                    # TODO(ros2): see get_result() TODO above.
                    result = self.mb_client.get_result() or MoveBaseResult()
                    self.server.set_aborted(
                        result, f"move_base terminated in state {mb_state}."
                    )
                    return

            rate.sleep()

        # rospy shutting down mid-goal.
        # TODO(ros2): see cancel_goal() TODO above.
        self.mb_client.cancel_goal()


def main() -> None:
    node_handle.init_node("shared_autonomy_manager")
    SharedAutonomyManager()
    rospy.spin()


if __name__ == "__main__":
    main()
