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

This is why the manager exists: an action client only believes the terminal
status from the server it is connected to, so when a human (not move_base)
finishes the goal, *something* must be the server that can legitimately say
SUCCEEDED. That something is this node.
"""

import threading

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseResult
from std_msgs.msg import Empty

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
        self.navigate_action = rospy.get_param("~navigate_action", "navigate")
        self.move_base_action = rospy.get_param("~move_base_action", "move_base")
        self.takeover_topic = rospy.get_param(
            "~takeover_topic", "/shared_autonomy/takeover"
        )
        self.done_topic = rospy.get_param("~done_topic", "/shared_autonomy/done")
        self.resume_topic = rospy.get_param(
            "~resume_topic", "/shared_autonomy/resume"
        )
        self.cancel_topic = rospy.get_param(
            "~cancel_topic", "/shared_autonomy/cancel"
        )
        self.loop_hz = float(rospy.get_param("~loop_hz", 20.0))
        self.move_base_wait_s = float(rospy.get_param("~move_base_wait_s", 30.0))

        # Edge flags set by topic callbacks, consumed in the execute loop.
        self._lock = threading.Lock()
        self._takeover_req = False
        self._done_req = False
        self._resume_req = False
        self._cancel_req = False

        # Client to the real move_base.
        self.mb_client = actionlib.SimpleActionClient(
            self.move_base_action, MoveBaseAction
        )
        rospy.loginfo(
            "shared_autonomy_manager: waiting for '%s' action server...",
            self.move_base_action,
        )
        if not self.mb_client.wait_for_server(rospy.Duration(self.move_base_wait_s)):
            raise RuntimeError(
                f"Timed out waiting for move_base action server "
                f"'{self.move_base_action}'"
            )

        rospy.Subscriber(self.takeover_topic, Empty, self._on_takeover, queue_size=1)
        rospy.Subscriber(self.done_topic, Empty, self._on_done, queue_size=1)
        rospy.Subscriber(self.resume_topic, Empty, self._on_resume, queue_size=1)
        rospy.Subscriber(self.cancel_topic, Empty, self._on_cancel, queue_size=1)

        # Our own action server. auto_start=False so we can start() after setup.
        self.server = actionlib.SimpleActionServer(
            self.navigate_action,
            MoveBaseAction,
            execute_cb=self._execute,
            auto_start=False,
        )
        self.server.start()
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

        rospy.loginfo("shared_autonomy_manager: new goal -> forwarding to move_base.")
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
            if self.server.is_preempt_requested():
                self.mb_client.cancel_goal()
                rospy.logwarn("shared_autonomy_manager: caller preempted the goal.")
                self.server.set_preempted()
                return

            takeover, done, resume, cancel = self._consume_flags()

            if takeover and state == AUTONOMOUS:
                state = TELEOP
                self.mb_client.cancel_goal()
                rospy.loginfo(
                    "shared_autonomy_manager: TAKEOVER -> move_base cancelled, "
                    "human in control. Waiting for 'done' or 'resume'."
                )

            if done and state == TELEOP:
                rospy.loginfo(
                    "shared_autonomy_manager: DONE -> reporting SUCCEEDED "
                    "(human-completed)."
                )
                self.server.set_succeeded(
                    MoveBaseResult(), "Goal completed by human teleoperation."
                )
                return

            if cancel and state == TELEOP:
                # Human ended the takeover without reaching the goal. Report
                # ABORTED so the caller does NOT treat it as goal-reached.
                rospy.loginfo(
                    "shared_autonomy_manager: CANCEL -> reporting ABORTED "
                    "(takeover ended without reaching the goal)."
                )
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
                goal.target_pose.header.stamp = rospy.Time.now()
                self.mb_client.send_goal(goal)
                seen_active = False
                rospy.loginfo(
                    "shared_autonomy_manager: RESUME -> re-sent goal, autonomy "
                    "driving from current pose to the goal."
                )

            if state == AUTONOMOUS:
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
                    result = self.mb_client.get_result() or MoveBaseResult()
                    self.server.set_aborted(
                        result, f"move_base terminated in state {mb_state}."
                    )
                    return

            rate.sleep()

        # rospy shutting down mid-goal.
        self.mb_client.cancel_goal()


def main() -> None:
    rospy.init_node("shared_autonomy_manager")
    SharedAutonomyManager()
    rospy.spin()


if __name__ == "__main__":
    main()
