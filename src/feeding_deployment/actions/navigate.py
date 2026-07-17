import json
import math
import os
import time
from collections import deque
from pathlib import Path
from typing import Any, Tuple

import yaml

try:
    import actionlib
    import rospy
    import tf2_ros
    from actionlib_msgs.msg import GoalStatus
    from geometry_msgs.msg import Twist
    from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool, Empty

    ROS_NAV_IMPORTED = True
except ModuleNotFoundError:
    ROS_NAV_IMPORTED = False

from relational_structs import (
    LiftedAtom,
    LiftedOperator,
    Object,
    Variable,
)
from feeding_deployment.actions.base import (
    HighLevelAction,
    nav_target_type,
    InFrontOf,
    DoorClosed,
    SafeToNavigate,
    GripperFree,
)
from feeding_deployment.interfaces.web_interface import WebInterfaceTakeoverInterrupt


class NavigateHLA(HighLevelAction):
    """Navigate from one target to another."""

    _VALID_TARGETS = ("fridge", "microwave", "sink", "table")

    # Frames used for the post-nav refinement window.
    _MAP_FRAME = "map"
    _BASE_FRAME = "vention_base_link"

    # Localization-stall watchdog. While driving autonomously, if the map->base
    # transform stops updating (e.g. Cartographer/VIO drops out and move_base
    # starts logging "Unable to get starting pose"), wait up to
    # _LOCALIZATION_STALL_TIMEOUT_S for it to recover before giving up. The timer
    # RESETS the instant localization is healthy again, so every dropout gets a
    # fresh window -- this is per-incident, not a cumulative whole-leg budget.
    _LOCALIZATION_STALL_TIMEOUT_S = 30.0   # stop if localization is gone this long
    _LOCALIZATION_STALE_AFTER_S = 1.0      # map->base older than this == dropped

    # Kitchen-egress staging. The kitchen is reached through a narrow corridor
    # the robot cannot turn around in. The microwave->table leg (navigate_to_
    # table) is therefore routed through an intermediate "open area" waypoint, so
    # TEB drives it as "reverse out to the open area, then turn and go" instead
    # of oscillating while trying to turn inside the corridor. The staging pose
    # lives in nav_named_locations.yaml; until a real pose is captured there
    # (placeholder: false) the via-waypoint is skipped and we go direct.
    _STAGING_WAYPOINT = "kitchen_exit"
    # Kitchen-ingress staging: the mirror of the egress, for the table->sink
    # leg (navigate_to_sink). Drive to the open area in front of the corridor
    # mouth first, turn there, then enter the corridor straight-on -- instead
    # of TEB turning at the corridor entrance. Same yaml/placeholder skip
    # rules as _STAGING_WAYPOINT.
    _INGRESS_WAYPOINT = "kitchen_enter"

    # On a failed leg (move_base aborted, or localization lost past the watchdog
    # window) we hand the base to the user via the navigation teleop screen and
    # re-try, up to this many times, before giving up (fatal).
    _MAX_RECOVERY_ATTEMPTS = 3

    # Post-arrival goal confirmation. move_base/TEB declares SUCCEEDED against the
    # map->base ESTIMATE, which Cartographer keeps correcting in discrete jumps for
    # a while after the base stops -- so the first "reached" can be on a stale
    # estimate that later snaps several cm off the true pose. After the first
    # SUCCEEDED we wait this long for localization to correct, then re-send the
    # SAME goal ONCE: TEB now sees the revealed gap (if any) and drives it out
    # against the corrected estimate. A single replan, not a loop. The subsequent
    # refinement window's before-residual records whether the second park truly
    # landed within tolerance. (Pure wait -- no shared clock with the refinement
    # timeout, so it cannot eat into the refinement driving budget.)
    # 25 -> 15 s: sized for Cartographer's correction cadence. At
    # optimize_every_n_nodes=90 a parked robot got a correction only every
    # ~45 s; at 25 (2026-07-06) it's every ~12 s, so 15 s guarantees at least
    # one full optimization epoch lands (10 s could just miss one).
    _GOAL_CONFIRM_SETTLE_S = 15.0

    # Learned per-location navigation offset (ParkingOffset BT parameter):
    # an SE(2) correction (dx m, dy m, dyaw rad) in the goal pose's LOCAL frame,
    # accumulated from the user's post-arrival teleop adjustments and applied to
    # the nominal goal on every navigation. Bounds must match the Box space in
    # the navigate_to_*.yaml behavior trees (and preference_bundle's
    # NAV_OFFSET_BOUNDS).
    _MAX_OFFSET_XY_M = 0.5
    _MAX_OFFSET_YAW_RAD = math.radians(45.0)
    # A post-arrival adjustment smaller than BOTH of these (measured on the
    # user's actual movement between the two settled pose reads, not on the
    # residual vs the goal) does not update the learned offset: TEB parks with
    # up to 5 cm / 2.9 deg scatter (xy/yaw_goal_tolerance) and folding that
    # noise into the offset would make it drift. The TF noise floor between two
    # settled reads is ~1 cm, so 2 cm / 2 deg separates intent from noise.
    _MIN_ADJUST_XY_M = 0.02
    _MIN_ADJUST_YAW_RAD = math.radians(2.0)
    # Localization settle before measuring the adjusted pose. Shorter than
    # _GOAL_CONFIRM_SETTLE_S (which absorbs move_base's stale-estimate
    # SUCCEEDED): here the base has been stationary since the user pressed
    # Done, matching the refinement window's settle scale.
    _ADJUST_SETTLE_S = 10.0

    # Defaults for the post-nav refinement window. Overridden at runtime by
    # config/nav/custom_param.yaml (section: refinement); see that file for the
    # meaning of each field. These are the fallbacks if the file is
    # missing/empty or omits a key.
    _REFINEMENT_DEFAULTS: dict[str, Any] = {
        "enabled": True,
        "actuate": True,
        "timeout_s": 5.0,
        "rate_hz": 10.0,
        "window_s": 1.0,
        "warmup_s": 1.0,
        "localization_settle_s": 5.0,
        "success_yaw_rad": None,    # None => half of move_base yaw_goal_tolerance
        "divergence_margin_rad": 0.02,
        "cmd_vel_topic": "/cmd_vel",
        "k_ang": 1.2,
        "max_ang_rps": 0.1667,  # == teleop max_vel_theta (shared_autonomy.launch)
    }

    # Hardcoded ("logged") navigation mode (run.py --logged-navigation /
    # FEEDING_LOGGED_NAV=1). Scripted open-loop-ish segments replace move_base
    # on specific kitchen legs, keyed by the true PDDL origin (execute_action
    # stashes it): fridge->microwave is a straight forward drive ending at the
    # normal confirmation/adjust page; the *->table legs first back out of the
    # kitchen and rotate, then continue with normal autonomous navigation.
    # Progress is measured from fused odometry, deliberately NOT map->base TF
    # -- Cartographer aliasing near the microwave is the reason this mode
    # exists. No fallback odom sources and no hold handling: anything
    # unexpected (odom missing/stale, timeout) stops the base and raises.
    _SCRIPTED_CMD_VEL_TOPIC = "/cmd_vel"   # autonomous stream (teleop-muted at the bridge)
    _SCRIPTED_RATE_HZ = 10.0               # match controller_frequency
    # Segment speeds are NOT constants: _scripted_speeds() reads max_vel_x /
    # max_vel_theta fresh from config/nav/teb_local_planner.yaml at every leg,
    # so the scripted motions always match the autonomous speed tier.
    _SCRIPTED_TIME_MARGIN = 1.5            # drive-time cap = nominal time * margin
    _SCRIPTED_ODOM_TOPIC = "/odometry/fused_imu_wheel"  # fused EKF (owns odom->base)
    _SCRIPTED_ODOM_FRESH_S = 1.0           # sample older than this == stale
    _SCRIPTED_ODOM_WAIT_S = 3.0            # wait for a first fresh sample, then fail
    _FRIDGE_TO_MICROWAVE_FWD_M = 1.4
    _EGRESS_ROTATE_RAD = math.pi / 2.0     # magnitude; driven CW (w < 0)
    # Kitchen-egress reverse distance before the rotate, by the leg's origin.
    _SCRIPTED_TABLE_EGRESS_REVERSE_M = {"microwave": 1.75, "fridge": 0.35}
    # After a mid-segment teleop abort, the leg falls back to autonomous
    # navigation once the teleop stream has been quiet this long (unless the
    # user pressed Done first -- then the park is final).
    _SCRIPTED_TELEOP_QUIET_S = 2.0
    # Straight-segment slip detector: wheel slip rotates the base while the
    # encoders read "straight"; the fused yaw (gyro-informed -- the EKF fuses
    # the debiased ZED gyro vyaw) sees the true rotation. Drift past the
    # threshold, persisting a few ticks, aborts the scripted leg -> autonomous
    # fallback to the leg's destination. Threshold sits ~3x above the
    # residual-gyro-bias + heading-wobble floor (~3-5 deg over a 28 s drive).
    # Rotate segments are exempt (slip there just slows the rotation; the
    # time cap covers it).
    _SCRIPTED_YAW_DRIFT_RAD = math.radians(15.0)
    _SCRIPTED_YAW_DRIFT_TICKS = 3   # consecutive 10 Hz ticks before triggering

    def _default_location_yaml(self) -> Path:
        return Path(__file__).resolve().parents[3] / "config" / "nav_named_locations.yaml"

    def _location_yaml(self) -> Path:
        user_path = os.environ.get("FEEDING_NAV_LOCATIONS_FILE", "").strip()
        if user_path:
            return Path(user_path).expanduser().resolve()
        return self._default_location_yaml()

    def _load_target_pose(self, location_name: str) -> dict[str, Any]:
        loc_file = self._location_yaml()
        if not loc_file.exists():
            raise FileNotFoundError(
                f"Named-location file not found at {loc_file}. "
                "Run capture_named_locations.py first."
            )

        with open(loc_file, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}

        locations = data.get("locations", {})
        if location_name not in locations:
            raise KeyError(
                f"Missing location '{location_name}' in {loc_file}. "
                f"Available locations: {sorted(locations)}"
            )

        target = locations[location_name]
        frame_id = target.get("frame_id", data.get("frame_id", "map"))
        position = target.get("position", {})
        orientation = target.get("orientation", {})
        return {
            "frame_id": frame_id,
            "x": float(position["x"]),
            "y": float(position["y"]),
            "z": float(position.get("z", 0.0)),
            "qx": float(orientation["x"]),
            "qy": float(orientation["y"]),
            "qz": float(orientation["z"]),
            "qw": float(orientation["w"]),
            "placeholder": bool(target.get("placeholder", False)),
        }

    def _get_tf_buffer(self) -> "tf2_ros.Buffer":
        """Persistent TF buffer/listener, kept warm across navigation legs."""
        if not getattr(self, "_tf_buffer", None):
            self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        return self._tf_buffer

    def _localization_fresh(self) -> bool:
        """True if map->base is still publishing recent transforms.

        Looks up the latest available map->base transform. If the chain
        (map->odom from Cartographer, odom->base from the fused wheel+IMU
        EKF) has stalled, the
        newest common timestamp falls behind now() and we report it stale. A
        failed lookup (no common transform at all) also counts as stale.
        """
        try:
            tf = self._get_tf_buffer().lookup_transform(
                self._MAP_FRAME, self._BASE_FRAME, rospy.Time(0)
            )
        except tf2_ros.TransformException:
            return False
        age_s = (rospy.Time.now() - tf.header.stamp).to_sec()
        return age_s <= self._LOCALIZATION_STALE_AFTER_S

    def _resolve_via(self, via: list) -> list:
        """Keep only the intermediate staging waypoints that are actually usable.

        A staging waypoint that is missing from nav_named_locations.yaml, or is
        still a placeholder, is dropped (with a note) so navigation safely falls
        back to driving direct to the destination.
        """
        usable = []
        for name in via:
            try:
                pose = self._load_target_pose(name)
            except (KeyError, FileNotFoundError):
                print(f"[nav] staging waypoint '{name}' not defined; going direct.")
                continue
            if pose.get("placeholder", False):
                print(
                    f"[nav] staging waypoint '{name}' is a placeholder "
                    f"(capture a real pose to enable it); going direct."
                )
                continue
            usable.append(name)
        return usable

    def _get_move_base_client(self):
        if not ROS_NAV_IMPORTED:
            raise RuntimeError("ROS navigation dependencies are not available")

        if not rospy.core.is_initialized():
            rospy.init_node(
                "feeding_deployment_navigate_hla",
                anonymous=True,
                disable_signals=True,
            )

        if not hasattr(self, "_move_base_client"):
            # Connect to the shared_autonomy_manager's "navigate" action server
            # (a transparent passthrough to move_base) rather than move_base
            # directly, so a human takeover can still report SUCCEEDED. Set the
            # FEEDING_NAV_ACTION env var to "move_base" to bypass the manager.
            nav_action = os.environ.get("FEEDING_NAV_ACTION", "navigate").strip()
            self._move_base_client = actionlib.SimpleActionClient(
                nav_action, MoveBaseAction
            )
            if not self._move_base_client.wait_for_server(rospy.Duration(15.0)):
                raise RuntimeError(
                    f"Timed out waiting for navigation action server '{nav_action}'"
                )
        return self._move_base_client

    def _ensure_teleop_subscribers(self) -> None:
        """Track whether a human teleop takeover is currently active.

        The navigation timeout below must count only AUTONOMOUS driving time --
        a human rescue can take arbitrarily long and must not trip it. The
        shared_autonomy_manager owns the AUTONOMOUS/TELEOP state machine; we
        mirror it here by watching the same takeover/resume intents the webapp
        and Xbox node publish. We also latch "done" directly (see
        _done_pressed_this_leg) so "the human parked it" never depends on the
        manager's state or the action's status text.
        """
        if getattr(self, "_teleop_subs_ready", False):
            return
        self._teleop_active = False
        # End-of-adjustment signal for the post-arrival fine-adjust flow (which
        # runs WITHOUT an active move_base goal): "done" = user parked it where
        # they want (measure), "resume" = user handed back without finishing
        # (abort, no update).
        self._adjust_end_reason = None
        # Reliable per-leg record of a Done press: set the instant
        # /shared_autonomy/done arrives (native ROS subscription -- no dependency
        # on the manager reaching TELEOP or on actionlib status-text propagation,
        # both of which can race/drop). Reset at the start of each drive leg;
        # consulted when the leg succeeds to decide "human parked it" -> skip the
        # confirm re-drive.
        self._done_pressed_this_leg = False
        rospy.Subscriber(
            "/shared_autonomy/takeover", Empty, self._on_teleop_takeover, queue_size=1
        )
        rospy.Subscriber(
            "/shared_autonomy/resume", Empty, self._on_teleop_resume, queue_size=1
        )
        rospy.Subscriber(
            "/shared_autonomy/done", Empty, self._on_teleop_done, queue_size=1
        )
        # We also PUBLISH takeover to start a recovery leg ourselves: re-send the
        # failed goal, then assert a takeover so the manager cancels move_base and
        # waits for the human. (The webapp asserts it too on the teleop screen.)
        self._takeover_pub = rospy.Publisher(
            "/shared_autonomy/takeover", Empty, queue_size=1
        )
        # Safety hold (/nav_safety_hold). DORMANT since 2026-07-15: the original
        # publisher (zed_health_monitor) was deleted with the IMU-only ZED sweep;
        # a Phase-3 interlock (e.g. map->odom yank channel) can republish it and
        # this hook re-engages unchanged. While held: the cmd_vel bridge stops
        # the wheels; here at the nav level we PAUSE (don't count a hold as a
        # localization failure) and, if move_base aborts during a hold, re-send
        # the goal on release -- "stop, wait, resume" instead of a failed leg.
        self._safety_hold = False
        self._last_hold_time = None
        rospy.Subscriber(
            "/nav_safety_hold", Bool, self._on_safety_hold, queue_size=5
        )
        self._teleop_subs_ready = True

    def _on_safety_hold(self, msg: "Bool") -> None:
        self._safety_hold = bool(msg.data)
        if self._safety_hold:
            self._last_hold_time = rospy.Time.now()

    def _wait_while_safety_hold(self) -> None:
        """Block while the ZED-divergence hold is asserted (robot already stopped
        at the motor level). Returns when the ZED recovers or on ROS shutdown."""
        if not self._safety_hold:
            return
        print("[nav] ZED-divergence hold active -- pausing, waiting for the ZED to "
              "recover before resuming navigation...")
        rate = rospy.Rate(5.0)
        while self._safety_hold and not rospy.is_shutdown():
            rate.sleep()
        if not rospy.is_shutdown():
            print("[nav] ZED recovered -- resuming navigation.")

    def _on_teleop_takeover(self, _msg: "Empty") -> None:
        self._teleop_active = True

    def _on_teleop_resume(self, _msg: "Empty") -> None:
        self._teleop_active = False
        self._adjust_end_reason = "resume"

    def _on_teleop_done(self, _msg: "Empty") -> None:
        # "done" = the human parked the base and is finished. Two consumers:
        #   - goal-driven navigation: the manager reports SUCCEEDED; we also latch
        #     _done_pressed_this_leg here so the "human parked it" decision never
        #     depends on the manager reaching TELEOP or on the action's status
        #     text (both can race/drop -- see _drive_to_pose's success branch).
        #   - goal-less post-arrival adjust flow: driven by _adjust_end_reason.
        self._teleop_active = False
        self._adjust_end_reason = "done"
        self._done_pressed_this_leg = True

    def _publish_base_takeover(self) -> None:
        """Assert a shared-autonomy takeover from the backend so the manager
        cancels move_base and waits for the human (starts a recovery leg)."""
        if getattr(self, "_takeover_pub", None) is None:
            return
        self._takeover_pub.publish(Empty())

    def _await_goal_active(self, client, timeout_s: float = 5.0) -> None:
        """Block until the freshly-sent goal goes ACTIVE, so a subsequent takeover
        is honored rather than cleared as a stale intent at the goal's start
        (the manager consumes pending intents when a new goal begins)."""
        deadline = rospy.Time.now() + rospy.Duration(timeout_s)
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if client.get_state() == GoalStatus.ACTIVE:
                return
            rate.sleep()

    def _prepare_for_navigation(self, speed: str, need_move_base: bool = True) -> None:
        """Common pre-drive setup for both autonomous and scripted legs.

        Tuck the arm into the left-back retract config before driving the base,
        so the arm is in a safe, compact pose for the whole navigation (all
        via waypoints + destination). No-op visualization in sim. Scripted legs
        pass need_move_base=False: they drive /cmd_vel directly and must not
        block on the "navigate" action server.
        """
        if self.robot_interface is not None:
            self.robot_interface.set_speed(speed)
        self.report_activity("Tucking the arm in before driving")
        print("Retracting arm to left_back_retract_pos before navigation ...")
        self.move_to_joint_positions(self.sim.scene_description.left_back_retract_pos)

        if not ROS_NAV_IMPORTED:
            raise RuntimeError(
                "ROS navigation modules not found. "
                "Please run in a ROS environment with move_base installed."
            )

        self._ensure_teleop_subscribers()
        if need_move_base:
            self._get_move_base_client()
        self._get_tf_buffer()  # warm the TF listener (used by the stall watchdog)

    def _navigate_to_target(
        self, location_name: str, speed: str, position_offset,
        arrival_confirm_mode, autocontinue_seconds, via: list = None,
    ) -> None:
        # if self.robot_interface is None:
        #     print(f"[SIM] Would navigate to {location_name} (speed={speed}).")
        #     return

        self._prepare_for_navigation(speed)

        # Drive any (usable) intermediate staging waypoints first, then the
        # destination. `via` is set only by the legs that need it -- e.g.
        # navigate_to_table routes through the kitchen-exit staging pose.
        waypoints = self._resolve_via(list(via or [])) + [location_name]
        if len(waypoints) > 1:
            print(f"[nav] routing {' -> '.join(waypoints)}")

        self.report_activity(f"Driving to the {location_name}")
        for i, wp in enumerate(waypoints):
            pose = self._load_target_pose(wp)
            is_final = i == len(waypoints) - 1
            if is_final:
                # The learned offset applies ONLY to the final destination;
                # staging/via poses are driven exactly as mapped. Keep the
                # nominal pose around: the post-arrival adjustment measures the
                # new TOTAL offset against it (final user pose vs nominal).
                nominal_pose = dict(pose)
                pose = self._apply_offset_to_pose(pose, position_offset)
            self._drive_to_pose(wp, pose)
            if is_final:
                # Final destination only -- staging poses just need to be reached
                # coarsely. move_base's first SUCCEEDED is against a map->base
                # estimate Cartographer is still correcting, so wait for it to
                # settle, then re-send the SAME goal ONCE: TEB drives out whatever
                # gap the correction revealed (a no-op if it was already there),
                # against the now-corrected estimate. Then fine-tune the heading.
                # Recovery use is accumulated across BOTH drives: the confirm
                # resend usually succeeds trivially (attempts == 0) and would
                # otherwise mask a recovery on the initial drive.
                # EXCEPTION: if the human parked the base and pressed Done, the
                # park is final -- confirm replan / refinement would drive the
                # base away from where the human deliberately put it.
                leg_used_recovery = getattr(self, "_last_leg_used_recovery", False)
                human_parked = getattr(self, "_last_leg_human_completed", False)
                if human_parked:
                    print(f"[nav] {wp}: human parked the base (Done) -- the "
                          "park is final; skipping confirm settle/replan, "
                          "refinement, and the adjust prompt.")
                else:
                    self._wait_for_localization_settle(
                        self._GOAL_CONFIRM_SETTLE_S,
                        "confirming the goal pose (replan once)",
                    )
                    self._drive_to_pose(wp, pose)
                    leg_used_recovery |= getattr(
                        self, "_last_leg_used_recovery", False)
                    human_parked = getattr(
                        self, "_last_leg_human_completed", False)
                self.report_activity(f"Arriving at the {location_name}")
                if not human_parked:
                    self._refinement_window(wp, pose)
                self._offer_position_adjustment(
                    wp, nominal_pose, pose, position_offset,
                    autocontinue_seconds=autocontinue_seconds,
                    leg_used_recovery=leg_used_recovery,
                    arrival_confirm_mode=arrival_confirm_mode,
                    human_parked=human_parked,
                )

    def _await_nav_result(self, client, location_name: str) -> str:
        """Block until the current navigation goal terminates, under the
        localization-stall watchdog. Returns "succeeded" or "failed".

        Raises RuntimeError only on ROS shutdown (an environment failure, not a
        recoverable navigation outcome). Both a move_base abort and a localization
        stall past the watchdog window return "failed" so the caller can hand the
        base to the user for manual recovery.

        Per-incident localization watchdog: move_base stays alive through a
        dropout and auto-resumes when TF returns, so we only give up if the
        dropout PERSISTS for the full window. The counter resets the instant
        localization is healthy again, and is paused entirely during a human
        takeover -- so an earlier struggle can never shorten a later incident's
        recovery window.
        """
        poll_s = 0.2
        stall_s = 0.0
        held_during_goal = False
        while True:
            if client.wait_for_result(rospy.Duration(poll_s)):
                break  # move_base/manager reached a terminal state
            if rospy.is_shutdown():
                client.cancel_goal()
                raise RuntimeError("ROS shutdown during navigation")
            if self._safety_hold:
                # ZED diverged: the base is stopped at the motor level and its
                # pose is garbage. Pause the failure watchdog -- this is an
                # intentional hold, not a lost-localization failure.
                held_during_goal = True
                stall_s = 0.0
                continue
            if self._teleop_active or self._localization_fresh():
                stall_s = 0.0  # human driving, or localization healthy -> reset
                continue
            stall_s += poll_s
            if stall_s >= self._LOCALIZATION_STALL_TIMEOUT_S:
                client.cancel_goal()
                print(
                    f"[nav] localization (map->{self._BASE_FRAME}) lost for "
                    f">{self._LOCALIZATION_STALL_TIMEOUT_S:.0f}s; treating "
                    f"{location_name} as a failed leg."
                )
                return "failed"

        state = client.get_state()
        if state == GoalStatus.SUCCEEDED:
            return "succeeded"
        # If move_base aborted because of / during a ZED-divergence hold (its
        # recovery behaviors fail while the wheels are held, so it eventually
        # gives up), don't treat it as a real failure -> the caller waits out the
        # hold and re-sends the same goal so navigation auto-resumes.
        if self._safety_hold or held_during_goal:
            print(f"[nav] move_base ended (state={state}) during a ZED-divergence "
                  f"hold for {location_name}; will resume when the ZED recovers.")
            return "held"
        print(f"[nav] move_base failed for {location_name}. Action state code={state}")
        return "failed"

    @staticmethod
    def _make_goal(pose: dict[str, Any]) -> "MoveBaseGoal":
        """Build a MoveBaseGoal from a pose dict (see _load_target_pose)."""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = str(pose["frame_id"])
        goal.target_pose.pose.position.x = pose["x"]
        goal.target_pose.pose.position.y = pose["y"]
        goal.target_pose.pose.position.z = pose["z"]
        goal.target_pose.pose.orientation.x = pose["qx"]
        goal.target_pose.pose.orientation.y = pose["qy"]
        goal.target_pose.pose.orientation.z = pose["qz"]
        goal.target_pose.pose.orientation.w = pose["qw"]
        return goal

    def _drive_to_pose(self, location_name: str, pose: dict[str, Any]) -> None:
        """Drive to one pose under the localization-stall watchdog, with manual
        teleop recovery on failure.

        On a failed leg (move_base aborted, or localization lost past the watchdog
        window) and when a web interface is available, route the iPad to the
        navigation teleop screen, re-send the goal, and assert a takeover so the
        manager cancels move_base and waits for the human. The user drives to the
        goal (Done -> blind SUCCEEDED) or hands back to autonomy (Resume).

        Bounded by _MAX_RECOVERY_ATTEMPTS. If recovery is unavailable (no web
        interface) or exhausted, raises RuntimeError -- the executive treats this
        as a fatal failure.
        """
        client = self._get_move_base_client()

        goal = self._make_goal(pose)

        print(
            f"Navigating to {location_name} using {self._location_yaml()} ...\n"
            f"  Localization-stall watchdog: stop only if map->{self._BASE_FRAME} "
            f"is lost for > {self._LOCALIZATION_STALL_TIMEOUT_S:.0f}s of autonomous "
            f"driving (resets the moment localization recovers)."
        )

        attempts = 0
        recovering = False
        held_resumes = 0
        # Re-send the goal after a ZED-divergence hold this many times before
        # falling through to human recovery -- guards against a persistent ZED
        # fault masquerading as a transient hold.
        max_held_resumes = 8
        # Whether THIS leg ended up needing the failure-recovery teleop. The
        # post-arrival adjustment step skips (and does not log) recovery-rescued
        # navigations -- a recovery park is "get past the failure", not a
        # position preference.
        self._last_leg_used_recovery = False
        # Whether THIS leg was completed by the human pressing Done (manager
        # reports a blind SUCCEEDED). The caller treats such a park as final:
        # no confirm replan, no refinement, no adjustment prompt.
        self._last_leg_human_completed = False
        # Clear any Done latched before this leg started; _on_teleop_done sets it
        # if the user presses Done at any point during this drive.
        self._done_pressed_this_leg = False
        while True:
            goal.target_pose.header.stamp = rospy.Time.now()
            client.send_goal(goal)
            # This leg starts under autonomy; enable the webapp's Robot Base
            # Control button so the user can take over mid-drive. The manager
            # honors /shared_autonomy/takeover (cancel move_base), /done (blind
            # SUCCEEDED) and /resume (replan from the current pose).
            self._teleop_active = False
            self._set_base_control_available(True)
            if recovering:
                # Recovery leg: hand to the human the instant the goal is ACTIVE,
                # before autonomy (which just failed) can drive again. Asserting
                # the takeover AFTER the goal is active means the manager honors
                # it instead of clearing it as a stale intent at goal start.
                self._await_goal_active(client)
                self._publish_base_takeover()
            try:
                outcome = self._await_nav_result(client, location_name)
            finally:
                self._set_base_control_available(False)

            if outcome == "succeeded":
                self._last_leg_used_recovery = attempts > 0
                # "Human parked it" -> skip the confirm re-drive/refinement/prompt.
                # Prefer the direct signal (did a /shared_autonomy/done land during
                # this leg?), which is robust to the takeover/done race and to
                # status-text propagation quirks that previously left this False
                # and triggered a spurious confirm re-drive. Fall back to the
                # manager's success text ("Goal completed by human teleoperation."
                # vs "move_base reached the goal."; a direct move_base client never
                # matches either).
                self._last_leg_human_completed = (
                    self._done_pressed_this_leg
                    or "human teleoperation" in (client.get_goal_status_text() or "")
                )
                print(f"Reached {location_name}."
                      + (" (human parked via Done)"
                         if self._last_leg_human_completed else ""))
                return

            # ZED-divergence hold: not a real failure. Wait for the ZED to
            # recover, then re-send the SAME goal so autonomy resumes -- without
            # spending a human-recovery attempt.
            if outcome == "held" and held_resumes < max_held_resumes:
                held_resumes += 1
                self._wait_while_safety_hold()
                if rospy.is_shutdown():
                    raise RuntimeError("ROS shutdown during ZED-divergence hold")
                print(f"[nav] resuming {location_name} after ZED recovery "
                      f"(resume {held_resumes}/{max_held_resumes}).")
                continue

            if self.web_interface is None or attempts >= self._MAX_RECOVERY_ATTEMPTS:
                raise RuntimeError(
                    f"move_base failed for {location_name} after "
                    f"{attempts} recovery attempt(s)."
                )

            attempts += 1
            recovering = True
            print(
                f"[nav] navigation to {location_name} failed; handing base "
                f"control to the user (recovery attempt "
                f"{attempts}/{self._MAX_RECOVERY_ATTEMPTS})."
            )
            # Route the iPad to the navigation teleop screen in skill mode
            # (Resume/Done). The loop re-sends the goal above and asserts takeover.
            self.web_interface._send_message(
                {"state": "navigation_teleop", "status": "recover"}
            )

    @staticmethod
    def _yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
        return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

    @staticmethod
    def _quat_from_yaw(yaw: float) -> tuple:
        return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

    @staticmethod
    def _se2_compose(x: float, y: float, yaw: float, dx: float, dy: float, dyaw: float) -> tuple:
        """(x,y,yaw) o (dx,dy,dyaw): apply a local-frame offset to a map-frame pose."""
        c, s = math.cos(yaw), math.sin(yaw)
        return x + dx * c - dy * s, y + dx * s + dy * c, NavigateHLA._wrap(yaw + dyaw)

    @staticmethod
    def _se2_relative(ax: float, ay: float, ayaw: float, bx: float, by: float, byaw: float) -> tuple:
        """a^-1 o b: pose b expressed in pose a's local frame."""
        c, s = math.cos(ayaw), math.sin(ayaw)
        ddx, ddy = bx - ax, by - ay
        return c * ddx + s * ddy, -s * ddx + c * ddy, NavigateHLA._wrap(byaw - ayaw)

    def _apply_offset_to_pose(self, pose: dict, offset) -> dict:
        """Compose the learned ParkingOffset onto a nominal goal pose.

        ``offset`` is the BT parameter value ([dx, dy, dyaw] in the goal's
        local frame) or None (per-user trees that predate the parameter).
        Components are clamped to the same bounds the BT Box space encodes.
        """
        if offset is None:
            return dict(pose)
        dx, dy, dyaw = (float(v) for v in offset)
        dx = self._clamp(dx, self._MAX_OFFSET_XY_M)
        dy = self._clamp(dy, self._MAX_OFFSET_XY_M)
        dyaw = self._clamp(dyaw, self._MAX_OFFSET_YAW_RAD)
        if dx == 0.0 and dy == 0.0 and dyaw == 0.0:
            return dict(pose)
        yaw = self._yaw_from_quat(pose["qx"], pose["qy"], pose["qz"], pose["qw"])
        nx, ny, nyaw = self._se2_compose(pose["x"], pose["y"], yaw, dx, dy, dyaw)
        qx, qy, qz, qw = self._quat_from_yaw(nyaw)
        out = dict(pose)
        out.update(x=nx, y=ny, qx=qx, qy=qy, qz=qz, qw=qw)
        print(
            f"[nav-offset] applying learned offset to goal: "
            f"dx={dx:+.3f}m dy={dy:+.3f}m dyaw={math.degrees(dyaw):+.1f}deg"
        )
        return out

    def _refinement_config_path(self) -> Path:
        return Path(__file__).resolve().parents[3] / "config" / "nav" / "custom_param.yaml"

    def _load_refinement_config(self) -> dict[str, Any]:
        """Overlay config/nav/custom_param.yaml (refinement section) onto the
        defaults. Missing/empty file or unknown keys degrade gracefully."""
        cfg = dict(self._REFINEMENT_DEFAULTS)
        path = self._refinement_config_path()
        try:
            if path.exists():
                with open(path, "r", encoding="utf-8") as f:
                    data = yaml.safe_load(f) or {}
                section = (data.get("refinement") or {}) if isinstance(data, dict) else {}
                for key, value in section.items():
                    if key in cfg:
                        cfg[key] = value
                    else:
                        print(f"[refinement] ignoring unknown key in {path.name}: {key}")
        except Exception as exc:  # noqa: BLE001 - config must never break nav
            print(f"[refinement] could not load {path}: {exc}; using defaults")
        return cfg

    @staticmethod
    def _wrap(angle: float) -> float:
        """Wrap an angle to [-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        return max(-limit, min(limit, value))

    def _wait_for_localization_settle(self, duration_s: float, reason: str) -> None:
        """Give map-frame localization time to settle before reading residual error."""
        if duration_s <= 0.0:
            return
        print(f"  Waiting {duration_s:.0f}s for localization to settle before {reason}...")
        deadline = rospy.Time.now() + rospy.Duration(duration_s)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            remaining_s = (deadline - rospy.Time.now()).to_sec()
            rospy.sleep(min(1.0, max(0.0, remaining_s)))

    def _refine_cmd(self, yaw: float, goal_yaw: float, cfg: dict) -> "Twist":
        """Rotate in place toward the goal pose's heading.

        Heading-only refinement: no matter how far off in xy move_base parked,
        we ALWAYS rotate toward the goal orientation and never translate. This
        base cannot creep position smoothly (the cmd_vel bridge floors linear
        commands), and turning to face the goal *point* would swing the base
        away from its final heading and make yaw worse -- that turn-to-face was
        the divergence seen on >6cm parks. So we accept move_base's position and
        refine heading alone -- the part this base refines smoothly.
        """
        cmd = Twist()
        final_yaw_err = self._wrap(goal_yaw - yaw)
        cmd.angular.z = self._clamp(
            float(cfg["k_ang"]) * final_yaw_err, float(cfg["max_ang_rps"])
        )
        return cmd

    def _refinement_window(self, location_name: str, pose: dict) -> None:
        """Refine the park pose for up to `timeout_s` after move_base succeeds.

        move_base/TEB only drives until it is within its goal tolerance, then
        stops — leaving the last few cm/deg on the table. When actuate=true this
        window rotates in place toward the goal *heading* (xy is left as
        move_base parked; see _refine_cmd) and stops on convergence, divergence
        (safety), or timeout. When actuate=false it is a passive monitor (old
        behavior). Tunables live in config/nav/custom_param.yaml.

        Hardware note: the cmd_vel bridge floors linear commands at ~0.31 m/s
        (min_lin_units/linear_scale), so fine *linear* nudges overshoot — which
        is why this refines heading only and never translates. Rotation has no
        floor (min_rot_units=0), so *heading* is what this base refines smoothly.
        """
        if not ROS_NAV_IMPORTED:
            return
        cfg = self._load_refinement_config()
        if not cfg["enabled"]:
            return

        # Success threshold (yaw only — we refine heading, not position):
        # explicit config value, else half the active planner's
        # yaw_goal_tolerance (stop once comfortably inside what the controller
        # required). TEB nests its tolerances under TebLocalPlannerROS, not at
        # the move_base root. No get_param default on purpose: a missing param
        # means move_base isn't configured as expected and we want to fail loudly
        # rather than refine to a wrong threshold. (Switching planners changes
        # this namespace, e.g. DWAPlannerROS.)
        yaw_tol = rospy.get_param("move_base/TebLocalPlannerROS/yaw_goal_tolerance")
        success_yaw_rad = (
            float(cfg["success_yaw_rad"]) if cfg["success_yaw_rad"] is not None else yaw_tol * 0.5
        )

        actuate = bool(cfg["actuate"])
        cmd_pub = (
            rospy.Publisher(str(cfg["cmd_vel_topic"]), Twist, queue_size=1)
            if actuate else None
        )

        tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        _listener = tf2_ros.TransformListener(tf_buffer)

        gx = pose["x"]
        gy = pose["y"]
        goal_yaw = self._yaw_from_quat(pose["qx"], pose["qy"], pose["qz"], pose["qw"])

        def _residual(prefix: str):
            """Print the signed map-frame pose error (current - goal) in x, y, yaw.

            Returns (dx, dy, dyaw) in meters/radians, or None if the TF lookup
            failed, so callers can build a before/after summary.
            """
            try:
                tfm = tf_buffer.lookup_transform(
                    self._MAP_FRAME, self._BASE_FRAME,
                    rospy.Time(0), rospy.Duration(1.0),
                )
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
                tf2_ros.TimeoutException,
            ) as exc:
                print(f"  {prefix}: TF lookup failed ({exc}).")
                return None
            tt = tfm.transform.translation
            qq = tfm.transform.rotation
            yy = self._yaw_from_quat(qq.x, qq.y, qq.z, qq.w)
            dx = tt.x - gx
            dy = tt.y - gy
            dyaw = self._wrap(yy - goal_yaw)
            print(
                f"  {prefix}: dx={dx * 100:+.1f}cm dy={dy * 100:+.1f}cm "
                f"dyaw={math.degrees(dyaw):+.2f}deg (|xy|={math.hypot(dx, dy) * 100:.1f}cm)"
            )
            return dx, dy, dyaw

        rate = rospy.Rate(float(cfg["rate_hz"]))
        if actuate:
            rospy.sleep(0.2)  # let the cmd_vel publisher connect before commanding
        window: deque = deque()  # (elapsed_s, err_xy, err_yaw)
        best_yaw = float("inf")

        mode = "actuating" if actuate else "monitoring"
        print(
            f"Refinement window: {mode} heading-only — rotating to goal heading "
            f"(success yaw<{math.degrees(success_yaw_rad):.2f}deg; "
            f"xy left as parked)..."
        )

        # Residual pose error the moment move_base declared the goal reached,
        # BEFORE any refinement driving.
        self._wait_for_localization_settle(
            float(cfg["localization_settle_s"]),
            "measuring pre-refinement residual error",
        )
        residual_before = _residual("Goal reached. Residual error vs goal")

        # Start the refinement clock AFTER the settle wait, so `timeout_s` budgets
        # only actual refinement driving -- otherwise the settle wait alone exceeds
        # the timeout and the loop bails before commanding the base.
        start = rospy.Time.now()

        try:
            while not rospy.is_shutdown():
                elapsed_s = (rospy.Time.now() - start).to_sec()
                if elapsed_s >= float(cfg["timeout_s"]):
                    print(f"\n  Refinement: timed out after {float(cfg['timeout_s']):.0f}s.")
                    break

                try:
                    tf = tf_buffer.lookup_transform(
                        self._MAP_FRAME, self._BASE_FRAME,
                        rospy.Time(0), rospy.Duration(0.1),
                    )
                except (
                    tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException,
                    tf2_ros.TimeoutException,
                ):
                    rate.sleep()
                    continue

                tr = tf.transform.translation
                q = tf.transform.rotation
                cur_yaw = self._yaw_from_quat(q.x, q.y, q.z, q.w)
                err_xy = math.hypot(tr.x - gx, tr.y - gy)
                err_yaw = abs(self._wrap(cur_yaw - goal_yaw))

                window.append((elapsed_s, err_xy, err_yaw))
                while window and elapsed_s - window[0][0] > float(cfg["window_s"]):
                    window.popleft()

                if len(window) < 3:
                    rate.sleep()
                    continue

                avg_xy = sum(w[1] for w in window) / len(window)
                avg_yaw = sum(w[2] for w in window) / len(window)

                best_yaw = min(best_yaw, avg_yaw)

                print(
                    f"\r  [{elapsed_s:4.1f}s] xy={avg_xy*100:.1f}cm (parked)  "
                    f"yaw={math.degrees(avg_yaw):.2f}deg "
                    f"(best {math.degrees(best_yaw):.2f})   ",
                    end="", flush=True,
                )

                # Heading-only: converge on yaw alone. xy is deliberately NOT a
                # gate -- this base can't refine position, so move_base's park is
                # accepted as-is and only reported (avg_xy above).
                if avg_yaw < success_yaw_rad:
                    print(
                        f"\n  Converged: yaw={math.degrees(avg_yaw):.2f}deg "
                        f"(xy={avg_xy*100:.1f}cm, accepted as parked)"
                    )
                    break

                # Divergence safety, on yaw only (the only axis we drive): stop
                # if the windowed heading error clearly worsens.
                if elapsed_s >= float(cfg["warmup_s"]):
                    if avg_yaw > best_yaw + float(cfg["divergence_margin_rad"]):
                        print(
                            f"\n  Stopped: yaw diverging — "
                            f"yaw={math.degrees(avg_yaw):.2f}deg vs best "
                            f"{math.degrees(best_yaw):.2f}deg"
                        )
                        break

                # Drive toward the exact goal using the instantaneous pose.
                if actuate and cmd_pub is not None:
                    cmd_pub.publish(self._refine_cmd(cur_yaw, goal_yaw, cfg))

                rate.sleep()
        finally:
            # Always leave the base stopped, whatever the exit path.
            if actuate and cmd_pub is not None:
                for _ in range(5):
                    cmd_pub.publish(Twist())
                    rospy.sleep(0.02)

        # Residual pose error AFTER the refinement window ended and the base has
        # stopped, so the before/after pair shows whether refinement helped.
        rospy.sleep(0.3)  # let the base settle before the final read
        residual_after = _residual("Refinement ended.  Residual error vs goal")

        # Consolidated before/after summary for this navigation, so the effect of
        # the refinement window is readable in one line instead of being scrolled
        # apart by the per-iteration status updates above.
        def _fmt(res) -> str:
            if res is None:
                return "unavailable"
            dx, dy, dyaw = res
            return (
                f"|xy|={math.hypot(dx, dy) * 100:.1f}cm "
                f"yaw={math.degrees(dyaw):+.2f}deg"
            )

        print(
            f"  Refinement summary: before [{_fmt(residual_before)}] "
            f"-> after [{_fmt(residual_after)}]"
        )

        # Persist the before/after residuals for this navigation to the per-user
        # log directory (log/<user>/nav_residual_log.jsonl), one JSON record per
        # navigation, so refinement quality can be reviewed offline.
        self._log_residual_to_file(location_name, residual_before, residual_after)

    @staticmethod
    def _residual_record(res) -> "dict[str, float] | None":
        """Convert a (dx, dy, dyaw) residual tuple into a JSON-friendly dict."""
        if res is None:
            return None
        dx, dy, dyaw = res
        return {
            "dx_m": dx,
            "dy_m": dy,
            "dyaw_rad": dyaw,
            "xy_m": math.hypot(dx, dy),
            "yaw_deg": math.degrees(dyaw),
        }

    def _log_residual_to_file(self, location_name: str, before, after) -> None:
        """Append one navigation's before/after residuals to the per-user log.

        Best-effort: a logging failure must never break navigation.
        """
        log_dir = getattr(self, "log_dir", None)
        if log_dir is None:
            return
        record = {
            "t": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "location": location_name,
            "before_refinement": self._residual_record(before),
            "after_refinement": self._residual_record(after),
        }
        try:
            log_dir = Path(log_dir)
            log_dir.mkdir(parents=True, exist_ok=True)
            with open(log_dir / "nav_residual_log.jsonl", "a", encoding="utf-8") as f:
                f.write(json.dumps(record) + "\n")
        except Exception as exc:  # noqa: BLE001 - logging must never break nav
            print(f"[nav] could not write residual log: {exc}")

    def _set_base_control_available(self, available: bool) -> None:
        """Tell the webapp whether the Robot Base Control button should be enabled."""
        if self.web_interface is None:
            return
        try:
            self.web_interface._send_message(
                {"state": "base_control", "status": "enabled" if available else "disabled"}
            )
        except Exception as e:
            print(f"Could not signal base-control availability: {e}")

    # ------------------------------------------------------------------ #
    # Scripted kitchen legs (logged-navigation mode)
    # ------------------------------------------------------------------ #
    @staticmethod
    def _logged_nav_enabled() -> bool:
        """Master switch for the hardcoded navigation mode (run.py
        --logged-navigation sets the env var; standalone harnesses can too)."""
        return os.environ.get("FEEDING_LOGGED_NAV", "").strip() == "1"

    def _scripted_speeds(self) -> "tuple[float, float]":
        """(linear m/s, angular rad/s) for scripted segments, read fresh from
        config/nav/teb_local_planner.yaml (max_vel_x / max_vel_theta) at every
        leg -- edit the TEB config and the scripted motions follow, no code
        change. Fail-loud on a missing file/key: driving at a guessed speed is
        exactly the silent drift this mode exists to avoid. (The nominal-time
        cap scales with the speed automatically.)"""
        path = (
            Path(__file__).resolve().parents[3]
            / "config" / "nav" / "teb_local_planner.yaml"
        )
        try:
            with open(path, "r", encoding="utf-8") as f:
                cfg = yaml.safe_load(f) or {}
            teb = cfg["TebLocalPlannerROS"]
            return float(teb["max_vel_x"]), float(teb["max_vel_theta"])
        except Exception as exc:
            raise RuntimeError(
                "cannot read scripted-segment speeds (max_vel_x / "
                f"max_vel_theta) from {path}: {exc}"
            ) from exc

    def _ensure_scripted_odom_subscriber(self) -> None:
        """Mirror the latest fused-odom pose (the segment progress source) and
        the teleop command stream (any nonzero /cmd_vel_teleop mid-segment is a
        human intervention, even if no takeover intent was asserted)."""
        if getattr(self, "_scripted_subs_ready", False):
            return
        self._scripted_odom = None      # (x, y, yaw, receipt time)
        self._last_teleop_cmd_t = None  # receipt time of the last NONZERO teleop cmd
        rospy.Subscriber(
            self._SCRIPTED_ODOM_TOPIC, Odometry, self._on_scripted_odom, queue_size=5
        )
        rospy.Subscriber(
            "/cmd_vel_teleop", Twist, self._on_teleop_cmd, queue_size=5
        )
        self._scripted_subs_ready = True

    def _on_scripted_odom(self, msg: "Odometry") -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._scripted_odom = (
            p.x, p.y, self._yaw_from_quat(q.x, q.y, q.z, q.w), rospy.get_time(),
        )

    def _on_teleop_cmd(self, msg: "Twist") -> None:
        if msg.linear.x != 0.0 or msg.angular.z != 0.0:
            self._last_teleop_cmd_t = rospy.get_time()

    def _fresh_odom_sample(self):
        """(x, y, yaw) of the latest fused-odom sample if younger than
        _SCRIPTED_ODOM_FRESH_S, else None."""
        sample = getattr(self, "_scripted_odom", None)
        if sample is None:
            return None
        x, y, yaw, recv_t = sample
        if rospy.get_time() - recv_t > self._SCRIPTED_ODOM_FRESH_S:
            return None
        return x, y, yaw

    def _get_scripted_cmd_pub(self):
        if getattr(self, "_scripted_cmd_pub", None) is None:
            self._scripted_cmd_pub = rospy.Publisher(
                self._SCRIPTED_CMD_VEL_TOPIC, Twist, queue_size=1
            )
            rospy.sleep(0.3)  # let connections settle before the first command
        return self._scripted_cmd_pub

    def _scripted_segment(
        self, purpose: str, label: str, v_mps: float, w_rps: float,
        dist_m: float = None, yaw_rad: float = None,
    ) -> dict:
        """Drive one scripted segment on /cmd_vel until fused odometry says the
        target distance/angle is reached.

        Fail-fast by design: no fallback odom sources, no waiting out holds.
        If fused odom is missing or goes stale, or the drive hits its time cap
        without reaching the target, the base is stopped and RuntimeError is
        raised (the executive's fatal path) -- a fixed-policy motion that isn't
        going as scripted is a problem a human should look at, not something to
        absorb. A human teleop command mid-segment ABORTS the segment (returns
        stop_reason="teleop"); it is never paused-and-resumed, because the
        remaining scripted distance is only meaningful from the assumed start
        pose. Likewise, yaw drifting past _SCRIPTED_YAW_DRIFT_RAD on a
        STRAIGHT segment means wheel slip rotated the base off its heading
        (the gyro-informed fused yaw sees what the encoders miss) -- the
        segment aborts with stop_reason="yaw_drift" so the caller can fall
        back to autonomous navigation. Zero twists are published on every
        exit path. The segment record is appended to scripted_nav_log.jsonl
        before any raise.

        FEEDING_LOGGED_NAV_DRY_RUN=1 runs the full loop and logging without
        publishing any motion.
        """
        pub = self._get_scripted_cmd_pub()
        dry_run = os.environ.get("FEEDING_LOGGED_NAV_DRY_RUN", "").strip() == "1"
        if dist_m is not None:
            kind, target = "dist", abs(dist_m)
            nominal_t = target / max(abs(v_mps), 1e-6)
        else:
            assert yaw_rad is not None, "segment needs dist_m or yaw_rad"
            kind, target = "yaw", abs(yaw_rad)
            nominal_t = target / max(abs(w_rps), 1e-6)
        time_cap = nominal_t * self._SCRIPTED_TIME_MARGIN

        record = {
            "t": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "purpose": purpose,
            "label": label,
            "cmd_v_mps": float(v_mps),
            "cmd_w_rps": float(w_rps),
            "target_kind": kind,
            "target": float(target),
            "nominal_s": round(nominal_t, 2),
            "time_cap_s": round(time_cap, 2),
            "dry_run": dry_run,
            "progress": None,
            "yaw_drift_rad": None,   # straight segments only; null for rotates
            "driven_s": None,
            "stop_reason": None,
            "odom_origin": None,
            "odom_final": None,
            "map_pose_before": self._se2_record(self._read_base_pose_se2()),
            "map_pose_after": None,
        }

        def _fail(reason: str, message: str) -> None:
            # Stop the base FIRST -- the TF read and file write below can take
            # up to ~1s and the wheels must not keep rolling through them.
            for _ in range(5):
                try:
                    if not dry_run:
                        pub.publish(Twist())
                    rospy.sleep(0.02)
                except Exception:
                    break
            record["stop_reason"] = reason
            record["map_pose_after"] = self._se2_record(self._read_base_pose_se2())
            self._log_scripted_event(record)
            print(f"[logged-nav] {label}: FAILED ({reason}) -- {message}")
            raise RuntimeError(f"scripted {label} segment failed: {message}")

        # Origin latch, fail-fast: without fresh fused odom there is no way to
        # measure progress and we refuse to drive blind.
        wait_deadline = rospy.get_time() + self._SCRIPTED_ODOM_WAIT_S
        origin = self._fresh_odom_sample()
        while origin is None and rospy.get_time() < wait_deadline and not rospy.is_shutdown():
            rospy.sleep(0.1)
            origin = self._fresh_odom_sample()
        if origin is None:
            _fail(
                "odom_unavailable",
                f"no fresh {self._SCRIPTED_ODOM_TOPIC} sample within "
                f"{self._SCRIPTED_ODOM_WAIT_S:.0f}s -- cannot measure progress",
            )
        record["odom_origin"] = self._se2_record(origin)

        cmd = Twist()
        cmd.linear.x = float(v_mps)
        cmd.angular.z = float(w_rps)
        rate = rospy.Rate(self._SCRIPTED_RATE_HZ)
        progress = 0.0
        yaw_drift_ticks = 0
        start_t = rospy.get_time()
        seg_start_t = start_t
        stop_reason = None
        print(
            f"[logged-nav] {label}: driving v={v_mps:+.3f} m/s w={w_rps:+.3f} rad/s "
            f"until {target:.2f} {'m' if kind == 'dist' else 'rad'} "
            f"(nominal {nominal_t:.1f}s, cap {time_cap:.1f}s"
            f"{', DRY RUN' if dry_run else ''})"
        )
        try:
            while not rospy.is_shutdown():
                now = rospy.get_time()
                # Human intervention: takeover intent (webapp/Xbox both publish
                # it) or any nonzero teleop command since this segment started.
                teleop_cmd_t = getattr(self, "_last_teleop_cmd_t", None)
                if self._teleop_active or (
                    teleop_cmd_t is not None and teleop_cmd_t >= seg_start_t
                ):
                    stop_reason = "teleop"
                    print(f"[logged-nav] {label}: human teleop detected -- "
                          "aborting the scripted segment.")
                    break
                sample = self._fresh_odom_sample()
                if sample is not None:
                    record["odom_final"] = self._se2_record(sample)
                    if kind == "dist":
                        progress = math.hypot(
                            sample[0] - origin[0], sample[1] - origin[1]
                        )
                        # Slip detector: the base should not rotate while
                        # driving straight. The fused yaw is gyro-informed, so
                        # it sees a slip-induced rotation the encoders miss.
                        drift = abs(self._wrap(sample[2] - origin[2]))
                        record["yaw_drift_rad"] = round(drift, 4)
                        if drift > self._SCRIPTED_YAW_DRIFT_RAD:
                            yaw_drift_ticks += 1
                        else:
                            yaw_drift_ticks = 0
                        if yaw_drift_ticks >= self._SCRIPTED_YAW_DRIFT_TICKS:
                            stop_reason = "yaw_drift"
                            print(
                                f"[logged-nav] {label}: yaw drifted "
                                f"{math.degrees(drift):.1f} deg while driving "
                                f"straight (wheel slip?) at {progress:.2f}/"
                                f"{target:.2f} m -- aborting the scripted leg "
                                "for autonomous navigation."
                            )
                            break
                    else:
                        progress = abs(self._wrap(sample[2] - origin[2]))
                    record["progress"] = round(progress, 4)
                    if progress >= target:
                        stop_reason = "target"
                        break
                else:
                    # Grace of 2x the freshness window before declaring the
                    # odom dead (a single dropped message must not abort a run).
                    odom = getattr(self, "_scripted_odom", None)
                    if odom is None or now - odom[3] > 2.0 * self._SCRIPTED_ODOM_FRESH_S:
                        record["driven_s"] = round(now - start_t, 2)
                        _fail(
                            "odom_stale",
                            f"{self._SCRIPTED_ODOM_TOPIC} went stale mid-segment "
                            f"after {progress:.2f}/{target:.2f} "
                            f"{'m' if kind == 'dist' else 'rad'}",
                        )
                if now - start_t >= time_cap:
                    record["driven_s"] = round(now - start_t, 2)
                    _fail(
                        "time_cap",
                        f"hit the {time_cap:.1f}s cap at {progress:.2f}/{target:.2f} "
                        f"{'m' if kind == 'dist' else 'rad'} -- wheels stalled or "
                        "odometry not tracking",
                    )
                if not dry_run:
                    pub.publish(cmd)
                rate.sleep()
            if rospy.is_shutdown() and stop_reason is None:
                stop_reason = "shutdown"
        finally:
            # Zero twists on EVERY exit path (target, teleop, failure raise,
            # shutdown, unexpected exception). publish() can raise after
            # shutdown -> best-effort.
            for _ in range(5):
                try:
                    if not dry_run:
                        pub.publish(Twist())
                    rospy.sleep(0.02)
                except Exception:
                    break

        record["driven_s"] = round(rospy.get_time() - start_t, 2)
        record["stop_reason"] = stop_reason
        record["map_pose_after"] = self._se2_record(self._read_base_pose_se2())
        self._log_scripted_event(record)
        if stop_reason == "shutdown":
            raise RuntimeError(f"ROS shutdown during the scripted {label} segment")
        print(
            f"[logged-nav] {label}: {stop_reason} at "
            f"{progress:.2f}/{target:.2f} {'m' if kind == 'dist' else 'rad'} "
            f"in {record['driven_s']:.1f}s"
        )
        return record

    def _run_scripted_motion(self, purpose: str, segments: list) -> str:
        """Run scripted segments in order. Returns "completed", "human_done"
        (the user teleoped in and pressed Done -- the park is final),
        "teleop_fallback" (the user intervened and handed back), or
        "slip_fallback" (yaw drifted on a straight segment -- wheel slip).
        On either fallback the caller must fall back to autonomous
        navigation. Segment failures raise."""
        self._ensure_scripted_odom_subscriber()
        self._done_pressed_this_leg = False
        self._set_base_control_available(True)
        try:
            for i, seg in enumerate(segments):
                rec = self._scripted_segment(purpose, **seg)
                if rec["stop_reason"] == "teleop":
                    return self._await_teleop_resolution(purpose)
                if rec["stop_reason"] == "yaw_drift":
                    # Base already stopped; nobody is driving -- no teleop
                    # resolution to wait for. Remaining segments are void.
                    return "slip_fallback"
                if i < len(segments) - 1:
                    rospy.sleep(0.5)  # settle between direction changes
        finally:
            self._set_base_control_available(False)
        return "completed"

    def _await_teleop_resolution(self, purpose: str) -> str:
        """After a mid-segment teleop abort, wait for the human to finish:
        Done -> the park is final; Resume (or the teleop stream quiet for
        _SCRIPTED_TELEOP_QUIET_S with no takeover asserted) -> hand back for
        an autonomous fallback."""
        print(
            f"[logged-nav] {purpose}: waiting for the human to finish "
            "(Done = park is final; Resume / release = autonomous fallback)..."
        )
        rate = rospy.Rate(5.0)
        while not rospy.is_shutdown():
            if self._done_pressed_this_leg:
                print(f"[logged-nav] {purpose}: human pressed Done -- park is final.")
                return "human_done"
            teleop_cmd_t = getattr(self, "_last_teleop_cmd_t", None)
            quiet = (
                teleop_cmd_t is None
                or rospy.get_time() - teleop_cmd_t >= self._SCRIPTED_TELEOP_QUIET_S
            )
            if not self._teleop_active and quiet:
                print(f"[logged-nav] {purpose}: teleop released -- "
                      "falling back to autonomous navigation.")
                return "teleop_fallback"
            rate.sleep()
        raise RuntimeError(f"ROS shutdown while waiting out a teleop takeover ({purpose})")

    def _log_scripted_event(self, record: dict) -> None:
        """Append one scripted-segment record to the per-user log
        (log/<user>/scripted_nav_log.jsonl). Best-effort: a logging failure
        must never break navigation."""
        log_dir = getattr(self, "log_dir", None)
        if log_dir is None:
            return
        try:
            log_dir = Path(log_dir)
            log_dir.mkdir(parents=True, exist_ok=True)
            with open(log_dir / "scripted_nav_log.jsonl", "a", encoding="utf-8") as f:
                f.write(json.dumps(record) + "\n")
        except Exception as exc:  # noqa: BLE001 - logging must never break nav
            print(f"[logged-nav] could not write scripted log: {exc}")

    def _hardcoded_microwave_approach(
        self, speed: str, position_offset, arrival_confirm_mode,
        autocontinue_seconds,
    ) -> None:
        """Logged-nav fridge->microwave: a scripted forward drive instead of
        move_base. Skips the goal-confirm settle/replan and the refinement
        window (both are map-frame closed-loop -- meaningless under the
        aliasing this mode sidesteps) and goes straight to the confirmation/
        adjustment page. The learned offset is NOT applied to the motion
        (commanded == nominal); the adjust flow still measures, logs, and
        updates the learned offset exactly as today."""
        self._prepare_for_navigation(speed, need_move_base=False)
        v_mps, _ = self._scripted_speeds()
        self.report_activity("Driving to the microwave")
        print(
            "[logged-nav] fridge->microwave: scripted "
            f"{self._FRIDGE_TO_MICROWAVE_FWD_M:.2f} m forward drive at "
            f"{v_mps:.3f} m/s (learned offset not applied to the motion)."
        )
        outcome = self._run_scripted_motion("fridge_to_microwave", [
            {"label": "forward", "v_mps": v_mps, "w_rps": 0.0,
             "dist_m": self._FRIDGE_TO_MICROWAVE_FWD_M},
        ])
        if outcome in ("teleop_fallback", "slip_fallback"):
            # The scripted assumption is void -- either the human moved the
            # base mid-drive, or wheel slip rotated it off its heading. Drive
            # the leg autonomously to the microwave pose (offset applies as on
            # any autonomous leg, full arrival flow); move_base corrects the
            # heading closed-loop from wherever the base actually is.
            trigger = "human teleop" if outcome == "teleop_fallback" else "wheel slip"
            print(f"[logged-nav] microwave: {trigger} voided the scripted "
                  "drive -- continuing with autonomous navigation.")
            self._navigate_to_target(
                "microwave", speed, position_offset=position_offset,
                arrival_confirm_mode=arrival_confirm_mode,
                autocontinue_seconds=autocontinue_seconds,
            )
            return
        self.report_activity("Arriving at the microwave")
        nominal_pose = self._load_target_pose("microwave")
        self._offer_position_adjustment(
            "microwave", nominal_pose, dict(nominal_pose), position_offset,
            leg_used_recovery=False,
            arrival_confirm_mode=arrival_confirm_mode,
            autocontinue_seconds=autocontinue_seconds,
            human_parked=(outcome == "human_done"),
        )

    # ------------------------------------------------------------------ #
    # Post-arrival position adjustment (learned per-location nav offset)
    # ------------------------------------------------------------------ #
    def _read_base_pose_se2(self):
        """(x, y, yaw) of map->base from TF, or None if localization is stale.

        Closed-loop on localization (the same Cartographer/VIO chain the
        refinement window trusts) -- teleop cmd_vel is never integrated.
        """
        if not self._localization_fresh():
            return None
        try:
            tf = self._get_tf_buffer().lookup_transform(
                self._MAP_FRAME, self._BASE_FRAME, rospy.Time(0), rospy.Duration(1.0)
            )
        except tf2_ros.TransformException:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        return t.x, t.y, self._yaw_from_quat(q.x, q.y, q.z, q.w)

    @staticmethod
    def _pose_dict_se2(pose: dict[str, Any]) -> tuple:
        """(x, y, yaw) of a _load_target_pose-style dict."""
        return (
            pose["x"],
            pose["y"],
            NavigateHLA._yaw_from_quat(pose["qx"], pose["qy"], pose["qz"], pose["qw"]),
        )

    def _offer_position_adjustment(
        self,
        location_name: str,
        nominal_pose: dict[str, Any],
        commanded_pose: dict[str, Any],
        prev_offset,
        autocontinue_seconds,
        leg_used_recovery: bool = False,
        arrival_confirm_mode=None,
        human_parked: bool = False,
    ) -> None:
        """After arrival at a real destination, let the user teleop the base to
        fine-adjust its position, and fold the adjustment into the learned
        ParkingOffset for this location.

        The new TOTAL offset is the user's final localized pose expressed in
        the NOMINAL (mapped) goal's local frame -- this accumulates previous
        corrections by construction and does not double-count nav noise the
        user just drove out. Best-effort throughout: a failure here must never
        break the navigation that already succeeded.

        Deliberately NOT gated on no_waits: on-robot runs pass --no_waits (to
        use the NullSimulator instead of PyBullet), and the adjustment prompt
        is a real-robot interaction that must still fire there. Only a missing
        web interface skips it.
        """
        if self.web_interface is None:
            return
        if location_name not in self._VALID_TARGETS:
            return
        # confirm_navigation_arrival preference (ArrivalConfirmMode BT
        # param): 0 = page off (parking offsets stop being refined), 1 = page
        # with autocontinue, 2 = page waits indefinitely. None (per-user YAML
        # predating the param) keeps today's behavior (mode 1).
        mode = 1 if arrival_confirm_mode is None else int(arrival_confirm_mode)
        if mode == 0:
            print(
                f"[nav-offset] {location_name}: arrival confirmation disabled "
                "by preference; keeping the learned offset as-is."
            )
            return
        if leg_used_recovery:
            # User decision: a recovery-teleop rescue is "get past the
            # failure", not a position preference. No prompt, no dataset row.
            print(
                f"[nav-offset] {location_name}: final leg used recovery teleop; "
                "skipping the position-adjustment prompt."
            )
            return
        if human_parked:
            # The user drove the base here and pressed Done -- the park IS the
            # confirmation. No prompt, no dataset row (park residual is not a
            # measured adjustment; same reasoning as the recovery branch above).
            print(
                f"[nav-offset] {location_name}: human parked via Done; "
                "skipping the position-adjustment prompt."
            )
            return

        pose_before = self._read_base_pose_se2()
        if pose_before is None:
            print(
                f"[nav-offset] {location_name}: localization stale before the "
                "adjustment prompt; cannot measure -- skipping."
            )
            return

        try:
            # Mode 2 ("wait for me"): autocontinue_seconds <= 0 tells the page
            # to show no countdown and wait for an explicit answer. In mode 1
            # the countdown is the skill's ArrivalConfirmAutocontinueSeconds BT
            # parameter.
            autocontinue_s = 0.0 if mode == 2 else float(autocontinue_seconds)
            choice = self.web_interface.get_nav_position_adjust_choice(
                location_name, autocontinue_s
            )
        except WebInterfaceTakeoverInterrupt:
            # A mid-skill arm takeover during the prompt wait must reach the
            # HLA boundary (execute_action), which owns the takeover flow.
            raise
        except Exception as e:
            print(f"[nav-offset] adjustment prompt failed ({e}); skipping.")
            return

        nominal_se2 = self._pose_dict_se2(nominal_pose)
        commanded_se2 = self._pose_dict_se2(commanded_pose)

        if choice != "adjust":
            self._log_offset_event(
                location_name, "declined",
                nominal_se2, commanded_se2, pose_before, None, prev_offset,
            )
            return

        # Hand the base to the user WITHOUT re-sending a move_base goal: the
        # robot is already within tolerance of the commanded pose, so a fresh
        # goal can be declared SUCCEEDED before a takeover lands and the user
        # would never get to drive. The webapp joystick publishes /cmd_vel
        # directly (no active goal needed); we wait on the teleop screen's
        # /shared_autonomy/done ("parked where I want it" -> measure) or
        # /shared_autonomy/resume ("hand back without finishing" -> abort, no
        # update). Both routes return the iPad to robot_executing; the
        # manager's stale takeover/done flags are cleared at the next goal
        # start, so they are harmless without an active goal.
        print(f"[nav-offset] {location_name}: handing base to the user for fine adjustment.")
        self._adjust_end_reason = None
        self._set_base_control_available(True)
        try:
            self.web_interface._send_message(
                {"state": "navigation_teleop", "status": "recover"}
            )
            while not rospy.is_shutdown() and self._adjust_end_reason is None:
                rospy.sleep(0.2)
        finally:
            self._set_base_control_available(False)

        if self._adjust_end_reason != "done":
            print(
                f"[nav-offset] {location_name}: adjustment ended via "
                f"{self._adjust_end_reason or 'shutdown'}; not updating."
            )
            self._log_offset_event(
                location_name, "aborted",
                nominal_se2, commanded_se2, pose_before, None, prev_offset,
            )
            return

        self._wait_for_localization_settle(
            self._ADJUST_SETTLE_S, "measuring the adjusted pose"
        )
        pose_after = self._read_base_pose_se2()
        if pose_after is None:
            print(
                f"[nav-offset] {location_name}: localization stale after the "
                "adjustment; cannot measure -- not updating."
            )
            self._log_offset_event(
                location_name, "localization_stale",
                nominal_se2, commanded_se2, pose_before, None, prev_offset,
            )
            return

        # Pure user motion between the two settled reads gates the update;
        # the TOTAL offset is measured against the nominal (mapped) goal.
        movement = self._se2_relative(*pose_before, *pose_after)
        total = self._se2_relative(*nominal_se2, *pose_after)

        if (
            math.hypot(movement[0], movement[1]) < self._MIN_ADJUST_XY_M
            and abs(movement[2]) < self._MIN_ADJUST_YAW_RAD
        ):
            print(
                f"[nav-offset] {location_name}: adjustment below the "
                f"{self._MIN_ADJUST_XY_M * 100:.0f}cm/"
                f"{math.degrees(self._MIN_ADJUST_YAW_RAD):.0f}deg threshold; not updating."
            )
            self._log_offset_event(
                location_name, "below_threshold",
                nominal_se2, commanded_se2, pose_before, pose_after, prev_offset,
            )
            return

        clamped = [
            self._clamp(total[0], self._MAX_OFFSET_XY_M),
            self._clamp(total[1], self._MAX_OFFSET_XY_M),
            self._clamp(total[2], self._MAX_OFFSET_YAW_RAD),
        ]
        if clamped != list(total):
            print(
                f"[nav-offset] {location_name}: total offset saturated at the "
                f"+/-{self._MAX_OFFSET_XY_M}m / +/-{math.degrees(self._MAX_OFFSET_YAW_RAD):.0f}deg "
                "bounds; consider re-capturing the named location instead."
            )

        # Persist the new TOTAL into this navigation's BT YAML (the same
        # write-back mechanism the plate pickups use for corrected colors).
        # The preference session reads it back after the HLA completes.
        objects = (
            Object(location_name, nav_target_type),
            Object(location_name, nav_target_type),
        )
        node_name = f"NavigateTo{location_name.capitalize()}"
        result = self.process_behavior_tree_parameter_update(
            objects, {}, node_name, "ParkingOffset",
            [float(clamped[0]), float(clamped[1]), float(clamped[2])],
        )
        print(
            f"[nav-offset] {location_name}: user moved "
            f"dx={movement[0] * 100:+.1f}cm dy={movement[1] * 100:+.1f}cm "
            f"dyaw={math.degrees(movement[2]):+.1f}deg; new total offset "
            f"[{clamped[0]:+.3f}, {clamped[1]:+.3f}, {clamped[2]:+.3f}]. BT update: {result}"
        )
        if not str(result).startswith("Success"):
            # The measurement happened but the persist failed (e.g. stale
            # per-user tree without the param and no upsert path here) -- log
            # it as such so the dataset never claims an update that isn't in
            # the YAML.
            self._log_offset_event(
                location_name, "bt_write_failed",
                nominal_se2, commanded_se2, pose_before, pose_after, prev_offset,
                total_offset=clamped,
            )
            return
        self._log_offset_event(
            location_name, "updated",
            nominal_se2, commanded_se2, pose_before, pose_after, prev_offset,
            total_offset=clamped,
        )

    @staticmethod
    def _se2_record(se2) -> "dict[str, float] | None":
        if se2 is None:
            return None
        x, y, yaw = se2
        return {"x": float(x), "y": float(y), "yaw": float(yaw)}

    def _log_offset_event(
        self,
        location_name: str,
        outcome: str,
        nominal_se2,
        commanded_se2,
        pose_before,
        pose_after,
        prev_offset,
        total_offset=None,
    ) -> None:
        """Append one position-adjustment event to the per-user log
        (log/<user>/nav_offset_log.jsonl). Best-effort: a logging failure must
        never break navigation."""
        log_dir = getattr(self, "log_dir", None)
        if log_dir is None:
            return
        delta = None
        movement = None
        if pose_after is not None:
            delta = list(self._se2_relative(*commanded_se2, *pose_after))
            movement = list(self._se2_relative(*pose_before, *pose_after))
        record = {
            "t": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "location": location_name,
            "outcome": outcome,
            "nominal_pose": self._se2_record(nominal_se2),
            "commanded_pose": self._se2_record(commanded_se2),
            "pose_before_adjust": self._se2_record(pose_before),
            "pose_after_adjust": self._se2_record(pose_after),
            "previous_offset": [float(v) for v in prev_offset] if prev_offset is not None else None,
            "delta": delta,
            "user_movement": movement,
            "total_offset": [float(v) for v in total_offset] if total_offset is not None else None,
        }
        try:
            log_dir = Path(log_dir)
            log_dir.mkdir(parents=True, exist_ok=True)
            with open(log_dir / "nav_offset_log.jsonl", "a", encoding="utf-8") as f:
                f.write(json.dumps(record) + "\n")
        except Exception as exc:  # noqa: BLE001 - logging must never break nav
            print(f"[nav-offset] could not write offset log: {exc}")

    def get_name(self) -> str:
        return "Navigate"

    def get_operator(self) -> LiftedOperator:
        src = Variable("?from", nav_target_type)
        dst = Variable("?to", nav_target_type)

        return LiftedOperator(
            self.get_name(),
            parameters=[src, dst],
            preconditions={
                LiftedAtom(InFrontOf, [src]),
                LiftedAtom(SafeToNavigate, []),
                LiftedAtom(GripperFree, []),
            },
            add_effects={
                LiftedAtom(InFrontOf, [dst]),
            },
            delete_effects={
                LiftedAtom(InFrontOf, [src]),
            },
        )

    def get_behavior_tree_filename(
        self,
        objects: Tuple[Object, ...],
        params: dict[str, Any],
    ) -> str:
        del params
        assert len(objects) == 2
        _, dst = objects
        assert self.sim.scene_description.scene_label == "vention"
        assert dst.name in self._VALID_TARGETS
        return f"navigate_to_{dst.name}.yaml"

    def execute_action(
        self,
        objects: Tuple[Object, ...],
        params: dict[str, Any],
    ) -> None:
        # Stash the true PDDL origin (?from) for the duration of the skill: the
        # behavior tree only carries the destination, but the logged-nav
        # scripted legs are origin-specific (fridge->microwave vs a table->
        # microwave re-heat leg must not run the same hardcoded motion). Not a
        # BT parameter on purpose -- per-user trees are copies seeded at first
        # run and never receive new parameters.
        self._nav_origin = objects[0].name if len(objects) == 2 else None
        try:
            super().execute_action(objects, params)
        finally:
            self._nav_origin = None

    def navigate_to_fridge(self, speed: str, position_offset, arrival_confirm_mode, autocontinue_seconds) -> None:
        self._navigate_to_target(
            "fridge", speed, position_offset=position_offset,
            arrival_confirm_mode=arrival_confirm_mode,
            autocontinue_seconds=autocontinue_seconds,
        )

    def navigate_to_microwave(self, speed: str, position_offset, arrival_confirm_mode, autocontinue_seconds) -> None:
        # Logged-nav mode, fridge->microwave only: scripted forward drive. Any
        # other origin (table->microwave re-heat leg, unknown after a resume)
        # stays fully autonomous.
        if self._logged_nav_enabled():
            if getattr(self, "_nav_origin", None) == "fridge":
                self._hardcoded_microwave_approach(
                    speed, position_offset, arrival_confirm_mode,
                    autocontinue_seconds=autocontinue_seconds,
                )
                return
            print(
                "[logged-nav] microwave: origin is "
                f"{getattr(self, '_nav_origin', None)!r}, not 'fridge' -- "
                "using normal autonomous navigation."
            )
        self._navigate_to_target(
            "microwave", speed, position_offset=position_offset,
            arrival_confirm_mode=arrival_confirm_mode,
            autocontinue_seconds=autocontinue_seconds,
        )

    def navigate_to_sink(self, speed: str, position_offset, arrival_confirm_mode, autocontinue_seconds) -> None:
        # table -> sink is the kitchen ingress: cross the open area to the
        # staging pose at the corridor mouth, turn there, then drive straight
        # into the corridor -- the mirror of the microwave->table egress via
        # kitchen_exit. (Auto-skipped while the staging pose is missing or a
        # placeholder.)
        input("Is the FT sensor wire tucked in? Press Enter to continue...")
        self._navigate_to_target(
            "sink", speed, via=[self._INGRESS_WAYPOINT], position_offset=position_offset,
            arrival_confirm_mode=arrival_confirm_mode,
            autocontinue_seconds=autocontinue_seconds,
        )

    def navigate_to_table(self, speed: str, position_offset, arrival_confirm_mode, autocontinue_seconds) -> None:
        # microwave -> table is the kitchen egress: reverse out through the narrow
        # corridor to the open staging area, then turn and drive to the table.
        # Routing via the staging waypoint stops TEB from oscillating as it tries
        # to turn inside the corridor. (Auto-skipped while the staging pose is an
        # unset placeholder.)
        #
        # Logged-nav mode: the egress is scripted instead -- reverse a fixed
        # origin-specific distance, rotate 90 deg CW, then drive autonomously
        # DIRECT to the table (the scripted egress replaces the staging
        # waypoint). Unknown origins keep the fully autonomous route.
        origin = getattr(self, "_nav_origin", None)
        reverse_m = self._SCRIPTED_TABLE_EGRESS_REVERSE_M.get(origin)
        if self._logged_nav_enabled() and reverse_m is not None:
            self._prepare_for_navigation(speed, need_move_base=False)
            v_mps, w_rps = self._scripted_speeds()
            self.report_activity("Backing out of the kitchen")
            print(
                f"[logged-nav] {origin}->table: scripted egress "
                f"(reverse {reverse_m:.2f} m at {v_mps:.3f} m/s, rotate 90 deg "
                f"CW at {w_rps:.3f} rad/s), then autonomous navigation to the "
                "table."
            )
            outcome = self._run_scripted_motion(f"{origin}_to_table_egress", [
                {"label": "reverse", "v_mps": -v_mps,
                 "w_rps": 0.0, "dist_m": reverse_m},
                {"label": "rotate_right", "v_mps": 0.0,
                 "w_rps": -w_rps,
                 "yaw_rad": self._EGRESS_ROTATE_RAD},
            ])
            if outcome == "human_done":
                # System-wide Done contract: the human parked the base and the
                # park is final -- the leg ends here.
                print("[logged-nav] table: human parked via Done; leg ends.")
                return
            # completed: the egress replaced kitchen_exit -> go direct.
            # teleop_fallback / slip_fallback: the egress was interrupted
            # (human intervention or wheel slip) and the base may still be
            # inside the corridor -> full normal route incl. staging.
            via = [] if outcome == "completed" else [self._STAGING_WAYPOINT]
            self._navigate_to_target(
                "table", speed, via=via, position_offset=position_offset,
                arrival_confirm_mode=arrival_confirm_mode,
                autocontinue_seconds=autocontinue_seconds,
            )
            return
        if self._logged_nav_enabled():
            print(
                f"[logged-nav] table: origin is {origin!r} (no scripted egress "
                "defined) -- using normal autonomous navigation."
            )
        self._navigate_to_target(
            "table", speed, via=[self._STAGING_WAYPOINT], position_offset=position_offset,
            arrival_confirm_mode=arrival_confirm_mode,
            autocontinue_seconds=autocontinue_seconds,
        )
