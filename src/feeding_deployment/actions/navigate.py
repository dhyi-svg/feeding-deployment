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
    from std_msgs.msg import Empty

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
    _GOAL_CONFIRM_SETTLE_S = 25.0

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
        "max_ang_rps": 0.4,
    }

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
        (map->odom from Cartographer, odom->base from VIO) has stalled, the
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
        and Xbox node publish. (done ends the action, so it needs no handling.)
        """
        if getattr(self, "_teleop_subs_ready", False):
            return
        self._teleop_active = False
        rospy.Subscriber(
            "/shared_autonomy/takeover", Empty, self._on_teleop_takeover, queue_size=1
        )
        rospy.Subscriber(
            "/shared_autonomy/resume", Empty, self._on_teleop_resume, queue_size=1
        )
        # We also PUBLISH takeover to start a recovery leg ourselves: re-send the
        # failed goal, then assert a takeover so the manager cancels move_base and
        # waits for the human. (The webapp asserts it too on the teleop screen.)
        self._takeover_pub = rospy.Publisher(
            "/shared_autonomy/takeover", Empty, queue_size=1
        )
        self._teleop_subs_ready = True

    def _on_teleop_takeover(self, _msg: "Empty") -> None:
        self._teleop_active = True

    def _on_teleop_resume(self, _msg: "Empty") -> None:
        self._teleop_active = False

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

    def _navigate_to_target(
        self, location_name: str, speed: str, via: list = None
    ) -> None:
        # if self.robot_interface is None:
        #     print(f"[SIM] Would navigate to {location_name} (speed={speed}).")
        #     return

        # Tuck the arm into the left-back retract config before driving the base,
        # so the arm is in a safe, compact pose for the whole navigation (all
        # via waypoints + destination). No-op visualization in sim.
        if self.robot_interface is not None:
            self.robot_interface.set_speed(speed)
        print("Retracting arm to left_back_retract_pos before navigation ...")
        self.move_to_joint_positions(self.sim.scene_description.left_back_retract_pos)

        if not ROS_NAV_IMPORTED:
            raise RuntimeError(
                "ROS navigation modules not found. "
                "Please run in a ROS environment with move_base installed."
            )

        self._ensure_teleop_subscribers()
        self._get_move_base_client()
        self._get_tf_buffer()  # warm the TF listener (used by the stall watchdog)

        # Drive any (usable) intermediate staging waypoints first, then the
        # destination. `via` is set only by the legs that need it -- e.g.
        # navigate_to_table routes through the kitchen-exit staging pose.
        waypoints = self._resolve_via(list(via or [])) + [location_name]
        if len(waypoints) > 1:
            print(f"[nav] routing {' -> '.join(waypoints)}")

        for i, wp in enumerate(waypoints):
            pose = self._load_target_pose(wp)
            self._drive_to_pose(wp, pose)
            if i == len(waypoints) - 1:
                # Final destination only -- staging poses just need to be reached
                # coarsely. move_base's first SUCCEEDED is against a map->base
                # estimate Cartographer is still correcting, so wait for it to
                # settle, then re-send the SAME goal ONCE: TEB drives out whatever
                # gap the correction revealed (a no-op if it was already there),
                # against the now-corrected estimate. Then fine-tune the heading.
                self._wait_for_localization_settle(
                    self._GOAL_CONFIRM_SETTLE_S,
                    "confirming the goal pose (replan once)",
                )
                self._drive_to_pose(wp, pose)
                self._refinement_window(wp, pose)

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
        while True:
            if client.wait_for_result(rospy.Duration(poll_s)):
                break  # move_base/manager reached a terminal state
            if rospy.is_shutdown():
                client.cancel_goal()
                raise RuntimeError("ROS shutdown during navigation")
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
        print(f"[nav] move_base failed for {location_name}. Action state code={state}")
        return "failed"

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

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = str(pose["frame_id"])
        goal.target_pose.pose.position.x = pose["x"]
        goal.target_pose.pose.position.y = pose["y"]
        goal.target_pose.pose.position.z = pose["z"]
        goal.target_pose.pose.orientation.x = pose["qx"]
        goal.target_pose.pose.orientation.y = pose["qy"]
        goal.target_pose.pose.orientation.z = pose["qz"]
        goal.target_pose.pose.orientation.w = pose["qw"]

        print(
            f"Navigating to {location_name} using {self._location_yaml()} ...\n"
            f"  Localization-stall watchdog: stop only if map->{self._BASE_FRAME} "
            f"is lost for > {self._LOCALIZATION_STALL_TIMEOUT_S:.0f}s of autonomous "
            f"driving (resets the moment localization recovers)."
        )

        attempts = 0
        recovering = False
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
                print(f"Reached {location_name}.")
                return

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

    def navigate_to_fridge(self, speed: str) -> None:
        self._navigate_to_target("fridge", speed)

    def navigate_to_microwave(self, speed: str) -> None:
        self._navigate_to_target("microwave", speed)

    def navigate_to_sink(self, speed: str) -> None:
        self._navigate_to_target("sink", speed)

    def navigate_to_table(self, speed: str) -> None:
        # microwave -> table is the kitchen egress: reverse out through the narrow
        # corridor to the open staging area, then turn and drive to the table.
        # Routing via the staging waypoint stops TEB from oscillating as it tries
        # to turn inside the corridor. (Auto-skipped while the staging pose is an
        # unset placeholder.)
        self._navigate_to_target("table", speed, via=[self._STAGING_WAYPOINT])
