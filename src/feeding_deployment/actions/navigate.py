import math
import os
from collections import deque
from pathlib import Path
from typing import Any, Tuple

import yaml

try:
    import actionlib
    import rospy
    import tf2_ros
    from actionlib_msgs.msg import GoalStatus
    from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

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

    # Set to False to skip the refinement window entirely.
    _USE_REFINEMENT_WINDOW: bool = True

    # How long to monitor after move_base declares SUCCEEDED.
    _REFINEMENT_TIMEOUT_S = 5.0
    _REFINEMENT_RATE_HZ = 10.0
    # Sliding-window length for averaging (smooths out localization jitter).
    _REFINEMENT_WINDOW_S = 1.0
    # Don't check divergence until this many seconds in (lets best establish).
    _REFINEMENT_WARMUP_S = 1.0
    # Stop early if windowed avg rises this much above the rolling minimum.
    _DIVERGENCE_MARGIN_M = 0.02      # 2 cm
    _DIVERGENCE_MARGIN_RAD = 0.005   # ~0.3 deg
    # Success thresholds are derived from move_base params at runtime (see
    # _refinement_window), defaulting to half of xy/yaw_goal_tolerance.

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
        }

    def _speed_to_timeout(self, speed: str) -> float:
        return {
            "low": 300.0,
            "medium": 300.0,
            "high": 180.0,
        }.get(speed, 300.0)

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

    def _navigate_to_target(self, location_name: str, speed: str) -> None:
        # if self.robot_interface is None:
        #     print(f"[SIM] Would navigate to {location_name} (speed={speed}).")
        #     return

        if not ROS_NAV_IMPORTED:
            raise RuntimeError(
                "ROS navigation modules not found. "
                "Please run in a ROS environment with move_base installed."
            )

        pose = self._load_target_pose(location_name)
        client = self._get_move_base_client()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = str(pose["frame_id"])
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = pose["x"]
        goal.target_pose.pose.position.y = pose["y"]
        goal.target_pose.pose.position.z = pose["z"]
        goal.target_pose.pose.orientation.x = pose["qx"]
        goal.target_pose.pose.orientation.y = pose["qy"]
        goal.target_pose.pose.orientation.z = pose["qz"]
        goal.target_pose.pose.orientation.w = pose["qw"]

        timeout_s = self._speed_to_timeout(speed)
        print(
            f"Navigating to {location_name} with timeout={timeout_s:.1f}s "
            f"using {self._location_yaml()} ..."
        )
        client.send_goal(goal)
        # The base is now driving (via the shared_autonomy_manager). Enable the
        # webapp's Robot Base Control button for the duration of this action so
        # the user can take over mid-drive. The navigation page publishes
        # /shared_autonomy/takeover, which the manager honors and still reports
        # SUCCEEDED once the human signals done.
        self._set_base_control_available(True)
        try:
            finished = client.wait_for_result(rospy.Duration(timeout_s))
        finally:
            self._set_base_control_available(False)
        if not finished:
            client.cancel_goal()
            raise TimeoutError(
                f"Navigation to {location_name} timed out after {timeout_s:.1f}s"
            )

        state = client.get_state()
        if state != GoalStatus.SUCCEEDED:
            raise RuntimeError(
                f"move_base failed for {location_name}. Action state code={state}"
            )

        print(f"Reached {location_name}.")
        self._refinement_window(pose)

    @staticmethod
    def _yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
        return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

    def _refinement_window(self, pose: dict) -> None:
        """Monitor convergence for up to _REFINEMENT_TIMEOUT_S after move_base succeeds.

        Tracks a sliding-window average of xy and yaw error vs the goal pose.
        Stops early on convergence (below success thresholds) or divergence
        (windowed average rises above rolling minimum + margin) — same logic
        as ML early stopping: keep going while improving, quit when it gets worse.
        """
        if not self._USE_REFINEMENT_WINDOW or not ROS_NAV_IMPORTED:
            return

        # Success thresholds: half of the move_base goal tolerances so we stop
        # refining once we're comfortably inside what the controller required.
        xy_tol = rospy.get_param("move_base/xy_goal_tolerance", 0.07)
        yaw_tol = rospy.get_param("move_base/yaw_goal_tolerance", 0.015)
        success_xy_m = xy_tol * 0.5
        success_yaw_rad = yaw_tol * 0.5

        tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        _listener = tf2_ros.TransformListener(tf_buffer)

        gx = pose["x"]
        gy = pose["y"]
        goal_yaw = self._yaw_from_quat(pose["qx"], pose["qy"], pose["qz"], pose["qw"])

        rate = rospy.Rate(self._REFINEMENT_RATE_HZ)
        start = rospy.Time.now()
        window: deque = deque()  # (elapsed_s, err_xy, err_yaw)
        best_xy = float("inf")
        best_yaw = float("inf")

        print("Refinement window: monitoring convergence...")

        while not rospy.is_shutdown():
            elapsed_s = (rospy.Time.now() - start).to_sec()
            if elapsed_s >= self._REFINEMENT_TIMEOUT_S:
                print(f"\n  Refinement: timed out after {self._REFINEMENT_TIMEOUT_S:.0f}s.")
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
            err_yaw = abs(math.atan2(
                math.sin(cur_yaw - goal_yaw), math.cos(cur_yaw - goal_yaw)
            ))

            window.append((elapsed_s, err_xy, err_yaw))
            while window and elapsed_s - window[0][0] > self._REFINEMENT_WINDOW_S:
                window.popleft()

            if len(window) < 3:
                rate.sleep()
                continue

            avg_xy = sum(w[1] for w in window) / len(window)
            avg_yaw = sum(w[2] for w in window) / len(window)

            best_xy = min(best_xy, avg_xy)
            best_yaw = min(best_yaw, avg_yaw)

            print(
                f"\r  [{elapsed_s:4.1f}s] xy={avg_xy*100:.1f}cm "
                f"(best {best_xy*100:.1f})  "
                f"yaw={math.degrees(avg_yaw):.2f}deg "
                f"(best {math.degrees(best_yaw):.2f})   ",
                end="", flush=True,
            )

            if avg_xy < success_xy_m and avg_yaw < success_yaw_rad:
                print(
                    f"\n  Converged: xy={avg_xy*100:.1f}cm "
                    f"yaw={math.degrees(avg_yaw):.2f}deg"
                )
                break

            if elapsed_s >= self._REFINEMENT_WARMUP_S:
                xy_diverging = avg_xy > best_xy + self._DIVERGENCE_MARGIN_M
                yaw_diverging = avg_yaw > best_yaw + self._DIVERGENCE_MARGIN_RAD
                if xy_diverging or yaw_diverging:
                    print(
                        f"\n  Stopped: diverging — "
                        f"xy={avg_xy*100:.1f}cm vs best {best_xy*100:.1f}cm, "
                        f"yaw={math.degrees(avg_yaw):.2f}deg vs best {math.degrees(best_yaw):.2f}deg"
                    )
                    break

            rate.sleep()

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
        self._navigate_to_target("table", speed)