"""ROS 2 node: place the held container into the OPEN microwave.

The same steps as `real_gen3_ros2_place_container_microwave.py` (it imports them), exposed as a
node so other components (UI, executive) can drive the placement and watch the perception live.

    python3 -u microwave/placement/microwave_place_node.py \\
        --ros-args -p container_drop:=0.05 -p allow_execute:=false
    ros2 run rqt_image_view rqt_image_view /microwave_place/interior_overlay   # live SAM 3 view

Run from the repo root (the sim config path is relative), with the usual bring-up up (camera,
robot_state_publisher, joint_state_bridge, calibration_tf; arm_server for the services, plus
bulldog_bypass for motion). SAM 3 loads once at start (~tens of seconds).

Publishes
  ~/interior_overlay   sensor_msgs/Image (bgr8)  every preview tick: colour frame + interior mask
                       (green) + placement point (red) + a status line (green ok / red why not)
  ~/placement_point    geometry_msgs/PointStamped (arm_base_link)  container centre on the
                       cavity floor -- every preview tick that gives a plausible cavity
  ~/target_poses       geometry_msgs/PoseArray (arm_base_link)  [pre-insert, above, place] tool
                       poses of the last successful plan_place
  ~/status             std_msgs/String  one line per service call result

Services (std_srvs/Trigger; `message` = result or the reason it refused)
  ~/plan_place         look (2 looks within 3 cm) + compute poses + plan in sim. NO motion.
  ~/execute_place      run the last plan_place: approach, insert, lower. Hand stays on the container.
  ~/plan_release       plan: open gripper, back out to pre-insert, park. NO motion.
  ~/execute_release    run the last plan_release.
  execute_* need `allow_execute:=true`, a plan younger than PLAN_MAX_AGE_S, and the arm still where
  the plan started (else: re-plan). A plan is used once.

Parameters
  container_drop (double, m)   tool frame -> container bottom, measured; required for plan_place
  allow_execute  (bool)        false = the execute_* services refuse (dry-run only node)
  preview_hz     (double)      live overlay rate; 0 = off. Paused while plan_place is looking.

The live preview needs only the camera (mask only) -- plus tf and ~/.microwave_door.json for the
cavity / placement point. The services need arm_server.
"""
import json
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PointStamped, Pose, PoseArray, Quaternion
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

sys.path.insert(0, str(Path(__file__).resolve().parent))
from feeding_deployment.ros2.node import get_node  # noqa: E402

import real_gen3_ros2_place_container_microwave as place  # noqa: E402
from microwave_cavity import microwave_frame  # noqa: E402

PLAN_MAX_AGE_S = 120.0   # an older plan is refused: the scene (or the microwave) may have changed
FRAME_ID = "arm_base_link"


class MicrowavePlaceNode:
    def __init__(self, node):
        self.node = node
        self.log = node.get_logger()
        node.declare_parameter("container_drop", -1.0)
        node.declare_parameter("allow_execute", False)
        node.declare_parameter("preview_hz", 2.0)

        self.bridge = CvBridge()
        group = ReentrantCallbackGroup()   # long services must not block camera/tf callbacks
        self.pub_overlay = node.create_publisher(Image, "~/interior_overlay", 1)
        self.pub_point = node.create_publisher(PointStamped, "~/placement_point", 1)
        self.pub_targets = node.create_publisher(PoseArray, "~/target_poses", 1)
        self.pub_status = node.create_publisher(String, "~/status", 10)

        self.log.info("loading SAM 3 + camera + tf ...")
        self.detector = place.detect_handle_sam3.build_detector(prompt=place.PROMPT)[:3]
        self.model_lock = threading.Lock()   # one SAM 3 pass at a time (preview vs plan_place)
        self.task_lock = threading.Lock()    # one service at a time
        self.ai = None
        self.place_plan = self.release_plan = None

        for name, cb in (("plan_place", self.srv_plan_place), ("execute_place", self.srv_execute_place),
                         ("plan_release", self.srv_plan_release), ("execute_release", self.srv_execute_release)):
            node.create_service(Trigger, f"~/{name}", cb, callback_group=group)
        hz = float(node.get_parameter("preview_hz").value)
        if hz > 0:
            node.create_timer(1.0 / hz, self.preview, callback_group=group)
        self.log.info(f"ready (prompt {place.PROMPT!r}, preview {hz:g} Hz, "
                      f"allow_execute {node.get_parameter('allow_execute').value})")

    # --- live preview ---
    def preview(self):
        if not self.model_lock.acquire(blocking=False):
            return   # plan_place is looking, or the previous tick is still running
        try:
            cavity, _, vis = place.look_inside(*self.detector, self._frame_or_none())
        except Exception as e:  # noqa: BLE001 -- the preview must never take the node down
            self.log.warn(f"preview: {e}", throttle_duration_sec=5.0)
            return
        finally:
            self.model_lock.release()
        stamp = self.node.get_clock().now().to_msg()
        if vis is not None:
            msg = self.bridge.cv2_to_imgmsg(vis, "bgr8")
            msg.header.stamp, msg.header.frame_id = stamp, "camera_color_optical_frame"
            self.pub_overlay.publish(msg)
        if cavity is not None:
            pt = PointStamped(point=Point(**dict(zip("xyz", map(float, cavity["placement_point"])))))
            pt.header.stamp, pt.header.frame_id = stamp, FRAME_ID
            self.pub_point.publish(pt)

    @staticmethod
    def _frame_or_none():
        try:
            return microwave_frame(json.loads(place.DOOR_FILE.read_text())["closed_normal"])
        except Exception:  # noqa: BLE001 -- no/partial door file: mask-only preview
            return None

    # --- services ---
    def _run(self, res, fn):
        """Serialise service calls; turn refusals/errors into a failed Trigger response."""
        if not self.task_lock.acquire(blocking=False):
            res.success, res.message = False, "busy with another call -- refused."
            return res
        try:
            if self.ai is None:
                self.ai = place.ArmInterfaceClient()
            res.success, res.message = fn()
        except place.PlacementRefused as e:
            res.success, res.message = False, str(e)
        except Exception as e:  # noqa: BLE001 -- report, don't kill the node
            self.log.error(f"{type(e).__name__}: {e}")
            res.success, res.message = False, f"error: {type(e).__name__}: {e}"
        finally:
            self.task_lock.release()
        self.pub_status.publish(String(data=f"{'OK' if res.success else 'FAILED'}: {res.message}"))
        (self.log.info if res.success else self.log.warn)(res.message)
        return res

    def _execute_gate(self, plan, what):
        if not self.node.get_parameter("allow_execute").value:
            raise place.PlacementRefused("allow_execute is false -- dry-run node, nothing commanded.")
        if plan is None:
            raise place.PlacementRefused(f"no {what} plan -- call plan_{what} first.")
        if time.time() - plan["time"] > PLAN_MAX_AGE_S:
            raise place.PlacementRefused(f"the {what} plan is older than {PLAN_MAX_AGE_S:.0f} s -- re-plan.")

    def srv_plan_place(self, _req, res):
        def go():
            self.place_plan = None
            drop = float(self.node.get_parameter("container_drop").value)
            with self.model_lock:
                plan = place.plan_placement(self.ai, drop, self.detector)
            self.place_plan = plan
            self._publish_targets(plan)
            return True, (f"planned: pre {np.round(plan['pre'], 3).tolist()} above "
                          f"{np.round(plan['above'], 3).tolist()} place {np.round(plan['place'], 3).tolist()}")
        return self._run(res, go)

    def srv_execute_place(self, _req, res):
        def go():
            self._execute_gate(self.place_plan, "place")
            plan, self.place_plan = self.place_plan, None   # a plan is used once
            return place.execute_placement(self.ai, plan)
        return self._run(res, go)

    def srv_plan_release(self, _req, res):
        def go():
            self.release_plan = None
            plan = place.plan_release(self.ai)
            self.release_plan = plan
            return True, ("planned: release + back out"
                          + (" + park" if plan["park_leg"] is not None else " (no park leg -- return by hand)"))
        return self._run(res, go)

    def srv_execute_release(self, _req, res):
        def go():
            self._execute_gate(self.release_plan, "release")
            plan, self.release_plan = self.release_plan, None
            return place.execute_release(self.ai, plan)
        return self._run(res, go)

    def _publish_targets(self, plan):
        q = Quaternion(**dict(zip("xyzw", map(float, plan["quat"]))))
        arr = PoseArray(poses=[Pose(position=Point(**dict(zip("xyz", map(float, plan[k])))), orientation=q)
                               for k in ("pre", "above", "place")])
        arr.header.stamp, arr.header.frame_id = self.node.get_clock().now().to_msg(), FRAME_ID
        self.pub_targets.publish(arr)


def main():
    rclpy.init(args=sys.argv)   # so --ros-args -p ... reach the node's parameters
    node = get_node("microwave_place")   # the shared node the camera/tf interfaces also use
    MicrowavePlaceNode(node)
    try:
        while rclpy.ok():
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
