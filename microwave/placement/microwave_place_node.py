"""ROS 2 node: place the held OXO box into the OPEN microwave (same steps as the CLI,
`real_gen3_ros2_place_container_microwave.py`, via `placement_workflow`).

    python3 -u microwave/placement/microwave_place_node.py --ros-args \\
        -p container_drop:=0.09 -p lowering:=impedance -p allow_execute:=false
    ros2 run rqt_image_view rqt_image_view /microwave_place/overlay
    rviz2   # fixed frame arm_base_link; add /microwave_place/cloud, /obstacles, /markers

Run from the repo root (the sim config path is relative), bring-up up (bringup.sh).

Publishes
  ~/overlay        sensor_msgs/Image (bgr8)  live: YOLO box (grey), the cavity's fitted floor
                   (green dots) and bounds (yellow) once perceived, target points, status line
  ~/cloud          sensor_msgs/PointCloud2 (arm_base_link)  filtered ROI cloud of the last look
  ~/obstacles      sensor_msgs/PointCloud2 (arm_base_link)  voxel centres of the last plan's obstacles
  ~/markers        visualization_msgs/MarkerArray  cavity box, floor plane, container footprint, poses
  ~/target_poses   geometry_msgs/PoseArray  [pre-insert, above, contact] of the last plan
  ~/status         std_msgs/String  one line per service call

Services (std_srvs/Trigger; message = result or the reason it refused)
  ~/perceive         stable detection + cavity. No motion.
  ~/plan_place       perceive + plan every leg. No motion.
  ~/execute_place    run the last plan through `stop_after` (param). Hand stays on the box.
  ~/plan_release     plan open / lift / back out / park from where the hand is. No motion.
  ~/execute_release  run it.
  execute_* need allow_execute:=true, a plan younger than 120 s and the arm where the plan
  started; a plan is used once.

Parameters: container_drop (m, required for plan), lowering (impedance|position), stop_after
(pre-insert|insert|lower), allow_execute (bool), preview_hz (live detector overlay, 0 = off),
config (YAML path, '' = default).
"""
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Pose, PoseArray, Quaternion
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))
from feeding_deployment.ros2.node import get_node  # noqa: E402

import placement_workflow as wf  # noqa: E402
from microwave_detector import YoloMicrowaveDetector  # noqa: E402
from placement_config import load_config  # noqa: E402

FRAME_ID = "arm_base_link"


class MicrowavePlaceNode:
    def __init__(self, node):
        self.node = node
        self.log = node.get_logger()
        for name, default in (("container_drop", -1.0), ("lowering", ""), ("stop_after", "lower"),
                              ("allow_execute", False), ("preview_hz", 3.0), ("config", "")):
            node.declare_parameter(name, default)
        self.bridge = CvBridge()
        group = ReentrantCallbackGroup()
        self.pub_overlay = node.create_publisher(Image, "~/overlay", 1)
        self.pub_cloud = node.create_publisher(PointCloud2, "~/cloud", 1)
        self.pub_obst = node.create_publisher(PointCloud2, "~/obstacles", 1)
        self.pub_markers = node.create_publisher(MarkerArray, "~/markers", 1)
        self.pub_targets = node.create_publisher(PoseArray, "~/target_poses", 1)
        self.pub_status = node.create_publisher(String, "~/status", 10)

        from feeding_deployment.control.robot_controller.arm_client import ArmInterfaceClient
        from feeding_deployment.perception.tf_interface import TFInterface
        from feeding_deployment.ros2.realsense_ros2_interface import RealSenseROS2Interface
        cfg = self._cfg()
        self.log.info(f"loading {cfg.detector.model} + camera + tf ...")
        rs = RealSenseROS2Interface()
        if not rs.wait_for_frames(30.0):
            self.log.warn("no RGB-D frames yet -- is realsense2_camera up with align_depth.enable:=true?")
        self.source = wf.RosFrameSource(rs, TFInterface(), cfg.stabilizer.max_frame_age_s)
        self.detector = YoloMicrowaveDetector(cfg.detector)
        self.arm_cls = ArmInterfaceClient
        self.arm = None
        self.model_lock = threading.Lock()
        self.task_lock = threading.Lock()
        self.perception = self.place_plan = self.release_plan = None

        for name, cb in (("perceive", self.srv_perceive), ("plan_place", self.srv_plan_place),
                         ("execute_place", self.srv_execute_place), ("plan_release", self.srv_plan_release),
                         ("execute_release", self.srv_execute_release)):
            node.create_service(Trigger, f"~/{name}", cb, callback_group=group)
        hz = float(node.get_parameter("preview_hz").value)
        if hz > 0:
            node.create_timer(1.0 / hz, self.preview, callback_group=group)
        self.log.info(f"ready (preview {hz:g} Hz, allow_execute {node.get_parameter('allow_execute').value})")

    def _cfg(self):
        p = self.node.get_parameter
        over = {}
        if float(p("container_drop").value) > 0:
            over["container"] = {"drop": float(p("container_drop").value)}
        if p("lowering").value:
            over["lowering"] = {"mode": p("lowering").value}
        return load_config(p("config").value or None, over)

    # --- live preview: detector on the newest frame + the last perception drawn on it ---
    def preview(self):
        if not self.model_lock.acquire(blocking=False):
            return
        try:
            fr = self.source.get()
            det = self.detector.detect(fr.bgr, fr.base_T_cam, fr.stamp)
            per, plan = self.perception, self.place_plan
            msg = (f"microwave {det.conf:.2f}{' (FALLBACK ' + det.label + ')' if det.fallback else ''}"
                   if det else "no microwave detection")
            vis = wf.draw_overlay(fr, cavity=per.cavity if per else None, target=plan.target if plan else None,
                                  msg=msg, ok=det is not None, dets=[det] if det else [])
            img = self.bridge.cv2_to_imgmsg(vis, "bgr8")
            img.header.stamp, img.header.frame_id = self.node.get_clock().now().to_msg(), "camera_color_optical_frame"
            self.pub_overlay.publish(img)
        except Exception as e:  # noqa: BLE001 -- the preview must never take the node down
            self.log.warn(f"preview: {e}", throttle_duration_sec=5.0)
        finally:
            self.model_lock.release()

    # --- services ---
    def _run(self, res, fn):
        if not self.task_lock.acquire(blocking=False):
            res.success, res.message = False, "busy with another call -- refused."
            return res
        try:
            if self.arm is None:
                self.arm = self.arm_cls()
            res.success, res.message = fn()
        except wf.PlacementRefused as e:
            res.success, res.message = False, str(e)
        except Exception as e:  # noqa: BLE001
            self.log.error(f"{type(e).__name__}: {e}")
            res.success, res.message = False, f"error: {type(e).__name__}: {e}"
        finally:
            self.task_lock.release()
        self.pub_status.publish(String(data=f"{'OK' if res.success else 'FAILED'}: {res.message}"))
        (self.log.info if res.success else self.log.warn)(res.message)
        return res

    def _perceive(self, cfg):
        st = self.arm.get_state()
        ee = np.asarray(st["ee_pos"], float)
        with self.model_lock:
            per = wf.perceive(self.source, self.detector, cfg, tool_pose=(ee[:3], ee[3:7]), log=self.log.info)
        self.perception = per
        self._publish_cloud(self.pub_cloud, per.roi_cloud)
        self._publish_markers(per.cavity, None)
        return per

    def _gate(self, plan, what):
        if not self.node.get_parameter("allow_execute").value:
            raise wf.PlacementRefused("allow_execute is false -- dry-run node, nothing commanded")
        if plan is None:
            raise wf.PlacementRefused(f"no {what} plan -- call plan_{what} first")

    def srv_perceive(self, _req, res):
        def go():
            per = self._perceive(self._cfg())
            return True, f"{per.cavity.summary()} (logs {per.log_dir})"
        return self._run(res, go)

    def srv_plan_place(self, _req, res):
        def go():
            self.place_plan = None
            cfg = self._cfg()
            per = self._perceive(cfg)
            plan = wf.plan_placement(self.arm, per, cfg, log=self.log.info)
            self.place_plan, self.place_cfg = plan, cfg
            self._publish_cloud(self.pub_obst, plan.voxels)
            self._publish_markers(per.cavity, plan)
            self._publish_targets(plan)
            return True, plan.summary().replace("\n", " | ")
        return self._run(res, go)

    def srv_execute_place(self, _req, res):
        def go():
            self._gate(self.place_plan, "place")
            plan, self.place_plan = self.place_plan, None
            return wf.execute_placement(self.arm, plan, self.place_cfg,
                                        stop_after=self.node.get_parameter("stop_after").value, log=self.log.info)
        return self._run(res, go)

    def srv_plan_release(self, _req, res):
        def go():
            self.release_plan = None
            cfg = self._cfg()
            self.release_plan = wf.plan_release(self.arm, cfg, log=self.log.info)
            return True, ("planned: open, lift, back out"
                          + (" + park" if self.release_plan["park_leg"] is not None else " (no park leg)"))
        return self._run(res, go)

    def srv_execute_release(self, _req, res):
        def go():
            self._gate(self.release_plan, "release")
            if time.time() - self.release_plan["time"] > wf.PLAN_MAX_AGE_S:
                raise wf.PlacementRefused("release plan too old -- re-plan")
            plan, self.release_plan = self.release_plan, None
            return wf.execute_release(self.arm, plan, log=self.log.info)
        return self._run(res, go)

    # --- visualisation ---
    def _header(self):
        return Header(stamp=self.node.get_clock().now().to_msg(), frame_id=FRAME_ID)

    def _publish_cloud(self, pub, pts):
        if pts is not None and len(pts):
            pub.publish(point_cloud2.create_cloud_xyz32(self._header(), np.asarray(pts, np.float32).tolist()))

    def _publish_markers(self, cav, plan):
        h = self._header()
        arr = MarkerArray(markers=[Marker(header=h, action=Marker.DELETEALL)])
        rot = np.column_stack([cav.f, cav.l, [0, 0, 1]])
        from scipy.spatial.transform import Rotation as R
        q = Quaternion(**dict(zip("xyzw", map(float, R.from_matrix(rot).as_quat()))))

        def box(mid, ns, center, size, rgba):
            m = Marker(header=h, ns=ns, id=mid, type=Marker.CUBE, action=Marker.ADD)
            m.pose = Pose(position=Point(**dict(zip("xyz", map(float, center)))), orientation=q)
            m.scale.x, m.scale.y, m.scale.z = map(float, size)
            m.color.r, m.color.g, m.color.b, m.color.a = rgba
            return m

        c = cav.point((cav.front + cav.back) / 2, (cav.left + cav.right) / 2, (cav.floor_z + cav.top) / 2)
        arr.markers.append(box(1, "cavity", c, (cav.depth, cav.width, cav.height), (1.0, 1.0, 0.0, 0.15)))
        fc = cav.point((cav.front + cav.back) / 2, (cav.left + cav.right) / 2, cav.floor_z)
        arr.markers.append(box(2, "floor", fc, (cav.depth, cav.width, 0.003), (0.0, 1.0, 0.0, 0.6)))
        if plan is not None:
            cc = self.place_cfg.container
            b = plan.target.box_center + np.array([0, 0, cc.height / 2])
            arr.markers.append(box(3, "container", b, (cc.far_past_tool - cc.near_past_tool, cc.width, cc.height),
                                   (0.0, 0.4, 1.0, 0.6)))
        self.pub_markers.publish(arr)

    def _publish_targets(self, plan):
        q = Quaternion(**dict(zip("xyzw", map(float, plan.target.quat))))
        arr = PoseArray(header=self._header(), poses=[
            Pose(position=Point(**dict(zip("xyz", map(float, getattr(plan.target, k))))), orientation=q)
            for k in ("pre", "above", "contact")])
        self.pub_targets.publish(arr)


def main():
    rclpy.init(args=sys.argv)
    node = get_node("microwave_place")
    MicrowavePlaceNode(node)
    try:
        while rclpy.ok():
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
