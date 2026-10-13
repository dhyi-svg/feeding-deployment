"""Offline stand-in for kinova_gen3_node + the wrist RealSense + camera tf, for a press_button dry run.

Publishes /joint_states, /ee_state (flange = tool FK - 0.12 m), /gripper_state (closed),
/control_status (latched), /diagnostics, a synthetic dome panel on /wrist_camera/..., and a
static base_link -> camera_color_optical_frame. The arm sits 10 cm straight out from the
pre-press spot; the camera is CAM_D in front of the button, looking at it.
"""
import sys
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import TransformStamped
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_ros import StaticTransformBroadcaster

sys.path.insert(0, "/home/jetson/feeding-deployment/rammp_button_press")
sys.path.insert(0, "/home/jetson/feeding-deployment/rammp_button_press/test")
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parent))
from rammp_arm_interfaces.msg import EeState, GripperState  # noqa: E402
from rammp_common_interfaces.msg import ControlStatus  # noqa: E402
from test_dome_pattern import FX, panel  # noqa: E402

from rammp_button_press.kinematics import Kinematics  # noqa: E402
from rammp_button_press.panel_frame import panel_frame  # noqa: E402
import patch_offset  # noqa: E402,F401
from rammp_button_press.press_button import prepress_pose  # noqa: E402

BUTTON = np.array([float(v) for v in (sys.argv[2] if len(sys.argv) > 2 else "0.70,0.05,0.25").split(",")])
NORMAL = np.array([-1.0, 0.0, 0.0])      # panel faces the robot
CAM_D = float(sys.argv[1]) if len(sys.argv) > 1 else 0.23
TOOL_Z = 0.12
TARGET = "timer_clock"

kin = Kinematics()
origin, R = panel_frame(BUTTON, NORMAL)
p_goal, R_goal = prepress_pose(origin, R)
p_start = p_goal + 0.10 * NORMAL
q = np.array([0.0, 0.35, -3.14, -2.0, 0.0, -0.9, 1.57])
import pybullet as p  # noqa: E402
for _ in range(20):
    kin._set_sim(q)
    q = np.array(p.calculateInverseKinematics(kin.robot_id, kin.end_effector_id, list(p_start),
                                              list(Rotation.from_matrix(R_goal).as_quat()),
                                              maxNumIterations=400, residualThreshold=1e-6,
                                              physicsClientId=kin.cid)[:7])
tool, tq = kin.fk(q)
print("start tool err (mm):", np.linalg.norm(tool - p_start) * 1000, "joints deg", np.round(np.degrees(q), 1))
flange = tool - Rotation.from_quat(tq).apply([0, 0, TOOL_Z])

# camera: CAM_D out from the button along the normal, optical z looking at the panel, image y down
z_c = -NORMAL
y_c = np.array([0.0, 0.0, -1.0])
x_c = np.cross(y_c, z_c)
cam_pos = BUTTON + CAM_D * NORMAL
R_cam = np.column_stack([x_c, y_c, z_c])

# Synthetic panel: the domes drawn so that `timer_clock` lands on the image centre ray.
import rammp_button_press.dome_pattern as dp  # noqa: E402
bgr0, depth0, truth = panel(z=CAM_D, distractors=False)
off = np.array([320.0, 240.0]) - truth[TARGET]
bgr, depth, truth = panel(z=CAM_D, centre=(320 + off[0], 260 + off[1]), distractors=False)
print("target px", truth[TARGET], "fit:", dp.detect(bgr, depth, FX) is not None)

rclpy.init()
node = rclpy.create_node("mock_rig")
br = CvBridge()
pub_js = node.create_publisher(JointState, "/joint_states", qos_profile_sensor_data)
pub_ee = node.create_publisher(EeState, "/ee_state", qos_profile_sensor_data)
pub_g = node.create_publisher(GripperState, "/gripper_state", qos_profile_sensor_data)
pub_cs = node.create_publisher(ControlStatus, "/control_status", QoSProfile(
    depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
pub_d = node.create_publisher(DiagnosticArray, "/diagnostics", 10)
pub_c = node.create_publisher(Image, "/wrist_camera/color/image_raw", 1)
pub_dp = node.create_publisher(Image, "/wrist_camera/aligned_depth_to_color/image_raw", 1)
pub_i = node.create_publisher(CameraInfo, "/wrist_camera/color/camera_info", 1)
cs = ControlStatus()
cs.arbitration_enabled, cs.estopped, cs.owned = False, False, False
pub_cs.publish(cs)
stb = StaticTransformBroadcaster(node)
t = TransformStamped()
t.header.frame_id, t.child_frame_id = "base_link", "camera_color_optical_frame"
t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, cam_pos)
qc = Rotation.from_matrix(R_cam).as_quat()
t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = map(float, qc)
stb.sendTransform(t)
depth_mm = (depth * 1000).astype(np.uint16)


def tick_fast():
    js = JointState()
    js.header.stamp = node.get_clock().now().to_msg()
    js.name = [f"joint_{i}" for i in range(1, 8)] + ["robotiq_85_left_knuckle_joint"]
    js.position = list(map(float, q)) + [0.8]
    pub_js.publish(js)
    ee = EeState()
    ee.header.stamp = js.header.stamp
    ee.pose.position.x, ee.pose.position.y, ee.pose.position.z = map(float, flange)
    ee.pose.orientation.x, ee.pose.orientation.y, ee.pose.orientation.z, ee.pose.orientation.w = map(float, tq)
    pub_ee.publish(ee)


def tick_cam():
    g = GripperState()
    g.position, g.present = 0.98, True
    pub_g.publish(g)
    stamp = node.get_clock().now().to_msg()
    c = br.cv2_to_imgmsg(bgr, "bgr8")
    d = br.cv2_to_imgmsg(depth_mm, "16UC1")
    c.header.stamp = d.header.stamp = stamp
    c.header.frame_id = d.header.frame_id = "camera_color_optical_frame"
    info = CameraInfo()
    info.header = c.header
    info.width, info.height = 640, 480
    info.k = [FX, 0.0, 320.0, 0.0, FX, 240.0, 0.0, 0.0, 1.0]
    pub_c.publish(c)
    pub_dp.publish(d)
    pub_i.publish(info)


def tick_diag():
    a = DiagnosticArray()
    a.header.stamp = node.get_clock().now().to_msg()
    for name in ("Arbitration", "Arm", "Gripper"):
        s = DiagnosticStatus()
        s.name, s.level, s.message = name, DiagnosticStatus.OK, "mock"
        a.status.append(s)
    pub_d.publish(a)


node.create_timer(0.01, tick_fast)
node.create_timer(1 / 15, tick_cam)
node.create_timer(1.0, tick_diag)
print("mock rig up", flush=True)
rclpy.spin(node)
