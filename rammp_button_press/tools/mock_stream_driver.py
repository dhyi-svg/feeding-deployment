"""Mock kinova_gen3_node streaming tier for an offline impedance-press test. RUN ON ROS_DOMAIN_ID=77 ONLY.
args: SAG_MM WALL_MM  (sag: constant offset of the held pose, straight down; wall: panel this far in
from the start along the press normal -- the tool cannot pass it, like a compliant stop)."""
import os, sys, time
assert os.environ.get("ROS_DOMAIN_ID") == "77", "mock must run on domain 77"
import numpy as np, rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from rammp_arm_interfaces.msg import EeState, GripperState, PoseSetpoint
from rammp_arm_interfaces.srv import CloseStream, OpenStream
from rammp_common_interfaces.msg import ControlStatus
SAG = float(sys.argv[1]) / 1000 if len(sys.argv) > 1 else 0.002
WALL = float(sys.argv[2]) / 1000 if len(sys.argv) > 2 else 0.006
N_OUT = np.array([-0.9, -0.435, 0.025]); N_OUT /= np.linalg.norm(N_OUT)
quat = np.array([0.5, 0.6, 0.5, 0.2]); quat /= np.linalg.norm(quat); R = Rotation.from_quat(quat).as_matrix()
P0 = np.array([0.714, 0.049, 0.155])                  # tool frame start = the pre-press spot
tool = P0.copy(); target = P0.copy(); open_ = False; last_sp = 0.0; log = []
rclpy.init(); n = rclpy.create_node("kinova_gen3_node")
pub_js = n.create_publisher(JointState, "/joint_states", qos_profile_sensor_data)
pub_ee = n.create_publisher(EeState, "/ee_state", qos_profile_sensor_data)
pub_g = n.create_publisher(GripperState, "/gripper_state", qos_profile_sensor_data)
pub_cs = n.create_publisher(ControlStatus, "/control_status", QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
cs = ControlStatus(); pub_cs.publish(cs)
def on_sp(m):
    global target, last_sp
    if not open_: return
    f = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
    target = f + R[:, 2] * 0.12; last_sp = time.monotonic()
n.create_subscription(PoseSetpoint, "/setpoint/pose", on_sp, qos_profile_sensor_data)
def on_open(req, res):
    global open_, last_sp, target
    open_ = req.controller == "ee_pose_impedance" and req.timeout_s > 0
    res.accepted = open_; res.channels = ["/setpoint/pose"] if open_ else []; res.message = "mock"
    last_sp = time.monotonic(); target = tool.copy(); print("OPEN", req.controller, req.timeout_s, flush=True); return res
def on_close(req, res):
    global open_, target
    open_ = False; target = tool.copy(); res.closed = True; res.message = "mock"; print("CLOSE", flush=True); return res
n.create_service(OpenStream, "/open_stream", on_open); n.create_service(CloseStream, "/close_stream", on_close)
def tick():
    global tool, open_, target
    if open_ and time.monotonic() - last_sp > 0.1:
        open_ = False; target = tool.copy(); print("EXPIRED", flush=True)
    goal = target + (np.array([0, 0, -SAG]) if open_ else 0)
    tool = tool + (goal - tool) * (0.01 / 0.05)                       # first-order lag, tau 50 ms
    depth = -(tool - P0) @ N_OUT
    if depth > WALL: tool = tool + (depth - WALL) * N_OUT              # the panel stops it
    js = JointState(); js.name = [f"joint_{i}" for i in range(1, 8)]; js.position = [0.0]*7
    js.header.stamp = n.get_clock().now().to_msg(); pub_js.publish(js)
    e = EeState(); f = tool - R[:, 2] * 0.12
    e.pose.position.x, e.pose.position.y, e.pose.position.z = map(float, f)
    e.pose.orientation.x, e.pose.orientation.y, e.pose.orientation.z, e.pose.orientation.w = map(float, quat); pub_ee.publish(e)
    g = GripperState(); g.position = 1.0; g.present = True; pub_g.publish(g)
n.create_timer(0.01, tick); print("mock up", flush=True); rclpy.spin(n)
