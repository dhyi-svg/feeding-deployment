"""Read-only: old colour detector vs the shape-only one on live frames (+ the same frames made cream)."""
import sys, time, numpy as np, rclpy, importlib.util
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
S = str(__import__("pathlib").Path(__file__).resolve().parent); sys.path.insert(0,S)
import rammp_button_press.dome_pattern as old
spec=importlib.util.spec_from_file_location("dpx",f"{S}/dome_pattern_shape_wip.py"); new=importlib.util.module_from_spec(spec); sys.modules["dpx"]=new; spec.loader.exec_module(new)
from rammp_button_press.perception import to_bgr_depth
import eval_frames as e
rclpy.init(); n=rclpy.create_node("live_compare"); br=CvBridge(); st={}
ns="/wrist_camera"
n.create_subscription(CameraInfo,f"{ns}/color/camera_info",lambda m: st.__setitem__("k",m.k),qos_profile_sensor_data)
n.create_subscription(Image,f"{ns}/color/image_raw",lambda m: st.__setitem__("c",m),qos_profile_sensor_data)
n.create_subscription(Image,f"{ns}/aligned_depth_to_color/image_raw",lambda m: st.__setitem__("d",m),qos_profile_sensor_data)
last=None; k=0; t=time.time()
fmt=lambda f: "-" if f is None else "%s %.1fmm z%.2f" % (tuple(np.round(f.px["start_30s"]).astype(int)), f.s_mm, f.z)
while k<6 and time.time()-t<20:
    rclpy.spin_once(n,timeout_sec=0.05)
    if {"k","c","d"}<=st.keys() and st["c"] is not last:
        last=st["c"]; k+=1; bgr,d=to_bgr_depth(br,st["c"],st["d"]); fx=st["k"][0]
        print(f"{k} colour {fmt(old.detect(bgr,d,fx)):28s} shape {fmt(new.detect(bgr,d,fx)):28s} shape-on-cream {fmt(new.detect(e.creamify(bgr),d,fx))}")
