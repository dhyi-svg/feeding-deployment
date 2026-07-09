#!/usr/bin/env python3
"""Navigation diagnostics logger.

Passive (subscribe/lookup only) — safe to run alongside a session:
    rosrun feeding_deployment nav_diag_logger.py

Writes to system_logs/navlog_<stamp>/:

  params_snapshot.json  move_base + ZED params + git SHA at start (ablation bookkeeping)
  odom.csv              raw ZED odom pose + per-frame deltas (jumps, stalls, drift)
                        + ZED's native twist (does the SDK velocity survive a re-init?)
  sanitized.csv         sanitizer output pose (join with odom.csv on t_stamp: rows
                        where the two diverge are frames the sanitizer held/adopted)
  hold.csv              /nav_safety_hold transitions with the monitor's reason
                        (interlock timeline; also mirrored into events.csv)
  odom_feedback.csv     velocity feedback consumed by TEB (noise, spikes)
  cmd_vel.csv           commanded velocities (hunting, creep commands)
  applied_vel.csv       bridge post-stiction-floor echo (needs NUC-side bridge update)
  tf.csv                10 Hz: map->odom (cartographer correction), odom->base_link
                        stamp age (ZED latency), map->base_link (true pose)
  costmap.csv           2 Hz: local-costmap cost at robot + max within footprint
  costmap_dump_<t>.txt  full local costmap grid, dumped on TEB-infeasible/abort
  plan.csv              each global (re)plan: pose count + path length (churn)
  imu_1hz.csv           1 Hz RMS gyro / accel deviation (vibration during dwells)
  goals.csv             per-goal start/end/outcome, duration, terminal residual,
                        plus a "settled" row 30 s after success (post-refinement)
  bt_timeline.csv       execution_log.txt lines with wall timestamps (BT context)
  zed_status.csv        rostopic-echoed ZED odom tracking status (type-agnostic)
  zed_status_pose.csv   same for pose/status (odom/status has been empty in every
                        run to date; a zed_status_silent event flags if both stay
                        empty 120 s in, so we learn whether the echo is broken)
  events.csv            auto-flagged anomalies (thresholds below)
  rosout_warn.log       WARN+ from nav stack, plus keyword INFO lines
                        (constraint divergences, corrupted frames, recoveries)
  cmd_latency.csv       actual set_speeds RPC round-trip (compute->NUC->serial),
                        published by the cmd_vel bridge (the leg ~applied misses)
  wheel_odom.csv        wheel-encoder odometry + side disagreement (independent
                        motion witness for a wheel-vs-ZED-vs-carto compare)
  plan_tracking.csv     10 Hz base cross-track + heading error vs the global plan
                        (quantifies TEB path overshoot instead of inferring it)
  usb_dmesg.log         kernel USB/xHCI/Thunderbolt/reset lines (best-effort;
                        the open ZED corrupted-frame cause)
  trigger_<t>_<r>.bag   short rosbag of raw odom/imu/tf(/scan) auto-recorded when a
                        hold/yank/tilt/infeasible/reset fires (full-fidelity; scans
                        via ~bag_include_scans, default true; cap 6 x 15 s)
  map.pgm / map.yaml    the /map grid this run localized against (map_server
                        format) -- makes the log self-describing about its map

Orientation is now logged everywhere it was previously yaw-only: odom.csv and
sanitized.csv carry roll,pitch (+ odom cov_trace); tf.csv carries ob_roll,ob_pitch
(odom->base tilt). imu_1hz.csv carries the gravity vector + derived/ fused
roll,pitch. sys_1hz.csv carries per-core CPU, RAM/swap, GPU, governor, freq.
New events: odom_tilt, odom_origin_reset, odom_stamp_regression, plan_deviation,
rpc_slow, node_gone/node_new, dmesg_unavailable, trigger_bag.
"""

import glob
import json
import math
import os
import shutil
import subprocess
import time
from collections import deque

import rospy
import tf2_ros
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, Twist
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rosgraph_msgs.msg import Log
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, Float64, String

# ---- EVENT THRESHOLDS -------------------------------------------------------
ODOM_JUMP_M = 0.05        # single-frame pose delta (m); ~0.75 m/s implied @15 Hz
ODOM_JUMP_RAD = 0.10      # single-frame yaw delta (rad)
ODOM_GAP_S = 0.30         # stamp gap between odom msgs (nominal 0.067 s)
MAP_ODOM_JUMP_M = 0.05    # map->odom change between 10 Hz samples (yank)
MAP_ODOM_JUMP_RAD = 0.05
TF_STALE_S = 0.40         # odom->base_link stamp age (transform_tolerance 0.5)
CMD_FLIP_V = 0.05         # |vx| above this counts for sign-flip detection
CMD_FLIP_WINDOW_S = 0.7   # vx sign flip within this window = hunting
EVENT_THROTTLE_S = 1.0    # min interval between identical event types
COSTMAP_DUMP_THROTTLE_S = 30.0
FOOTPRINT_RADIUS_M = 0.47  # circumscribed radius of 0.74 x 0.59 footprint
SETTLE_S = 30.0           # re-measure goal residual this long after success
ODOM_TILT_RAD = 0.05      # |roll|/|pitch| of odom above this = tilt (gravity-init bug)
PLAN_DEV_M = 0.30         # base cross-track vs global plan above this = off-path
ORIGIN_RESET_M = 0.50     # prev pose this far out, now ~origin = ZED/odom restart
RPC_SLOW_S = 0.05         # set_speeds RPC round-trip above this = flagged
GPU_SAMPLE_EVERY = 5      # sample nvidia-smi every Nth cb_slow tick (~5 s)
NODE_CHECK_EVERY = 15     # diff the ros node set every Nth cb_slow tick (~15 s)
TRIGGER_BAG_DURATION_S = 15
TRIGGER_BAG_MIN_INTERVAL_S = 45.0
TRIGGER_BAG_MAX = 6       # cap triggered bags per run (disk guard)
TRIGGER_BAG_ON = ("hold_assert", "map_odom_yank", "odom_tilt", "teb_infeasible",
                  "odom_origin_reset")
# Base topics (small) always in a triggered bag. Lidar scans are the size hog
# (~2-5 MB/bag), appended only when ~bag_include_scans (default true) -- drop
# them when the concern is tilt/gravity (odom+imu+tf suffice).
TRIGGER_BAG_TOPICS = ("/zed_mini/zed_node/odom /zed_mini/zed_node/imu/data /tf /tf_static "
                      "/nav_safety_hold /nav_safety_hold_reason /cmd_vel /cmd_vel_teleop "
                      "/wheel_odom")
TRIGGER_BAG_SCAN_TOPICS = "/lidar_l/scan /lidar_r/scan"

# "zed" also matches zed_health_monitor + zed_pose_to_odom_feedback (sanitizer);
# shared_autonomy/cmd_vel_bridge carry the hold pause/resume + gating lines.
ROSOUT_NODES = ("move_base", "cartographer", "zed", "rplidar",
                "shared_autonomy", "cmd_vel_bridge")
# INFO-level lines worth keeping (constraint divergences and ZED grab errors
# are logged at INFO and would be lost with a WARN+ filter alone).
ROSOUT_INFO_KEYWORDS = ("differs by translation", "matches with score",
                        "Dropped", "CORRUPTED", "grab error", "elaboration",
                        "Clearing", "recovery", "Rotate")
LEVELS = {1: "DEBUG", 2: "INFO", 4: "WARN", 8: "ERROR", 16: "FATAL"}

REPO_DIR = os.path.expanduser("~/deployment_ws/src/feeding-deployment")
LOG_BASE = os.path.join(REPO_DIR, "src/feeding_deployment/integration/log")


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def rpy_from_quat(q):
    """(roll, pitch, yaw) from a geometry_msgs Quaternion. Pitch/roll are the
    tilt axes that were invisible in the 2026-07-08 gravity-init failure."""
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
    pitch = math.asin(sinp)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class NavDiagLogger:
    def __init__(self):
        default_dir = os.path.join(LOG_BASE, "system_logs",
                                   "navlog_" + time.strftime("%Y%m%d_%H%M%S"))
        self.outdir = rospy.get_param("~outdir", default_dir)
        os.makedirs(self.outdir, exist_ok=True)

        self.f_odom = self._open("odom.csv",
                                 "t_stamp,x,y,z,yaw,dstep_m,dyaw,dt_s,zvx,zvy,zwz,"
                                 "roll,pitch,cov_trace")
        self.f_san = self._open("sanitized.csv", "t_stamp,x,y,yaw,roll,pitch")
        self.f_hold = self._open("hold.csv", "t_wall,held,reason")
        self.f_fb = self._open("odom_feedback.csv", "t_stamp,vx,vy,wz")
        self.f_cmd = self._open("cmd_vel.csv", "t_wall,vx,wz")
        self.f_app = self._open("applied_vel.csv", "t_wall,vx,wz")
        self.f_lat = self._open("cmd_latency.csv", "t_wall,rpc_latency_s")
        self.f_tf = self._open("tf.csv",
                               "t_wall,mo_x,mo_y,mo_yaw,ob_age_s,mb_x,mb_y,mb_yaw,"
                               "ob_roll,ob_pitch")
        self.f_wheel = self._open("wheel_odom.csv", "t_stamp,x,y,yaw,side_disagree_m")
        self.f_track = self._open("plan_tracking.csv",
                                  "t_wall,cross_track_m,heading_err_rad,plan_n")
        self.f_cost = self._open("costmap.csv", "t_wall,robot_cost,max_footprint_cost")
        self.f_plan = self._open("plan.csv", "t_wall,n_poses,length_m")
        self.f_imu = self._open("imu_1hz.csv",
                                "t_wall,rms_gyro,rms_accel_dev,n,"
                                "grav_roll,grav_pitch,ori_roll,ori_pitch,ax,ay,az")
        self.f_sys = self._open("sys_1hz.csv",
                                "t_wall,n_scan_l,n_scan_r,load1,cpu_temp_c,"
                                "cpu_pct,cpu_max_core_pct,mem_used_pct,swap_used_pct,"
                                "gpu_util_pct,gpu_mem_mb,governor,cpu_mhz")
        self.f_goal = self._open("goals.csv", "t_wall,event,goal_x,goal_y,goal_yaw,"
                                              "duration_s,residual_xy_m,residual_yaw_rad")
        self.f_bt = self._open("bt_timeline.csv", "t_wall,line")
        self.f_ev = self._open("events.csv", "t_wall,type,value,detail")
        self.f_ros = self._open("rosout_warn.log", None)

        # May run before move_base/ZED load their params (logger pane starts
        # with the tmux session); cb_slow retries until the snapshot is complete.
        self.snap_complete = self._snapshot_params()
        self.snap_last_try = time.time()

        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf)

        self.prev_odom = None            # (stamp, x, y, z, yaw)
        self.prev_mo = None              # (x, y, yaw) of map->odom
        self.cmd_hist = deque()          # (t, vx) for flip detection
        self.last_event = {}             # type -> wall time (throttle)
        self.goal = None                 # ((x, y, yaw), t_start)
        self.grid = None                 # latest local costmap (mutable list)
        self.grid_info = None            # (w, h, res, ox, oy, frame)
        self.want_dump = False
        self.last_dump = 0.0
        self.imu_acc = []                # (gyro_mag, accel_dev) since last flush
        self.scan_n = [0, 0]             # lidar_l / lidar_r msgs since last flush
        self.bt_pos = self._bt_seek_end()
        self.status_last_try = 0.0
        self.heartbeat = 0
        self.start_wall = time.time()
        self.status_diag_done = False
        self.hold_state = None           # None until first /nav_safety_hold msg
        self.hold_reason = ""            # latest reason-topic payload
        self.hold_reason_logged = ""     # last reason written to hold.csv
        self.last_side_disagree = float("nan")  # latest /wheel_odom/side_disagreement
        self.plan_pts = []               # latest global plan [(x, y), ...]
        self.plan_yaws = []              # per-pose heading along the plan
        self.cpu_prev = None             # /proc/stat per-core (idle, total) snapshot
        self.gpu_cache = ("", "")        # (util_pct, mem_mb), refreshed periodically
        self.node_set = None             # watched ros node names (restart detection)
        self.dmesg_proc = None           # USB/xhci/thunderbolt dmesg tail
        self.bag_procs = []              # in-flight triggered rosbag record procs
        self.bag_count = 0
        self.last_bag = 0.0
        self.bag_include_scans = bool(rospy.get_param("~bag_include_scans", True))
        self.map_saved = False           # /map -> map.pgm/map.yaml written once

        rospy.Subscriber("/zed_mini/zed_node/odom", Odometry, self.cb_odom, queue_size=100)
        rospy.Subscriber("/zed_mini/zed_node/odom_sanitized", Odometry, self.cb_san,
                         queue_size=100)
        rospy.Subscriber("/nav_safety_hold", Bool, self.cb_hold, queue_size=10)
        rospy.Subscriber("/nav_safety_hold_reason", String, self.cb_hold_reason,
                         queue_size=10)
        rospy.Subscriber("/move_base/odom_feedback", Odometry, self.cb_fb, queue_size=100)
        rospy.Subscriber("/cmd_vel", Twist, self.cb_cmd, queue_size=50)
        # Teleop publishes on its own topic (bridge executes it hold-exempt with
        # priority); merge it into the same cmd_vel.csv for the full command view.
        rospy.Subscriber("/cmd_vel_teleop", Twist, self.cb_cmd, queue_size=50)
        rospy.Subscriber("/cmd_vel_bridge_basicmicro/applied", Twist, self.cb_applied,
                         queue_size=50)
        rospy.Subscriber("/move_base/current_goal", PoseStamped, self.cb_goal_pose, queue_size=5)
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.cb_status, queue_size=5)
        rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid,
                         self.cb_grid, queue_size=2)
        rospy.Subscriber("/move_base/local_costmap/costmap_updates", OccupancyGridUpdate,
                         self.cb_grid_update, queue_size=10)
        rospy.Subscriber("/move_base/NavfnROS/plan", Path, self.cb_plan, queue_size=5)
        # Latched occupancy grid the run localized against -> saved once as
        # map.pgm/map.yaml so the log is self-describing about its map.
        rospy.Subscriber("/map", OccupancyGrid, self.cb_map, queue_size=1)
        rospy.Subscriber("/zed_mini/zed_node/imu/data", Imu, self.cb_imu, queue_size=200)
        rospy.Subscriber("/lidar_l/scan", LaserScan,
                         lambda m: self.scan_n.__setitem__(0, self.scan_n[0] + 1),
                         queue_size=20)
        rospy.Subscriber("/lidar_r/scan", LaserScan,
                         lambda m: self.scan_n.__setitem__(1, self.scan_n[1] + 1),
                         queue_size=20)
        rospy.Subscriber("/rosout_agg", Log, self.cb_rosout, queue_size=200)
        # Independent motion witness (wheel encoders): gives a wheel-vs-ZED-vs-
        # cartographer three-way comparison in EVERY run, not just drift traces.
        rospy.Subscriber("/wheel_odom", Odometry, self.cb_wheel, queue_size=50)
        rospy.Subscriber("/wheel_odom/side_disagreement", Float64,
                         lambda m: setattr(self, "last_side_disagree", m.data),
                         queue_size=10)
        # Actual command round-trip (compute->NUC->serial), published by the
        # cmd_vel bridge. This is the leg the ~applied echo does NOT capture.
        rospy.Subscriber("/cmd_vel_bridge_basicmicro/rpc_latency_s", Float64,
                         self.cb_latency, queue_size=50)

        rospy.Timer(rospy.Duration(0.1), self.cb_tf_sample)
        rospy.Timer(rospy.Duration(0.5), self.cb_costmap_sample)
        rospy.Timer(rospy.Duration(1.0), self.cb_slow)

        # Type-agnostic capture of the ZED tracking status topics. odom/status
        # has been empty in every run to date, so pose/status is captured too
        # and cb_slow flags a zed_status_silent event if both stay empty.
        self.status_topics = (("/zed_mini/zed_node/odom/status", "zed_status.csv"),
                              ("/zed_mini/zed_node/pose/status", "zed_status_pose.csv"))
        self.status_procs = {t: self._spawn_status_echo(t, f)
                             for t, f in self.status_topics}
        self.dmesg_proc = self._spawn_dmesg_tail()
        rospy.on_shutdown(self._shutdown)

        rospy.loginfo("nav_diag_logger writing to %s", self.outdir)

    # ---- setup helpers -------------------------------------------------------
    def _open(self, name, header):
        f = open(os.path.join(self.outdir, name), "w", buffering=1)
        if header:
            f.write(header + "\n")
        return f

    def _snapshot_params(self):
        snap = {"_meta": {"date": time.strftime("%F %T"), "wall": time.time()}}
        try:
            snap["_meta"]["git_sha"] = subprocess.check_output(
                ["git", "-C", REPO_DIR, "rev-parse", "--short", "HEAD"],
                stderr=subprocess.DEVNULL).decode().strip()
        except Exception:
            snap["_meta"]["git_sha"] = "unknown"
        complete = True
        for ns in ("/move_base", "/zed_mini/zed_node", "/cmd_vel_bridge_basicmicro",
                   "/zed_pose_to_odom_feedback"):
            try:
                snap[ns] = rospy.get_param(ns)
            except Exception:
                snap[ns] = "unavailable"
                complete = False
        # Which map cartographer localized against (load_state_filename is a
        # cartographer_node command-line arg, not a rosparam) -- backup identity
        # for the offline plotter if the live /map wasn't captured.
        mapfile = ""
        try:
            out = subprocess.check_output(["pgrep", "-af", "cartographer_node"],
                                          stderr=subprocess.DEVNULL).decode()
            for line in out.splitlines():
                toks = line.replace("=", " ").split()
                for i, t in enumerate(toks):
                    if t == "-load_state_filename" and i + 1 < len(toks):
                        mapfile = toks[i + 1]
        except Exception:
            pass
        snap["_meta"]["cartographer_load_state_filename"] = mapfile
        with open(os.path.join(self.outdir, "params_snapshot.json"), "w") as f:
            json.dump(snap, f, indent=1, default=str)
        # Configs not on the param server (cartographer lua, nav yamls, launch)
        # plus uncommitted-change context, so the run is fully reproducible.
        cfgdir = os.path.join(self.outdir, "config_snapshot")
        os.makedirs(cfgdir, exist_ok=True)
        for pat in ("config/*.lua", "config/nav/*.yaml", "launch/sensors.launch"):
            for p in glob.glob(os.path.join(REPO_DIR, pat)):
                try:
                    dst = os.path.basename(p)
                    if dst.endswith(".launch"):
                        # rename: roslaunch resolves "pkg file.launch" by
                        # searching the whole package tree, and this snapshot
                        # lives inside it -- a copy named sensors.launch makes
                        # every `roslaunch feeding_deployment sensors.launch`
                        # ambiguous.
                        dst = "logged_" + dst
                    shutil.copy(p, os.path.join(cfgdir, dst))
                except Exception:
                    pass
        try:
            diff = subprocess.check_output(["git", "-C", REPO_DIR, "diff"],
                                           stderr=subprocess.DEVNULL).decode()
            with open(os.path.join(self.outdir, "git_diff.patch"), "w") as f:
                f.write(diff)
        except Exception:
            pass
        return complete

    def _spawn_status_echo(self, topic, fname):
        try:
            # append mode + line buffering: survives respawns, and status msgs
            # (published only on tracking-state changes) land immediately
            f = open(os.path.join(self.outdir, fname), "a")
            return subprocess.Popen(["stdbuf", "-oL", "rostopic", "echo", "-p", topic],
                                    stdout=f, stderr=subprocess.DEVNULL)
        except Exception:
            return None

    def _bt_seek_end(self):
        try:
            return os.path.getsize(os.path.join(LOG_BASE, "execution_log.txt"))
        except OSError:
            return 0

    # ---- system-resource samplers (all best-effort; never raise) -------------
    def _sample_cpu(self):
        """(overall %, busiest-core %) since the previous call, from /proc/stat."""
        try:
            cur = {}
            with open("/proc/stat") as f:
                for line in f:
                    if not line.startswith("cpu"):
                        break
                    parts = line.split()
                    name = parts[0]
                    if name == "cpu" or name[3:].isdigit():
                        vals = [int(v) for v in parts[1:]]
                        idle = vals[3] + (vals[4] if len(vals) > 4 else 0)
                        cur[name] = (idle, sum(vals))
            if self.cpu_prev is None:
                self.cpu_prev = cur
                return (0.0, 0.0)
            overall = maxcore = 0.0
            for name, (idle, total) in cur.items():
                pidle, ptotal = self.cpu_prev.get(name, (idle, total))
                dt = total - ptotal
                pct = 100.0 * (1.0 - (idle - pidle) / dt) if dt > 0 else 0.0
                if name == "cpu":
                    overall = pct
                else:
                    maxcore = max(maxcore, pct)
            self.cpu_prev = cur
            return (overall, maxcore)
        except Exception:
            return (-1.0, -1.0)

    def _sample_mem(self):
        """(RAM used %, swap used %) from /proc/meminfo."""
        try:
            mem = {}
            with open("/proc/meminfo") as f:
                for line in f:
                    k, v = line.split(":", 1)
                    mem[k] = float(v.strip().split()[0])
            mt, ma = mem.get("MemTotal", 0.0), mem.get("MemAvailable", 0.0)
            st, sf = mem.get("SwapTotal", 0.0), mem.get("SwapFree", 0.0)
            return (100.0 * (1.0 - ma / mt) if mt > 0 else -1.0,
                    100.0 * (1.0 - sf / st) if st > 0 else 0.0)
        except Exception:
            return (-1.0, -1.0)

    def _sample_gpu(self):
        """(util %, mem used MB) of GPU 0 via nvidia-smi, ('','') if unavailable."""
        try:
            out = subprocess.check_output(
                ["nvidia-smi", "--query-gpu=utilization.gpu,memory.used",
                 "--format=csv,noheader,nounits"],
                stderr=subprocess.DEVNULL, timeout=2.0).decode().splitlines()
            util, mem = (x.strip() for x in out[0].split(","))
            return (util, mem)
        except Exception:
            return ("", "")

    def _read_gov_freq(self):
        gov = mhz = "?"
        try:
            gov = open("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor").read().strip()
        except Exception:
            pass
        try:
            mhz = str(int(open(
                "/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq").read().strip()) // 1000)
        except Exception:
            pass
        return (gov, mhz)

    def _check_nodes(self):
        """Diff the set of watched nav nodes; flag one vanishing/appearing (a
        relaunch window). Same-name respawns are caught by odom_origin_reset."""
        try:
            import rosnode
            names = rosnode.get_node_names()
        except Exception:
            return
        watched = ("cartographer", "zed", "move_base", "rplidar",
                   "cmd_vel_bridge", "shared_autonomy", "base_server")
        cur = {n for n in names if any(w in n for w in watched)}
        if self.node_set is not None:
            gone = self.node_set - cur
            new = cur - self.node_set
            if gone:
                self.event("node_gone", len(gone), ";".join(sorted(gone))[:100])
            if new:
                self.event("node_new", len(new), ";".join(sorted(new))[:100])
        self.node_set = cur

    def _spawn_dmesg_tail(self):
        """Best-effort follow of kernel USB/xHCI/Thunderbolt/reset lines (the
        open ZED corrupted-frame cause). No-op if dmesg is denied."""
        path = os.path.join(self.outdir, "usb_dmesg.log")
        try:
            back = subprocess.check_output(
                "dmesg -T 2>/dev/null | grep -iE 'usb|xhci|thunderbolt|reset' | tail -50",
                shell=True, stderr=subprocess.DEVNULL).decode()
            with open(path, "w") as f:
                f.write(back)
            if not back.strip():
                self.event("dmesg_unavailable", 0,
                           "dmesg empty/denied (kernel.dmesg_restrict?)")
        except Exception:
            pass
        try:
            f = open(path, "a")
            return subprocess.Popen(
                "dmesg -T -w 2>/dev/null | stdbuf -oL grep -iE "
                "'usb|xhci|thunderbolt|reset'",
                shell=True, stdout=f, stderr=subprocess.DEVNULL)
        except Exception:
            return None

    def _trigger_bag(self, reason):
        """Record a short rosbag of raw topics around an anomaly, so the next
        incident is recoverable at full fidelity (quaternions, scans, IMU)."""
        now = time.time()
        self.bag_procs = [p for p in self.bag_procs if p.poll() is None]
        if self.bag_procs or self.bag_count >= TRIGGER_BAG_MAX:
            return
        if now - self.last_bag < TRIGGER_BAG_MIN_INTERVAL_S:
            return
        self.last_bag = now
        self.bag_count += 1
        prefix = os.path.join(self.outdir, "trigger_%d_%s" %
                              (int(now), reason.replace("/", "_")[:20]))
        topics = TRIGGER_BAG_TOPICS
        if self.bag_include_scans:
            topics = topics + " " + TRIGGER_BAG_SCAN_TOPICS
        try:
            p = subprocess.Popen(
                "rosbag record -O %s --duration=%d %s" %
                (prefix, TRIGGER_BAG_DURATION_S, topics),
                shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.bag_procs.append(p)
            self.f_ev.write("%.3f,trigger_bag,%d,%s\n" %
                            (now, self.bag_count, prefix + ".bag"))
        except Exception:
            pass

    def _shutdown(self):
        for p in list(self.status_procs.values()) + [self.dmesg_proc] + self.bag_procs:
            if p:
                try:
                    p.terminate()
                except Exception:
                    pass

    def event(self, etype, value, detail=""):
        now = time.time()
        if now - self.last_event.get(etype, 0.0) < EVENT_THROTTLE_S:
            return
        self.last_event[etype] = now
        self.f_ev.write("%.3f,%s,%.4f,%s\n" % (now, etype, value, detail))
        if etype in TRIGGER_BAG_ON:
            self._trigger_bag(etype)

    # ---- raw ZED odom: jumps / gaps / drift ---------------------------------
    def cb_odom(self, msg):
        t = msg.header.stamp.to_sec()
        p = msg.pose.pose.position
        roll, pitch, yaw = rpy_from_quat(msg.pose.pose.orientation)
        cov = msg.pose.covariance
        cov_trace = cov[0] + cov[7] + cov[14] + cov[21] + cov[28] + cov[35]
        dstep = dyaw = dt = 0.0
        if self.prev_odom is not None:
            pt, px, py, pz, pyaw = self.prev_odom
            dt = t - pt
            dstep = math.sqrt((p.x - px) ** 2 + (p.y - py) ** 2 + (p.z - pz) ** 2)
            dyaw = abs(angle_diff(yaw, pyaw))
            if dt > ODOM_GAP_S:
                self.event("odom_gap", dt, "stamp gap")
            if dt <= 0.0:
                self.event("odom_stamp_regression", dt, "stamp went backward (restart?)")
            if dstep > ODOM_JUMP_M or dyaw > ODOM_JUMP_RAD:
                self.event("odom_jump", dstep,
                           "dyaw=%.3f dt=%.3f implied=%.2fm/s" %
                           (dyaw, dt, dstep / dt if dt > 0 else -1))
            # was well away from origin, now snapped to ~0 == ZED/odom re-init
            prev_norm = math.sqrt(px * px + py * py + pz * pz)
            cur_norm = math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z)
            if prev_norm > ORIGIN_RESET_M and cur_norm < 0.05:
                self.event("odom_origin_reset", prev_norm,
                           "odom reset to origin (ZED/odom node restart)")
        # Pitch/roll tilt of the odom frame: invisible in yaw-only logs; this is
        # the 2026-07-08 gravity-init failure signature.
        if abs(roll) > ODOM_TILT_RAD or abs(pitch) > ODOM_TILT_RAD:
            self.event("odom_tilt", max(abs(roll), abs(pitch)),
                       "roll=%.1fdeg pitch=%.1fdeg" %
                       (math.degrees(roll), math.degrees(pitch)))
        # ZED's native twist alongside the pose: whether the SDK's own velocity
        # stays sane through a re-init teleport is exactly the open question.
        tw = msg.twist.twist
        self.f_odom.write("%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.4f,%.4f,%.4f,"
                          "%.4f,%.4f,%.5f\n" %
                          (t, p.x, p.y, p.z, yaw, dstep, dyaw, dt,
                           tw.linear.x, tw.linear.y, tw.angular.z,
                           roll, pitch, cov_trace))
        self.prev_odom = (t, p.x, p.y, p.z, yaw)

    # ---- sanitizer output (what Cartographer consumes) -----------------------
    def cb_san(self, msg):
        p = msg.pose.pose.position
        roll, pitch, yaw = rpy_from_quat(msg.pose.pose.orientation)
        self.f_san.write("%.3f,%.4f,%.4f,%.4f,%.4f,%.4f\n" %
                         (msg.header.stamp.to_sec(), p.x, p.y, yaw, roll, pitch))

    # ---- wheel odometry: independent motion witness --------------------------
    def cb_wheel(self, msg):
        p = msg.pose.pose.position
        self.f_wheel.write("%.3f,%.4f,%.4f,%.4f,%.4f\n" %
                           (msg.header.stamp.to_sec(), p.x, p.y,
                            yaw_from_quat(msg.pose.pose.orientation),
                            self.last_side_disagree))

    # ---- actual command round-trip latency (from the bridge) -----------------
    def cb_latency(self, msg):
        self.f_lat.write("%.3f,%.5f\n" % (time.time(), msg.data))
        if msg.data > RPC_SLOW_S:
            self.event("rpc_slow", msg.data,
                       "set_speeds RPC round-trip %.1fms" % (msg.data * 1000.0))

    # ---- safety-hold interlock timeline --------------------------------------
    def cb_hold(self, msg):
        held = bool(msg.data)
        if held == self.hold_state:
            return
        first = self.hold_state is None
        self.hold_state = held
        reason = self.hold_reason.replace('"', "'")
        self.hold_reason_logged = reason
        self.f_hold.write('%.3f,%d,"%s"\n' % (time.time(), int(held), reason))
        if first and not held:
            return  # latched startup False = baseline row, not an event
        self.event("hold_assert" if held else "hold_release", float(held),
                   reason.replace(",", ";")[:120])

    def cb_hold_reason(self, msg):
        self.hold_reason = msg.data
        # The reason can land just after the Bool, or change mid-hold
        # (escalation); append a row so hold.csv always carries the full cause.
        if self.hold_state and msg.data and msg.data != self.hold_reason_logged:
            self.hold_reason_logged = msg.data
            self.f_hold.write('%.3f,1,"%s"\n' %
                              (time.time(), msg.data.replace('"', "'")))

    # ---- velocity feedback (what TEB sees) ----------------------------------
    def cb_fb(self, msg):
        tw = msg.twist.twist
        self.f_fb.write("%.3f,%.4f,%.4f,%.4f\n" %
                        (msg.header.stamp.to_sec(), tw.linear.x, tw.linear.y, tw.angular.z))

    # ---- commanded velocity: hunting detection ------------------------------
    def cb_cmd(self, msg):
        now = time.time()
        self.f_cmd.write("%.3f,%.4f,%.4f\n" % (now, msg.linear.x, msg.angular.z))
        if abs(msg.linear.x) > CMD_FLIP_V:
            while self.cmd_hist and now - self.cmd_hist[0][0] > CMD_FLIP_WINDOW_S:
                self.cmd_hist.popleft()
            if any(v * msg.linear.x < 0 for _, v in self.cmd_hist):
                self.event("cmd_flip", msg.linear.x, "vx sign flip (hunting)")
            self.cmd_hist.append((now, msg.linear.x))

    # ---- applied velocity (bridge output, post stiction-floor) --------------
    def cb_applied(self, msg):
        self.f_app.write("%.3f,%.4f,%.4f\n" % (time.time(), msg.linear.x, msg.angular.z))

    # ---- 10 Hz TF sample: cartographer correction + latency -----------------
    def cb_tf_sample(self, _):
        now = time.time()
        mo = ob_age = mb = None
        try:
            tr = self.buf.lookup_transform("map", "odom", rospy.Time(0))
            t = tr.transform.translation
            mo = (t.x, t.y, yaw_from_quat(tr.transform.rotation))
        except Exception:
            pass
        ob_rp = None
        try:
            tr = self.buf.lookup_transform("odom", "base_link", rospy.Time(0))
            ob_age = (rospy.Time.now() - tr.header.stamp).to_sec()
            r, pch, _ = rpy_from_quat(tr.transform.rotation)
            ob_rp = (r, pch)
        except Exception:
            pass
        try:
            tr = self.buf.lookup_transform("map", "base_link", rospy.Time(0))
            t = tr.transform.translation
            mb = (t.x, t.y, yaw_from_quat(tr.transform.rotation))
        except Exception:
            pass

        if mo is not None and self.prev_mo is not None:
            d = math.hypot(mo[0] - self.prev_mo[0], mo[1] - self.prev_mo[1])
            dy = abs(angle_diff(mo[2], self.prev_mo[2]))
            if d > MAP_ODOM_JUMP_M or dy > MAP_ODOM_JUMP_RAD:
                self.event("map_odom_yank", d, "dyaw=%.3f (localization correction)" % dy)
        if mo is not None:
            self.prev_mo = mo
        if ob_age is not None and ob_age > TF_STALE_S:
            self.event("tf_stale", ob_age, "odom->base_link age")

        # Continuous plan-tracking: how far / off-heading the base is from the
        # global plan. Quantifies "TEB overshoots the path" instead of inferring
        # it from cmd_vel sign flips.
        if mb is not None and self.plan_pts:
            bx, by, byaw = mb
            best_i, best_d = 0, float("inf")
            for i, (qx, qy) in enumerate(self.plan_pts):
                d = (bx - qx) ** 2 + (by - qy) ** 2
                if d < best_d:
                    best_d, best_i = d, i
            cross = math.sqrt(best_d)
            seg_yaw = self.plan_yaws[best_i] if best_i < len(self.plan_yaws) else byaw
            herr = abs(angle_diff(byaw, seg_yaw))
            self.f_track.write("%.3f,%.4f,%.4f,%d\n" %
                               (now, cross, herr, len(self.plan_pts)))
            if cross > PLAN_DEV_M:
                self.event("plan_deviation", cross, "base %.2fm off global plan" % cross)

        self.f_tf.write("%.3f,%s,%s,%s,%s\n" % (
            now,
            "%.4f,%.4f,%.4f" % mo if mo else ",,",
            "%.3f" % ob_age if ob_age is not None else "",
            "%.4f,%.4f,%.4f" % mb if mb else ",,",
            "%.4f,%.4f" % ob_rp if ob_rp else ","))

    # ---- localization map: capture the grid this run localized against -------
    def cb_map(self, msg):
        if self.map_saved:
            return
        try:
            self._save_occupancy_grid(msg)
            self.map_saved = True
            rospy.loginfo("nav_diag_logger: saved map.pgm/map.yaml (%dx%d @ %.3fm)",
                          msg.info.width, msg.info.height, msg.info.resolution)
        except Exception as e:  # noqa: BLE001
            rospy.logwarn("nav_diag_logger: failed to save /map: %s", e)

    def _save_occupancy_grid(self, msg):
        """Write the OccupancyGrid as a map_server pgm+yaml (trinary: occupied=0,
        free=254, unknown=205). PGM rows run top=+y down, so y is flipped."""
        W, H = msg.info.width, msg.info.height
        res = msg.info.resolution
        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y
        oyaw = yaw_from_quat(msg.info.origin.orientation)
        data = msg.data
        lut = {v: (0 if v >= 65 else (254 if 0 <= v <= 20 else 205))
               for v in range(-1, 101)}
        px = bytearray(W * H)
        for r in range(H):
            my = H - 1 - r  # PGM top row = highest map y
            row = data[my * W:(my + 1) * W]
            px[r * W:(r + 1) * W] = bytes(lut.get(v, 205) for v in row)
        with open(os.path.join(self.outdir, "map.pgm"), "wb") as f:
            f.write(("P5\n%d %d\n255\n" % (W, H)).encode())
            f.write(bytes(px))
        with open(os.path.join(self.outdir, "map.yaml"), "w") as f:
            f.write("image: map.pgm\nresolution: %.6f\n" % res)
            f.write("origin: [%.6f, %.6f, %.6f]\n" % (ox, oy, oyaw))
            f.write("negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")

    # ---- local costmap: phantom-obstacle evidence ---------------------------
    def cb_grid(self, msg):
        self.grid = list(msg.data)
        self.grid_info = (msg.info.width, msg.info.height, msg.info.resolution,
                          msg.info.origin.position.x, msg.info.origin.position.y,
                          msg.header.frame_id)

    def cb_grid_update(self, msg):
        if self.grid is None or self.grid_info is None:
            return
        w = self.grid_info[0]
        for row in range(msg.height):
            dst = (msg.y + row) * w + msg.x
            src = row * msg.width
            self.grid[dst:dst + msg.width] = msg.data[src:src + msg.width]

    def cb_costmap_sample(self, _):
        if self.grid is None:
            return
        w, h, res, ox, oy, frame = self.grid_info
        try:
            # vention_base_link = the costmaps' robot_base_frame (footprint center)
            tr = self.buf.lookup_transform(frame, "vention_base_link", rospy.Time(0))
            rx, ry = tr.transform.translation.x, tr.transform.translation.y
        except Exception:
            return
        ix = int((rx - ox) / res)
        iy = int((ry - oy) / res)
        if not (0 <= ix < w and 0 <= iy < h):
            return
        robot_cost = self.grid[iy * w + ix]
        r = int(FOOTPRINT_RADIUS_M / res)
        mx = 0
        for j in range(max(0, iy - r), min(h, iy + r + 1)):
            row = self.grid[j * w + max(0, ix - r):j * w + min(w, ix + r + 1)]
            m = max(row) if row else 0
            if m > mx:
                mx = m
        self.f_cost.write("%.3f,%d,%d\n" % (time.time(), robot_cost, mx))

        if self.want_dump and time.time() - self.last_dump > COSTMAP_DUMP_THROTTLE_S:
            self.want_dump = False
            self.last_dump = time.time()
            path = os.path.join(self.outdir, "costmap_dump_%d.txt" % int(self.last_dump))
            with open(path, "w") as f:
                f.write("# t=%.3f frame=%s w=%d h=%d res=%.3f ox=%.3f oy=%.3f "
                        "robot=%.3f,%.3f\n" % (self.last_dump, frame, w, h, res, ox, oy, rx, ry))
                for j in range(h):
                    f.write(",".join(str(v) for v in self.grid[j * w:(j + 1) * w]) + "\n")
            self.event("costmap_dump", mx, path)

    # ---- global plan churn ---------------------------------------------------
    def cb_plan(self, msg):
        length = 0.0
        pts = msg.poses
        for a, b in zip(pts, pts[1:]):
            length += math.hypot(b.pose.position.x - a.pose.position.x,
                                 b.pose.position.y - a.pose.position.y)
        self.f_plan.write("%.3f,%d,%.3f\n" % (time.time(), len(pts), length))
        # Cache the plan geometry for continuous cross-track/heading error (cb_tf_sample).
        self.plan_pts = [(ps.pose.position.x, ps.pose.position.y) for ps in pts]
        yaws = [math.atan2(b.pose.position.y - a.pose.position.y,
                           b.pose.position.x - a.pose.position.x)
                for a, b in zip(pts, pts[1:])]
        if yaws:
            yaws.append(yaws[-1])
        self.plan_yaws = yaws

    # ---- IMU vibration aggregate (1 Hz) --------------------------------------
    def cb_imu(self, msg):
        g = msg.angular_velocity
        a = msg.linear_acceleration
        gyro = math.sqrt(g.x ** 2 + g.y ** 2 + g.z ** 2)
        accel_dev = abs(math.sqrt(a.x ** 2 + a.y ** 2 + a.z ** 2) - 9.81)
        # Keep the raw accel vector (gravity direction) + fused orientation so a
        # gravity/vertical-init error is directly visible, not just its RMS.
        o = msg.orientation
        if msg.orientation_covariance[0] >= 0.0 and (o.x or o.y or o.z or o.w):
            ori_roll, ori_pitch, _ = rpy_from_quat(o)
        else:
            ori_roll = ori_pitch = float("nan")
        self.imu_acc.append((gyro, accel_dev, a.x, a.y, a.z, ori_roll, ori_pitch))

    def cb_slow(self, _):
        now = time.time()
        # IMU 1 Hz flush
        if self.imu_acc:
            n = len(self.imu_acc)
            rg = math.sqrt(sum(s[0] * s[0] for s in self.imu_acc) / n)
            ra = math.sqrt(sum(s[1] * s[1] for s in self.imu_acc) / n)
            ax = sum(s[2] for s in self.imu_acc) / n
            ay = sum(s[3] for s in self.imu_acc) / n
            az = sum(s[4] for s in self.imu_acc) / n
            # gravity-derived tilt (accel opposes gravity when ~static)
            grav_roll = math.atan2(ay, az)
            grav_pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
            oris = [s[5] for s in self.imu_acc if not math.isnan(s[5])]
            opis = [s[6] for s in self.imu_acc if not math.isnan(s[6])]
            ori_roll = sum(oris) / len(oris) if oris else float("nan")
            ori_pitch = sum(opis) / len(opis) if opis else float("nan")
            self.f_imu.write("%.3f,%.4f,%.4f,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n" %
                             (now, rg, ra, n, grav_roll, grav_pitch,
                              ori_roll, ori_pitch, ax, ay, az))
            self.imu_acc = []
        # scan rates + system load/thermal (USB failures, throttling)
        nl, nr = self.scan_n
        self.scan_n = [0, 0]
        temp = -1.0
        try:
            temps = [int(open(p).read()) / 1000.0
                     for p in glob.glob("/sys/class/thermal/thermal_zone*/temp")]
            temp = max(temps) if temps else -1.0
        except Exception:
            pass
        cpu_pct, cpu_max = self._sample_cpu()
        mem_pct, swap_pct = self._sample_mem()
        if self.heartbeat % GPU_SAMPLE_EVERY == 0:
            self.gpu_cache = self._sample_gpu()
        gpu_util, gpu_mem = self.gpu_cache
        gov, mhz = self._read_gov_freq()
        self.f_sys.write("%.3f,%d,%d,%.2f,%.1f,%.1f,%.1f,%.1f,%.1f,%s,%s,%s,%s\n" %
                         (now, nl, nr, os.getloadavg()[0], temp,
                          cpu_pct, cpu_max, mem_pct, swap_pct,
                          gpu_util, gpu_mem, gov, mhz))
        if nl == 0 or nr == 0:
            self.event("scan_gap", nl + nr, "lidar_l=%d lidar_r=%d msgs/s" % (nl, nr))
        # watched-node set diff (catches a node vanishing/appearing in a relaunch)
        if self.heartbeat % NODE_CHECK_EVERY == 0:
            self._check_nodes()
        # BT timeline tail
        path = os.path.join(LOG_BASE, "execution_log.txt")
        try:
            size = os.path.getsize(path)
            if size > self.bt_pos:
                with open(path) as f:
                    f.seek(self.bt_pos)
                    for line in f.read().splitlines():
                        if line.strip():
                            self.f_bt.write('%.3f,"%s"\n' %
                                            (now, line.strip().replace('"', "'")))
                self.bt_pos = size
            elif size < self.bt_pos:
                self.bt_pos = 0  # file truncated/recreated: read the fresh run from its start
        except OSError:
            pass
        # params snapshot: retry until move_base + ZED params are all captured
        if not self.snap_complete and now - self.snap_last_try > 30.0:
            self.snap_last_try = now
            self.snap_complete = self._snapshot_params()
        # zed_status echoes: respawn any that died (started before sensors, or
        # sensors relaunched mid-run)
        if now - self.status_last_try > 30.0:
            for t, fname in self.status_topics:
                p = self.status_procs.get(t)
                if p is None or p.poll() is not None:
                    self.status_last_try = now
                    self.status_procs[t] = self._spawn_status_echo(t, fname)
        # One-shot diagnostic: if neither status echo has produced a byte after
        # 120 s, say so -- distinguishes "no tracking-state transitions" from
        # "the echo/topic is broken" when reading the run afterwards.
        if not self.status_diag_done and now - self.start_wall > 120.0:
            self.status_diag_done = True
            try:
                if all(os.path.getsize(os.path.join(self.outdir, f)) == 0
                       for _, f in self.status_topics):
                    self.event("zed_status_silent", 0,
                               "no PosTrackStatus captured in 120s "
                               "(on-change-only topic or echo broken)")
            except OSError:
                pass
        # liveness heartbeat in the pane
        self.heartbeat += 1
        if self.heartbeat % 60 == 0:
            rospy.loginfo("nav_diag_logger alive: %d min, %d event types flagged",
                          self.heartbeat // 60, len(self.last_event))

    # ---- goals: duration, outcome, terminal + settled residual ---------------
    def _residual(self, gx, gy, gyaw):
        try:
            # vention_base_link = move_base's robot_base_frame; residuals vs
            # base_link carry the ~0.3 m lever arm between the two frames.
            tr = self.buf.lookup_transform("map", "vention_base_link", rospy.Time(0))
            t = tr.transform.translation
            return (math.hypot(t.x - gx, t.y - gy),
                    abs(angle_diff(yaw_from_quat(tr.transform.rotation), gyaw)))
        except Exception:
            return (float("nan"), float("nan"))

    def cb_goal_pose(self, msg):
        p = msg.pose.position
        yaw = yaw_from_quat(msg.pose.orientation)
        self.goal = ((p.x, p.y, yaw), time.time())
        self.f_goal.write("%.3f,start,%.4f,%.4f,%.4f,,,\n" % (time.time(), p.x, p.y, yaw))

    def cb_status(self, msg):
        if self.goal is None or not msg.status_list:
            return
        st = msg.status_list[-1].status
        if st in (3, 4, 2):  # SUCCEEDED / ABORTED / PREEMPTED
            (gx, gy, gyaw), t0 = self.goal
            self.goal = None
            name = {3: "succeeded", 4: "aborted", 2: "preempted"}[st]
            if st == 4:
                self.want_dump = True
            res_xy, res_yaw = self._residual(gx, gy, gyaw)
            self.f_goal.write("%.3f,%s,%.4f,%.4f,%.4f,%.1f,%.4f,%.4f\n" %
                              (time.time(), name, gx, gy, gyaw,
                               time.time() - t0, res_xy, res_yaw))
            if st == 3:
                rospy.Timer(rospy.Duration(SETTLE_S),
                            lambda _e, g=(gx, gy, gyaw): self._log_settled(g),
                            oneshot=True)

    def _log_settled(self, g):
        gx, gy, gyaw = g
        res_xy, res_yaw = self._residual(gx, gy, gyaw)
        self.f_goal.write("%.3f,settled,%.4f,%.4f,%.4f,,%.4f,%.4f\n" %
                          (time.time(), gx, gy, gyaw, res_xy, res_yaw))

    # ---- rosout: WARN+ always; INFO on keywords ------------------------------
    def cb_rosout(self, msg):
        if not any(k in msg.name for k in ROSOUT_NODES):
            return
        keep = msg.level >= 4 or (
            msg.level == 2 and any(k in msg.msg for k in ROSOUT_INFO_KEYWORDS))
        if not keep:
            return
        if "not feasible" in msg.msg:
            self.want_dump = True
            self.event("teb_infeasible", 1, "trajectory not feasible")
        if "CORRUPTED" in msg.msg or "grab error" in msg.msg:
            self.event("zed_corrupted_frame", 1, msg.msg[:80])
        # Sanitizer decisions (zed_pose_to_odom_feedback WARNs) as events, so
        # held/adopted frames correlate against odom_jump/hold rows directly.
        if "odom sanitizer: rejected" in msg.msg:
            self.event("sanitizer_reject", 1, msg.msg[:80].replace(",", ";"))
        elif "adopting sustained" in msg.msg:
            self.event("sanitizer_adopt", 1, msg.msg[:80].replace(",", ";"))
        self.f_ros.write("%.3f %s %s %s\n" %
                         (msg.header.stamp.to_sec(), LEVELS.get(msg.level, "?"),
                          msg.name, msg.msg.replace("\n", " ")[:300]))


def main():
    # Tolerate starting before roscore (logger pane comes up with the tmux
    # session, ahead of the bringup panes).
    try:
        import rosgraph
        while not rosgraph.is_master_online():
            time.sleep(2.0)
    except ImportError:
        pass
    rospy.init_node("nav_diag_logger")
    NavDiagLogger()
    rospy.spin()


if __name__ == "__main__":
    main()
