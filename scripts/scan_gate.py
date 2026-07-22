#!/usr/bin/env python3
"""
scan_gate.py -- drop lidar scans too degenerate to localize on, so Cartographer
never sees them.

Why (session_20260714_171313_jul14_2, both table stays): while the robot sat
motionless at the table during feeding (wheel odom frozen for 11+ min), the
user/experimenter/arm occluded the lidars and Cartographer's scan-match nodes
collapsed from ~205 filtered points to 10-90. Its constraint score is the
FRACTION of points explained, unweighted by count, so a 10-point scan matched
submaps 4-7 m away at 90.0% (one 11-point node matched 8 different submaps,
all at 90.0%). 801/6246 accepted constraints in table stay #1 were >0.5 m
displaced -> pose-graph optimization (every 25 nodes ~ 12.5 s parked) yanked
map->odom 2-7 m every minute or two. Parked at the microwave: 1.3% bad
constraints, no storms. Post-mortem with figures:
log/system_logs/session_20260714_171313_jul14_2 analysis, 2026-07-15.

What this node does:

  /lidar_l/scan -> /lidar_l/scan_gated     (cartographer_localization.launch
  /lidar_r/scan -> /lidar_r/scan_gated      remaps scan_1/scan_2 to the gated
                                            topics; costmaps keep raw scans)

  - Per scan, compute COVERAGE: the number of occupied `cell_m`-sized 2D grid
    cells among valid returns in [range_lo, range_hi]. This mirrors what
    Cartographer's adaptive voxel filter feeds the matcher (spatial spread);
    raw valid-return count does NOT work -- an occluding torso still returns
    ~850 valid points, all piled into a handful of cells.
  - Gate OPEN: forward both scans untouched.
  - Gate CLOSED: forward NOTHING. Dropping only the bad lidar would desync
    Cartographer's ordered multi-queue (it stalls on the silent topic and
    dumps a stale backlog on resume), so both are always gated together.
  - OPEN -> CLOSED on a single unhealthy scan (coverage < min_cells) or on
    partner staleness (> stale_s). One dropped scan is harmless.
  - CLOSED -> OPEN only after BOTH lidars are continuously healthy for
    reopen_after_s (hysteresis; don't flap mid-bite).

Starved of range data, Cartographer creates no nodes and adds no constraints,
so map->odom freezes at its last good value and the pose rides wheel+IMU odom
(/odometry/fused_imu_wheel) -- which is exactly right for a parked robot:
zero information beats poisoned information. When the person leans away and
the scans clear up, localization resumes from a pose that is still correct.

Threshold calibration (trigger bags, jul14_2): healthy coverage at 0.2 m
cells is 36-114 (global min 36, mid-drive past a doorway); Cartographer's
numbers put the feeding-occlusion regime around 10-20. Default min_cells=25
splits that with ~30% margin on the healthy side. The degenerate side has NO
direct scan recording yet -- hence the 30 s stats lines below; read them from
the first table session with the gate running and retune min_cells if the
occluded coverage doesn't actually drop below it.

Diagnostics:
  - /scan_gate/open (Bool, latched) on every transition.
  - WARN on every transition with the offending coverage numbers.
  - INFO every stats_period_s: per-lidar coverage min/median over the window,
    forwarded/dropped counts, gate state. Lands in rosout / the session
    bundle's ros logs for post-hoc tuning.

Consumer: cartographer_node (localization mode) ONLY. Mapping keeps raw scans
-- you drive during mapping, and map builds want every return.
"""

import math
import threading

import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


def scan_coverage(msg, cell_m, range_lo, range_hi):
    """Occupied `cell_m` grid cells among valid returns in [range_lo, range_hi].

    Spatial-spread proxy for Cartographer's post-voxel-filter point count.
    """
    r = np.asarray(msg.ranges, dtype=np.float32)
    n = r.shape[0]
    a = msg.angle_min + msg.angle_increment * np.arange(n, dtype=np.float32)
    ok = np.isfinite(r) & (r >= max(range_lo, msg.range_min)) & (r <= range_hi)
    if not ok.any():
        return 0
    rr, aa = r[ok], a[ok]
    ix = np.floor(rr * np.cos(aa) / cell_m).astype(np.int32)
    iy = np.floor(rr * np.sin(aa) / cell_m).astype(np.int32)
    # pack the two int32 grid indices into one int64 for a single unique()
    return int(np.unique(ix.astype(np.int64) << 32 | (iy.astype(np.int64) & 0xFFFFFFFF)).shape[0])


class GateCore:
    """Pure gate logic (no ROS) so it can be replay-tested offline.

    Feed every scan via update(lidar, coverage, t); read .open for whether to
    forward. Times are float seconds (any epoch, must be monotonic-ish).
    """

    def __init__(self, lidars=("l", "r"), min_cells=25, reopen_after_s=2.0,
                 stale_s=1.0):
        self.min_cells = min_cells
        self.reopen_after_s = reopen_after_s
        self.stale_s = stale_s
        self.last_t = {k: None for k in lidars}       # last scan arrival
        self.last_cov = {k: None for k in lidars}     # last coverage
        self.open = False                             # start closed; opens
        self.healthy_since = None                     # after reopen_after_s
        self.transition = None                        # set by update() when
                                                      # state flips: (open, why)

    def _all_healthy(self, t):
        for k in self.last_t:
            if self.last_t[k] is None or t - self.last_t[k] > self.stale_s:
                return False, "%s stale" % k
            if self.last_cov[k] < self.min_cells:
                return False, "%s coverage %d < %d" % (k, self.last_cov[k],
                                                       self.min_cells)
        return True, ""

    def update(self, lidar, coverage, t):
        """Record one scan; returns True if the gate is open for it."""
        self.transition = None
        self.last_t[lidar] = t
        self.last_cov[lidar] = coverage
        healthy, why = self._all_healthy(t)
        if self.open:
            if not healthy:
                self.open = False
                self.healthy_since = None
                self.transition = (False, why)
        else:
            if healthy:
                if self.healthy_since is None:
                    self.healthy_since = t
                elif t - self.healthy_since >= self.reopen_after_s:
                    self.open = True
                    self.transition = (True, "healthy %.1fs" %
                                       (t - self.healthy_since))
            else:
                self.healthy_since = None
        return self.open


class ScanGateNode:
    def __init__(self):
        self.cell_m = rospy.get_param("~cell_m", 0.2)
        self.range_lo = rospy.get_param("~range_lo", 0.15)   # cartographer min_range
        self.range_hi = rospy.get_param("~range_hi", 12.0)   # cartographer max_range
        self.stats_period_s = rospy.get_param("~stats_period_s", 30.0)
        self.core = GateCore(
            min_cells=rospy.get_param("~min_cells", 25),
            reopen_after_s=rospy.get_param("~reopen_after_s", 2.0),
            stale_s=rospy.get_param("~stale_s", 1.0))
        self.lock = threading.Lock()
        self.n_fwd = {"l": 0, "r": 0}
        self.n_drop = {"l": 0, "r": 0}
        self.win_cov = {"l": [], "r": []}   # coverages since last stats line

        self.pub_open = rospy.Publisher("/scan_gate/open", Bool,
                                        queue_size=1, latch=True)
        self.pub = {
            "l": rospy.Publisher("/lidar_l/scan_gated", LaserScan, queue_size=5),
            "r": rospy.Publisher("/lidar_r/scan_gated", LaserScan, queue_size=5),
        }
        self.pub_open.publish(Bool(self.core.open))
        rospy.Subscriber("/lidar_l/scan", LaserScan,
                         self.cb_scan, callback_args="l", queue_size=5)
        rospy.Subscriber("/lidar_r/scan", LaserScan,
                         self.cb_scan, callback_args="r", queue_size=5)
        rospy.Timer(rospy.Duration(self.stats_period_s), self.cb_stats)
        rospy.loginfo("scan_gate: up (min_cells=%d cell=%.2fm reopen=%.1fs); "
                      "gate starts CLOSED until both lidars are healthy",
                      self.core.min_cells, self.cell_m,
                      self.core.reopen_after_s)

    def cb_scan(self, msg, lidar):
        cov = scan_coverage(msg, self.cell_m, self.range_lo, self.range_hi)
        now = rospy.Time.now().to_sec()
        with self.lock:
            forward = self.core.update(lidar, cov, now)
            tr = self.core.transition
            self.win_cov[lidar].append(cov)
            if forward:
                self.n_fwd[lidar] += 1
            else:
                self.n_drop[lidar] += 1
        if forward:
            self.pub[lidar].publish(msg)
        if tr is not None:
            opened, why = tr
            self.pub_open.publish(Bool(opened))
            if opened:
                rospy.logwarn("scan_gate: OPEN (%s) -- forwarding to "
                              "cartographer resumes", why)
            else:
                rospy.logwarn("scan_gate: CLOSED (%s) -- cartographer starved, "
                              "map->odom frozen on last good pose", why)

    def cb_stats(self, _):
        with self.lock:
            parts = []
            for k in ("l", "r"):
                v = sorted(self.win_cov[k])
                parts.append("%s cov min/med=%s/%s fwd=%d drop=%d" % (
                    k, v[0] if v else "-", v[len(v) // 2] if v else "-",
                    self.n_fwd[k], self.n_drop[k]))
                self.win_cov[k] = []
            state = "OPEN" if self.core.open else "CLOSED"
        rospy.loginfo("scan_gate: %s | %s", state, " | ".join(parts))


if __name__ == "__main__":
    rospy.init_node("scan_gate")
    ScanGateNode()
    rospy.spin()
