#!/usr/bin/env python3
"""
costmap_collision_probe.py — isolate collision/costmap behavior in a tight space.

Monitors the local costmap under the robot footprint so you can see, directly,
whether the robot is skirting lethal/inflated cells (the "collides sometimes" and
"TEB trajectory not feasible" symptom). It rasterizes the footprint polygon over
the local costmap each tick and reports:
  * cost at the robot center
  * MAX cost under the footprint      (100 = lethal, 99 = inscribed)
  * min clearance to the nearest lethal cell (m)
  * running count of TEB "not feasible / Resetting" warnings (from /rosout_agg)

Default is MONITOR-only (no motion) — teleop the base slowly through the tight
spot yourself, or pass --drive to creep straight forward via /cmd_vel_teleop
(hold-exempt) for --distance metres at --speed.

The local costmap is in the `odom` frame (rolling window), so this transforms the
footprint using TF to the grid's own header.frame_id — no assumption baked in.

Examples:
  rosrun feeding_deployment costmap_collision_probe.py            # monitor while you teleop
  rosrun feeding_deployment costmap_collision_probe.py --drive --distance 0.8 --speed 0.1
"""

import argparse
import csv
import math
import sys
import time
from pathlib import Path


def _parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--costmap-topic", default="/move_base/local_costmap/costmap")
    p.add_argument("--footprint-param", default="/move_base/local_costmap/footprint")
    p.add_argument("--base-frame", default="vention_base_link")
    p.add_argument("--rate", type=float, default=5.0, help="sample rate Hz")
    p.add_argument("--duration", type=float, default=60.0, help="monitor duration (s)")
    p.add_argument("--scan-radius", type=float, default=3.0, help="lethal-cell search radius (m)")
    p.add_argument("--lethal", type=int, default=99, help="cost >= this counts as blocking (99=inscribed)")
    p.add_argument("--drive", action="store_true", help="creep straight forward via /cmd_vel_teleop")
    p.add_argument("--speed", type=float, default=0.1, help="creep speed (m/s) when --drive")
    p.add_argument("--distance", type=float, default=0.8, help="creep distance (m) when --drive")
    p.add_argument("--out-dir", default=None, help="CSV dir (default <repo>/nav_tuning_logs)")
    p.add_argument("--label", default="tight")
    return p.parse_args()


_DEFAULT_FOOTPRINT = [[0.37, 0.295], [0.37, -0.295], [-0.37, -0.295], [-0.37, 0.295]]


def _yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _point_in_poly(x, y, poly):
    """Ray-cast point-in-polygon (poly = list of (x,y))."""
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi):
            inside = not inside
        j = i
    return inside


def main():
    args = _parse_args()

    try:
        import rospy
        import tf2_ros
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import OccupancyGrid
        from rosgraph_msgs.msg import Log
    except Exception as e:  # noqa: BLE001
        sys.stderr.write(f"ROS import failed (run inside your ROS env): {e}\n")
        return 2

    rospy.init_node("costmap_collision_probe", anonymous=True, disable_signals=True)

    repo_root = Path(__file__).resolve().parents[1]
    out_dir = Path(args.out_dir) if args.out_dir else repo_root / "nav_tuning_logs"
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / f"costmap_{args.label}_{time.strftime('%Y%m%d_%H%M%S')}.csv"

    # footprint (base frame)
    fp = _DEFAULT_FOOTPRINT
    try:
        raw = rospy.get_param(args.footprint_param)
        if isinstance(raw, str):
            import yaml
            raw = yaml.safe_load(raw)
        if isinstance(raw, list) and raw:
            fp = [(float(v[0]), float(v[1])) for v in raw]
    except Exception:  # noqa: BLE001
        rospy.logwarn("footprint param %s unavailable; using default", args.footprint_param)

    grid = {"g": None}

    def _on_grid(msg):
        grid["g"] = msg

    def _on_rosout(msg):
        t = msg.msg or ""
        if "not feasible" in t or "Resetting" in t:
            state["infeasible"] += 1

    state = {"infeasible": 0}
    rospy.Subscriber(args.costmap_topic, OccupancyGrid, _on_grid, queue_size=1)
    rospy.Subscriber("/rosout_agg", Log, _on_rosout, queue_size=200)
    pub = rospy.Publisher("/cmd_vel_teleop", Twist, queue_size=1) if args.drive else None

    tf_buf = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
    tf2_ros.TransformListener(tf_buf)

    def base_in_grid(frame):
        try:
            tf = tf_buf.lookup_transform(frame, args.base_frame,
                                         rospy.Time(0), rospy.Duration(0.1))
        except Exception:  # noqa: BLE001
            return None
        t = tf.transform.translation
        return t.x, t.y, _yaw_from_quat(tf.transform.rotation)

    def analyze():
        """Return (center_cost, max_fp_cost, min_clearance_m) for this tick."""
        g = grid["g"]
        if g is None:
            return None
        frame = g.header.frame_id
        bp = base_in_grid(frame)
        if bp is None:
            return None
        bx, by, byaw = bp
        res = g.info.resolution
        ox, oy = g.info.origin.position.x, g.info.origin.position.y
        W, H = g.info.width, g.info.height
        data = g.data

        def cost_at(wx, wy):
            cx = int((wx - ox) / res)
            cy = int((wy - oy) / res)
            if 0 <= cx < W and 0 <= cy < H:
                return data[cy * W + cx]
            return -1

        center_cost = cost_at(bx, by)

        # footprint polygon in grid frame
        c, s = math.cos(byaw), math.sin(byaw)
        poly = [(bx + fx * c - fy * s, by + fx * s + fy * c) for fx, fy in fp]
        xs = [px for px, _ in poly]
        ys = [py for _, py in poly]
        max_fp = -1
        cx0 = int((min(xs) - ox) / res)
        cx1 = int((max(xs) - ox) / res)
        cy0 = int((min(ys) - oy) / res)
        cy1 = int((max(ys) - oy) / res)
        for cy in range(max(0, cy0), min(H, cy1 + 1)):
            wy = oy + (cy + 0.5) * res
            for cx in range(max(0, cx0), min(W, cx1 + 1)):
                wx = ox + (cx + 0.5) * res
                if _point_in_poly(wx, wy, poly):
                    v = data[cy * W + cx]
                    if v > max_fp:
                        max_fp = v

        # min clearance to a blocking cell within scan-radius
        rr = int(args.scan_radius / res)
        bcx = int((bx - ox) / res)
        bcy = int((by - oy) / res)
        min_d = float("inf")
        for cy in range(max(0, bcy - rr), min(H, bcy + rr + 1)):
            for cx in range(max(0, bcx - rr), min(W, bcx + rr + 1)):
                if data[cy * W + cx] >= args.lethal:
                    wx = ox + (cx + 0.5) * res
                    wy = oy + (cy + 0.5) * res
                    d = math.hypot(wx - bx, wy - by)
                    if d < min_d:
                        min_d = d
        return center_cost, max_fp, (min_d if min_d != float("inf") else float("nan"))

    def stop():
        if pub is not None:
            for _ in range(5):
                pub.publish(Twist())
                rospy.sleep(0.02)

    rospy.loginfo("costmap_collision_probe: %s topic=%s footprint=%d pts -> %s",
                  "DRIVE" if args.drive else "MONITOR", args.costmap_topic, len(fp), csv_path)

    rate = rospy.Rate(args.rate)
    rows = []
    start = rospy.Time.now()
    origin_xy = None
    rospy.sleep(0.5)

    try:
        while not rospy.is_shutdown():
            now = (rospy.Time.now() - start).to_sec()
            if now > args.duration:
                break
            res = analyze()
            if res is not None:
                cc, mf, clr = res
                rows.append({
                    "t": round(now, 3),
                    "center_cost": cc, "max_footprint_cost": mf,
                    "min_clearance_m": round(clr, 4) if not math.isnan(clr) else clr,
                    "infeasible_cum": state["infeasible"],
                })
            if args.drive and pub is not None:
                # creep forward; stop at distance (measured by odom->base if available)
                bp = base_in_grid(grid["g"].header.frame_id) if grid["g"] else None
                if bp is not None:
                    if origin_xy is None:
                        origin_xy = (bp[0], bp[1])
                    travelled = math.hypot(bp[0] - origin_xy[0], bp[1] - origin_xy[1])
                    if travelled >= args.distance:
                        rospy.loginfo("costmap_collision_probe: reached %.2fm, stopping", travelled)
                        break
                tw = Twist()
                tw.linear.x = args.speed
                pub.publish(tw)
            rate.sleep()
    finally:
        stop()

    fields = ["t", "center_cost", "max_footprint_cost", "min_clearance_m", "infeasible_cum"]
    with open(csv_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)

    if rows:
        n = len(rows)
        lethal_fp = sum(1 for r in rows if r["max_footprint_cost"] >= args.lethal)
        clears = [r["min_clearance_m"] for r in rows
                  if isinstance(r["min_clearance_m"], float) and not math.isnan(r["min_clearance_m"])]
        min_clr = min(clears) if clears else float("nan")
        max_center = max(r["center_cost"] for r in rows)
        summary = (f"samples={n}  footprint held a blocking cell {100.0*lethal_fp/n:.1f}% of time  "
                   f"min clearance={min_clr*100:.1f}cm  max center cost={max_center}  "
                   f"TEB infeasible warnings during probe={state['infeasible']}")
    else:
        summary = "no samples (costmap or TF unavailable)"
    rospy.loginfo("costmap_collision_probe DONE. %s", summary)
    print(f"[costmap_collision_probe] wrote {len(rows)} rows -> {csv_path}")
    print(f"[costmap_collision_probe] {summary}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
