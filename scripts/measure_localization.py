#!/usr/bin/env python3
"""Measure localization quality by logging the map->base pose over time.

This is the diagnostic for picking navigation goal tolerances. It records the
robot pose in the map frame, but decomposed into its two TF links so the noise
can be attributed to a source:

  * map  -> odom               : Cartographer's correction (scan-matching vs the
                                 frozen .pbstream). Jitter here = lidar / map /
                                 environment-feature quality.
  * odom -> vention_base_link  : ZED VIO odometry. Jitter here = VIO drift.
  * map  -> vention_base_link  : the product -- exactly what NavigateHLA /
                                 move_base compare against when checking the goal.

Typical use (run once per goal station, robot held completely still):

    rosrun feeding_deployment measure_localization.py --label fridge_static
    # ... leave the robot stationary for ~2-3 minutes, then Ctrl-C ...

On exit it prints, for each link: mean, std, peak-to-peak, and the largest
single-step jump (the "correction jump" metric). Peak-to-peak on map->base is
your hard tolerance floor: no controller can hold a tolerance tighter than the
estimate itself jitters. A large single-step jump on map->odom is the signal
that goal-checking needs pose smoothing.

A full CSV is also written so the same run can be re-analyzed offline.
"""

from __future__ import annotations

import argparse
import math
import signal
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

try:
    import rospy
    import tf2_ros

    ROSPY_IMPORTED = True
except ModuleNotFoundError:
    ROSPY_IMPORTED = False


# Logical names for the three transforms we track.
LINKS = ("map2base", "map2odom", "odom2base")


def yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_diff(a: float, b: float) -> float:
    """Smallest signed difference a - b, wrapped to [-pi, pi]."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


def circular_mean(yaws: np.ndarray) -> float:
    return math.atan2(float(np.sin(yaws).mean()), float(np.cos(yaws).mean()))


class LocalizationMeasurer:
    def __init__(self, args: argparse.Namespace) -> None:
        self.map_frame = args.map_frame
        self.odom_frame = args.odom_frame
        self.base_frame = args.base_frame
        self.rate_hz = float(args.rate)
        self.lookup_timeout = float(args.lookup_timeout_s)
        self.duration = float(args.duration) if args.duration > 0 else None
        self.mode: str = args.mode  # "static" or "moving"
        self.settled_window_s = float(args.settled_window)
        self.settled_speed_m_s = float(args.settled_speed)

        self.out_path = self._resolve_out_path(args)

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self._tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # rows: list of (t, x, y, yaw) per logical link.
        self.rows: Dict[str, List[Tuple[float, float, float, float]]] = {
            link: [] for link in LINKS
        }
        self._start_wall: Optional[float] = None
        self._stop = False

    def _resolve_out_path(self, args: argparse.Namespace) -> Path:
        if args.out:
            return Path(args.out).expanduser().resolve()
        out_dir = Path(args.out_dir).expanduser().resolve()
        out_dir.mkdir(parents=True, exist_ok=True)
        # rospy.Time is used (not wall clock) so the filename is reproducible
        # within a bag-replay; fall back to a counter-free stable name.
        return out_dir / f"localization_{args.label}.csv"

    # ------------------------------------------------------------------ #
    # Acquisition
    # ------------------------------------------------------------------ #
    def _lookup(
        self, target: str, source: str
    ) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame=target,
                source_frame=source,
                time=rospy.Time(0),
                timeout=rospy.Duration(self.lookup_timeout),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
            tf2_ros.TimeoutException,
        ) as exc:
            rospy.logwarn_throttle(2.0, "TF %s<-%s failed: %s", target, source, exc)
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        return (float(t.x), float(t.y), yaw_from_quat(q.x, q.y, q.z, q.w))

    def spin(self) -> None:
        rate = rospy.Rate(self.rate_hz)
        rospy.loginfo(
            "measure_localization: logging %s<-%s (decomposed via %s) at %.0f Hz. "
            "Hold the robot STILL. Ctrl-C to stop.",
            self.map_frame,
            self.base_frame,
            self.odom_frame,
            self.rate_hz,
        )
        while not rospy.is_shutdown() and not self._stop:
            now = rospy.Time.now().to_sec()
            if self._start_wall is None:
                self._start_wall = now
            elapsed = now - self._start_wall

            samples = {
                "map2base": self._lookup(self.map_frame, self.base_frame),
                "map2odom": self._lookup(self.map_frame, self.odom_frame),
                "odom2base": self._lookup(self.odom_frame, self.base_frame),
            }
            for link, s in samples.items():
                if s is not None:
                    self.rows[link].append((elapsed, s[0], s[1], s[2]))

            mb = samples["map2base"]
            if mb is not None and self.rows["map2base"]:
                self._print_live(elapsed, mb)

            if self.duration is not None and elapsed >= self.duration:
                rospy.loginfo("Reached --duration %.1fs, stopping.", self.duration)
                break
            rate.sleep()

        self._finish()

    def _print_live(self, elapsed: float, mb: Tuple[float, float, float]) -> None:
        arr = np.asarray(self.rows["map2base"], dtype=float)
        x, y, yaw = arr[:, 1], arr[:, 2], arr[:, 3]
        n = len(arr)
        if self.mode == "moving":
            path_m = float(np.sum(np.hypot(np.diff(x), np.diff(y)))) if n >= 2 else 0.0
            if n >= 2:
                dt = arr[-1, 0] - arr[-2, 0]
                speed_cm_s = math.hypot(x[-1] - x[-2], y[-1] - y[-2]) / max(dt, 1e-6) * 100
            else:
                speed_cm_s = 0.0
            print(
                f"\r[{elapsed:6.1f}s] map->base x={mb[0]:+.4f} y={mb[1]:+.4f} "
                f"yaw={math.degrees(mb[2]):+7.2f}deg | "
                f"path={path_m:.2f}m  speed={speed_cm_s:.1f}cm/s  (n={n})   ",
                end="",
                flush=True,
            )
        else:
            xy_p2p = float(np.hypot(x - x.mean(), y - y.mean()).max() * 2.0)
            yaw_dev = np.array([angle_diff(v, circular_mean(yaw)) for v in yaw])
            yaw_p2p = float(yaw_dev.max() - yaw_dev.min())
            print(
                f"\r[{elapsed:6.1f}s] map->base x={mb[0]:+.4f} y={mb[1]:+.4f} "
                f"yaw={math.degrees(mb[2]):+7.2f}deg | running p2p: "
                f"xy~{xy_p2p*100:5.2f}cm yaw~{math.degrees(yaw_p2p):5.2f}deg "
                f"(n={n})   ",
                end="",
                flush=True,
            )

    # ------------------------------------------------------------------ #
    # Reporting
    # ------------------------------------------------------------------ #
    def _finish(self) -> None:
        print()  # close the live line
        self._write_csv()
        if self.mode == "moving":
            self._print_summary_moving()
        else:
            self._print_summary_static()

    def _write_csv(self) -> None:
        header = ["t"]
        for link in LINKS:
            header += [f"{link}_x", f"{link}_y", f"{link}_yaw",
                       f"{link}_speed_m_s", f"{link}_vyaw_rad_s"]

        # Pre-compute per-link instantaneous velocity from consecutive samples.
        # First sample in each link gets NaN (no previous point).
        vel: Dict[str, List[Optional[Tuple[float, float]]]] = {}
        for link in LINKS:
            link_rows = self.rows[link]
            v: List[Optional[Tuple[float, float]]] = [None]
            for i in range(1, len(link_rows)):
                t0, x0, y0, yaw0 = link_rows[i - 1]
                t1, x1, y1, yaw1 = link_rows[i]
                dt = t1 - t0
                if dt > 1e-6:
                    speed = math.hypot(x1 - x0, y1 - y0) / dt
                    vyaw = angle_diff(yaw1, yaw0) / dt
                else:
                    speed = vyaw = float("nan")
                v.append((speed, vyaw))
            vel[link] = v

        # Align rows by index (all links sampled together each tick, but a
        # failed lookup can desync counts -- pad short links with NaN).
        n = max((len(self.rows[link]) for link in LINKS), default=0)
        with open(self.out_path, "w", encoding="utf-8") as f:
            f.write(",".join(header) + "\n")
            for i in range(n):
                cells: List[str] = []
                t_written = False
                for link in LINKS:
                    if i < len(self.rows[link]):
                        t, x, y, yaw = self.rows[link][i]
                        if not t_written:
                            cells.append(f"{t:.4f}")
                            t_written = True
                        cells += [f"{x:.6f}", f"{y:.6f}", f"{yaw:.6f}"]
                        v_entry = vel[link][i]
                        if v_entry is not None:
                            cells += [f"{v_entry[0]:.6f}", f"{v_entry[1]:.6f}"]
                        else:
                            cells += ["nan", "nan"]
                    else:
                        if not t_written:
                            cells.append("nan")
                            t_written = True
                        cells += ["nan", "nan", "nan", "nan", "nan"]
                f.write(",".join(cells) + "\n")
        rospy.loginfo("Wrote %d samples to %s", n, self.out_path)

    @staticmethod
    def _link_stats(rows: List[Tuple[float, float, float, float]]) -> Dict[str, Any]:
        if len(rows) < 2:
            return {}
        arr = np.asarray(rows, dtype=float)
        t, x, y, yaw = arr[:, 0], arr[:, 1], arr[:, 2], arr[:, 3]

        ybar = circular_mean(yaw)
        yaw_dev = np.array([angle_diff(v, ybar) for v in yaw])

        # Single-step jumps (consecutive samples).
        d_xy = np.hypot(np.diff(x), np.diff(y))
        d_yaw = np.abs(np.array([angle_diff(yaw[i + 1], yaw[i]) for i in range(len(yaw) - 1)]))

        return {
            "n": len(rows),
            "dur": float(t[-1] - t[0]),
            "x_std": float(x.std()),
            "y_std": float(y.std()),
            "xy_p2p": float(np.hypot(x - x.mean(), y - y.mean()).max() * 2.0),
            "yaw_std": float(yaw_dev.std()),
            "yaw_p2p": float(yaw_dev.max() - yaw_dev.min()),
            "xy_jump_max": float(d_xy.max()),
            "xy_jump_p99": float(np.percentile(d_xy, 99)),
            "yaw_jump_max": float(d_yaw.max()),
            "yaw_jump_p99": float(np.percentile(d_yaw, 99)),
        }

    def _print_summary_static(self) -> None:
        print("\n" + "=" * 72)
        print("LOCALIZATION QUALITY SUMMARY  [STATIC]")
        print("=" * 72)
        labels = {
            "map2base": "map -> base   (what the goal-check uses)",
            "map2odom": "map -> odom   (Cartographer correction)",
            "odom2base": "odom -> base  (ZED VIO odometry)",
        }
        for link in LINKS:
            s = self._link_stats(self.rows[link])
            print(f"\n{labels[link]}")
            if not s:
                print("  (insufficient samples)")
                continue
            print(f"  samples={s['n']}  duration={s['dur']:.1f}s")
            print(
                f"  xy  : std={s['x_std']*100:.2f}/{s['y_std']*100:.2f}cm  "
                f"peak-to-peak={s['xy_p2p']*100:.2f}cm"
            )
            print(
                f"  yaw : std={math.degrees(s['yaw_std']):.2f}deg  "
                f"peak-to-peak={math.degrees(s['yaw_p2p']):.2f}deg"
            )
            print(
                f"  jump: xy max={s['xy_jump_max']*100:.2f}cm (p99 "
                f"{s['xy_jump_p99']*100:.2f})  yaw max="
                f"{math.degrees(s['yaw_jump_max']):.2f}deg (p99 "
                f"{math.degrees(s['yaw_jump_p99']):.2f})"
            )

        mb = self._link_stats(self.rows["map2base"])
        if mb:
            print("\n" + "-" * 72)
            print("INTERPRETATION")
            print(
                f"  Hard tolerance floor (map->base peak-to-peak):\n"
                f"    xy  >= {mb['xy_p2p']*100:.1f} cm\n"
                f"    yaw >= {math.degrees(mb['yaw_p2p']):.1f} deg "
                f"({mb['yaw_p2p']:.3f} rad)"
            )
            print(
                "  Do NOT set xy_goal_tolerance / yaw_goal_tolerance below these.\n"
                "  If the map->odom single-step jump exceeds your target tolerance,\n"
                "  smooth the pose used for goal-checking (see the plan)."
            )
        print("=" * 72)

    @staticmethod
    def _find_settled_tail(
        rows: List[Tuple[float, float, float, float]],
        min_duration_s: float,
        speed_thresh_m_s: float,
    ) -> List[Tuple[float, float, float, float]]:
        """Trailing slice of rows where speed stayed below speed_thresh_m_s."""
        if len(rows) < 2:
            return []
        arr = np.asarray(rows, dtype=float)
        t, x, y = arr[:, 0], arr[:, 1], arr[:, 2]
        dt = np.diff(t)
        speed = np.where(dt > 1e-6, np.hypot(np.diff(x), np.diff(y)) / dt, 0.0)
        # speed[i] = speed between rows[i] and rows[i+1]; scan backwards.
        settled_from = len(speed)
        for i in range(len(speed) - 1, -1, -1):
            if speed[i] < speed_thresh_m_s:
                settled_from = i
            else:
                break
        if settled_from >= len(speed):
            return []
        settled = rows[settled_from:]
        if len(settled) < 2 or settled[-1][0] - settled[0][0] < min_duration_s:
            return []
        return settled

    def _print_summary_moving(self) -> None:
        print("\n" + "=" * 72)
        print("LOCALIZATION QUALITY SUMMARY  [MOVING]")
        print("=" * 72)

        labels = {
            "map2base": "map -> base   (what the goal-check uses)",
            "map2odom": "map -> odom   (Cartographer correction)",
            "odom2base": "odom -> base  (ZED VIO odometry)",
        }
        for link in LINKS:
            s = self._link_stats(self.rows[link])
            print(f"\n{labels[link]}")
            if not s:
                print("  (insufficient samples)")
                continue
            print(f"  samples={s['n']}  duration={s['dur']:.1f}s")
            if link == "map2base":
                arr = np.asarray(self.rows[link], dtype=float)
                path_m = float(np.sum(np.hypot(np.diff(arr[:, 1]), np.diff(arr[:, 2]))))
                print(f"  path length: {path_m:.2f} m  (p2p/std not meaningful for moving run)")
            print(
                f"  jump: xy max={s['xy_jump_max']*100:.2f}cm (p99 "
                f"{s['xy_jump_p99']*100:.2f})  yaw max="
                f"{math.degrees(s['yaw_jump_max']):.2f}deg (p99 "
                f"{math.degrees(s['yaw_jump_p99']):.2f})"
            )

        # Settled-at-goal window.
        print("\n" + "-" * 72)
        settled = self._find_settled_tail(
            self.rows["map2base"],
            min_duration_s=self.settled_window_s,
            speed_thresh_m_s=self.settled_speed_m_s,
        )
        if settled:
            dur = settled[-1][0] - settled[0][0]
            ss = self._link_stats(settled)
            print(f"SETTLED AT GOAL  (last {dur:.1f}s while stationary, n={ss['n']})")
            print(
                f"  xy  : std={ss['x_std']*100:.2f}/{ss['y_std']*100:.2f}cm  "
                f"peak-to-peak={ss['xy_p2p']*100:.2f}cm"
            )
            print(
                f"  yaw : std={math.degrees(ss['yaw_std']):.2f}deg  "
                f"peak-to-peak={math.degrees(ss['yaw_p2p']):.2f}deg"
            )
            print(
                f"\n  Suggested accept_tol:\n"
                f"    xy_goal_tolerance  > {ss['xy_p2p']*100:.1f} cm\n"
                f"    yaw_goal_tolerance > {math.degrees(ss['yaw_p2p']):.1f} deg "
                f"({ss['yaw_p2p']:.3f} rad)"
            )
        else:
            print(
                f"SETTLED AT GOAL  — not detected\n"
                f"  (need >={self.settled_window_s:.0f}s stationary at end; "
                f"speed threshold={self.settled_speed_m_s*100:.1f} cm/s)\n"
                f"  Tip: stop the robot for a few seconds before Ctrl-C."
            )
        print("=" * 72)

    def request_stop(self, *_a: Any) -> None:
        self._stop = True


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--label",
        type=str,
        default="run",
        help="Tag for this run (used in the CSV filename), e.g. fridge_static.",
    )
    parser.add_argument(
        "--mode",
        choices=["static", "moving"],
        default="static",
        help="static: robot held still (reports p2p/std → tight_tol). "
             "moving: robot driving (reports jump stats + settled-at-goal → accept_tol).",
    )
    parser.add_argument(
        "--settled-window",
        type=float,
        default=5.0,
        help="[moving mode] min seconds of stillness at end to report settled-at-goal stats (default: 5).",
    )
    parser.add_argument(
        "--settled-speed",
        type=float,
        default=0.01,
        help="[moving mode] speed threshold (m/s) to consider robot stationary (default: 0.01 = 1 cm/s).",
    )
    parser.add_argument("--map-frame", type=str, default="map")
    parser.add_argument("--odom-frame", type=str, default="odom")
    parser.add_argument("--base-frame", type=str, default="vention_base_link")
    parser.add_argument("--rate", type=float, default=30.0, help="Sample rate (Hz).")
    parser.add_argument("--lookup-timeout-s", type=float, default=0.2)
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Auto-stop after N seconds (0 = run until Ctrl-C).",
    )
    parser.add_argument(
        "--out", type=str, default="", help="Explicit CSV path (overrides --out-dir)."
    )
    parser.add_argument(
        "--out-dir",
        type=str,
        default="localization_logs",
        help="Directory for the CSV when --out is not given.",
    )
    args = parser.parse_args()

    if not ROSPY_IMPORTED:
        raise RuntimeError("ROS not imported. Run this script in a ROS environment.")

    rospy.init_node("measure_localization", anonymous=True, disable_signals=True)
    measurer = LocalizationMeasurer(args)
    # disable_signals=True so our handler runs and we still flush CSV + summary.
    signal.signal(signal.SIGINT, measurer.request_stop)
    measurer.spin()


if __name__ == "__main__":
    main()
