#!/usr/bin/env python3
"""
plot_nav_traces.py -- offline 3-way trace comparison from a nav_diag_logger run.

Overlays, on the localization map, the three base trajectories a run recorded:
  * Cartographer (green)  -- tf.csv map->base (mb_*), ABSOLUTE in the map frame
  * ZED VIO    (red)      -- odom.csv, dead-reckoning in the odom frame
  * wheel odom (blue)     -- wheel_odom.csv, dead-reckoning in its own frame
  * sanitized  (orange)   -- sanitized.csv (optional)

The dead-reckoning traces (ZED, wheel) are in their own frames, so each is
SE(2)-anchored to Cartographer's pose at a confident-localization time t0 and run
open-loop from there -- divergence from the green line is that sensor's drift
(same idea as the live scripts/drift_trace_compare.py, done offline from CSVs).

Map background comes FROM THE LOG (never hardcoded):
  1) <navlog>/map.yaml            (the grid this run localized against)
  2) the pbstream named in params_snapshot.json -> converted with
     cartographer_pbstream_to_ros_map (cached beside the log)
  3) --map <yaml>                 (manual override)
  4) none                         (traces only, no background)

Run with a python that has matplotlib+numpy (e.g. /usr/bin/python3). Read-only:
reads CSVs (+ pbstream on fallback) and writes a PNG. Touches no ROS master.

Examples:
  /usr/bin/python3 scripts/plot_nav_traces.py                     # newest run
  /usr/bin/python3 scripts/plot_nav_traces.py <navlog_dir>
  /usr/bin/python3 scripts/plot_nav_traces.py <dir> --map maps/aimee-7-1.yaml
  /usr/bin/python3 scripts/plot_nav_traces.py <dir> --anchor-time 30 --max-jump 1.0
"""

import argparse
import csv
import glob
import json
import math
import os
import subprocess
import sys

CARTO_TO_ROS_MAP = os.path.expanduser(
    "~/cartographer_ws/install_isolated/bin/cartographer_pbstream_to_ros_map")
LOG_BASE = os.path.expanduser(
    "~/deployment_ws/src/feeding-deployment/src/feeding_deployment/integration/log/system_logs")


def _parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("navlog", nargs="?", default=None,
                   help="navlog dir (default: newest under system_logs/)")
    p.add_argument("--map", default=None, help="map_server yaml override for the background")
    p.add_argument("--anchor-time", type=float, default=None,
                   help="anchor time in seconds from run start (default: auto)")
    p.add_argument("--reanchor-per-goal", action="store_true",
                   help="re-anchor ZED/wheel at each goals.csv start (per-leg drift)")
    p.add_argument("--max-jump", type=float, default=1.0,
                   help="drop ZED samples with a >this-metre single-step jump (m)")
    p.add_argument("--start", type=float, default=None, help="window start (s from run start)")
    p.add_argument("--end", type=float, default=None, help="window end (s from run start)")
    p.add_argument("--out", default=None, help="output PNG path (default: <navlog>/traces_<name>.png)")
    return p.parse_args()


# ---- small IO helpers -------------------------------------------------------
def _f(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return float("nan")


def load_csv(path):
    """Return {column_name: list[float]} keyed by header; '' -> nan. None if absent/empty."""
    if not os.path.isfile(path):
        return None
    with open(path) as fh:
        rd = csv.reader(fh)
        try:
            header = next(rd)
        except StopIteration:
            return None
        cols = {h: [] for h in header}
        n = 0
        for row in rd:
            if not row:
                continue
            for i, h in enumerate(header):
                cols[h].append(_f(row[i]) if i < len(row) else float("nan"))
            n += 1
    return cols if n else None


def read_pgm(path):
    """Read a binary (P5) PGM into a (H,W) list-of-bytes; returns (data, W, H) or None."""
    try:
        with open(path, "rb") as f:
            raw = f.read()
    except OSError:
        return None
    # parse header tokens (skip comments), then binary block
    idx, tokens = 0, []
    # magic
    while raw[idx:idx + 1].isspace():
        idx += 1
    magic = raw[idx:idx + 2]
    idx += 2
    if magic != b"P5":
        return None
    while len(tokens) < 3:
        while raw[idx:idx + 1].isspace():
            idx += 1
        if raw[idx:idx + 1] == b"#":
            while raw[idx:idx + 1] not in (b"\n", b""):
                idx += 1
            continue
        start = idx
        while not raw[idx:idx + 1].isspace():
            idx += 1
        tokens.append(int(raw[start:idx]))
    W, H, _maxv = tokens
    idx += 1  # single whitespace after maxval
    return raw[idx:idx + W * H], W, H


def find_map(navlog, override, np):
    """Resolve the map background (see module docstring). Returns (img2d, extent) or None."""
    yaml_path = None
    if override:
        yaml_path = override
    elif os.path.isfile(os.path.join(navlog, "map.yaml")):
        yaml_path = os.path.join(navlog, "map.yaml")
    else:
        # fallback: pbstream named in the snapshot -> convert once, cache beside log
        snap = os.path.join(navlog, "params_snapshot.json")
        pb = ""
        if os.path.isfile(snap):
            try:
                with open(snap) as f:
                    pb = (json.load(f).get("_meta", {})
                          .get("cartographer_load_state_filename", "") or "")
            except Exception:
                pb = ""
        if pb and os.path.isfile(pb) and os.path.isfile(CARTO_TO_ROS_MAP):
            stem = os.path.join(navlog, "map_from_pbstream")
            try:
                subprocess.check_call([CARTO_TO_ROS_MAP,
                                       "-pbstream_filename", pb,
                                       "-map_filestem", stem, "-resolution", "0.05"],
                                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                yaml_path = stem + ".yaml"
            except Exception:
                yaml_path = None
    if not yaml_path or not os.path.isfile(yaml_path):
        return None
    meta = {}
    with open(yaml_path) as f:
        for line in f:
            if ":" in line:
                k, v = line.split(":", 1)
                meta[k.strip()] = v.strip()
    res = float(meta.get("resolution", "0.05"))
    origin = json.loads(meta.get("origin", "[0,0,0]").replace("(", "[").replace(")", "]"))
    img = meta.get("image", "map.pgm")
    if not os.path.isabs(img):
        img = os.path.join(os.path.dirname(yaml_path), img)
    pgm = read_pgm(img)
    if pgm is None:
        return None
    data, W, H = pgm
    arr = np.frombuffer(data, dtype=np.uint8)[:W * H].reshape(H, W)
    extent = [origin[0], origin[0] + W * res, origin[1], origin[1] + H * res]
    return arr, extent


# ---- SE(2) ------------------------------------------------------------------
def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def interp_pose(np, tq, ts, xs, ys, yaws):
    """Interpolate (x,y,yaw) at time tq; yaw via unwrapped interp."""
    x = float(np.interp(tq, ts, xs))
    y = float(np.interp(tq, ts, ys))
    yw = float(np.interp(tq, ts, np.unwrap(yaws)))
    return x, y, wrap(yw)


def anchor_to_carto(np, P, C, t0):
    """Map dead-reckoning trace P (dict t,x,y,yaw arrays) into the map frame,
    seeded at Cartographer pose C at t0, running open-loop. Returns (mx, my)."""
    cx0, cy0, cyaw0 = interp_pose(np, t0, C["t"], C["x"], C["y"], C["yaw"])
    px0, py0, pyaw0 = interp_pose(np, t0, P["t"], P["x"], P["y"], P["yaw"])
    cc, sc = math.cos(cyaw0), math.sin(cyaw0)
    cp, sp = math.cos(pyaw0), math.sin(pyaw0)
    mx, my = [], []
    for x, y in zip(P["x"], P["y"]):
        # P(t) relative to P(t0), expressed in P(t0) frame
        rx = cp * (x - px0) + sp * (y - py0)
        ry = -sp * (x - px0) + cp * (y - py0)
        # compose onto Cartographer's anchor pose
        mx.append(cx0 + cc * rx - sc * ry)
        my.append(cy0 + sc * rx + cc * ry)
    return mx, my


def pick_anchor(np, C, mo, start_t, warmup=5.0):
    """Auto-pick a confident-localization anchor time: earliest time (after a
    warmup) where map->odom is nearly stationary; fall back to first carto time."""
    if mo is not None and len(mo["t"]) > 3:
        t = np.array(mo["t"])
        x, y, yw = np.array(mo["x"]), np.array(mo["y"]), np.array(mo["yaw"])
        good = ~(np.isnan(x) | np.isnan(y))
        t, x, y = t[good], x[good], y[good]
        if len(t) > 3:
            dt = np.diff(t)
            spd = np.hypot(np.diff(x), np.diff(y)) / np.clip(dt, 1e-3, None)
            for i in range(len(spd)):
                if t[i] - start_t >= warmup and spd[i] < 0.02:
                    return float(t[i])
    return float(C["t"][0])


def main():
    args = _parse_args()
    try:
        import numpy as np
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as e:  # noqa: BLE001
        sys.stderr.write("need numpy+matplotlib (try /usr/bin/python3): %s\n" % e)
        return 2

    navlog = args.navlog
    if navlog is None:
        dirs = sorted(glob.glob(os.path.join(LOG_BASE, "navlog_*")), key=os.path.getmtime)
        if not dirs:
            sys.stderr.write("no navlog_* dirs under %s\n" % LOG_BASE)
            return 3
        navlog = dirs[-1]
    navlog = os.path.abspath(navlog)
    name = os.path.basename(navlog)

    # ---- load traces (by header name; tolerant of column changes) ----
    odom = load_csv(os.path.join(navlog, "odom.csv"))
    tf = load_csv(os.path.join(navlog, "tf.csv"))
    wheel = load_csv(os.path.join(navlog, "wheel_odom.csv"))
    san = load_csv(os.path.join(navlog, "sanitized.csv"))
    if tf is None or "mb_x" not in tf:
        sys.stderr.write("no tf.csv / mb_* (Cartographer) in %s -- cannot anchor\n" % navlog)
        return 3

    def trace(cols, tk, xk, yk, ykaw, jump=None):
        if cols is None or xk not in cols:
            return None
        t = np.array(cols[tk]); x = np.array(cols[xk]); y = np.array(cols[yk])
        yw = np.array(cols[ykaw]) if ykaw in cols else np.zeros_like(x)
        m = ~(np.isnan(t) | np.isnan(x) | np.isnan(y))
        t, x, y, yw = t[m], x[m], y[m], yw[m]
        if jump is not None and len(x) > 1:
            step = np.hypot(np.diff(x), np.diff(y))
            keep = np.concatenate([[True], step <= jump])
            t, x, y, yw = t[keep], x[keep], y[keep], yw[keep]
        order = np.argsort(t)
        return {"t": t[order], "x": x[order], "y": y[order], "yaw": yw[order]}

    C = trace(tf, "t_wall", "mb_x", "mb_y", "mb_yaw")
    Z = trace(odom, "t_stamp", "x", "y", "yaw", jump=args.max_jump)
    W = trace(wheel, "t_stamp", "x", "y", "yaw")
    S = trace(san, "t_stamp", "x", "y", "yaw")
    MO = trace(tf, "t_wall", "mo_x", "mo_y", "mo_yaw")
    if C is None or not len(C["t"]):
        sys.stderr.write("Cartographer trace empty\n")
        return 3

    start_t = C["t"][0]
    t0 = args.anchor_time + start_t if args.anchor_time is not None \
        else pick_anchor(np, C, MO, start_t)

    # ---- align dead-reckoning traces into the map frame ----
    aligned = {}
    if Z is not None and len(Z["t"]) > 1:
        aligned["ZED"] = (anchor_to_carto(np, Z, C, t0), "red", Z)
    if S is not None and len(S["t"]) > 1:
        aligned["sanitized"] = (anchor_to_carto(np, S, C, t0), "orange", S)
    if W is not None and len(W["t"]) > 1:
        aligned["wheel"] = (anchor_to_carto(np, W, C, t0), "blue", W)

    themap = find_map(navlog, args.map, np)

    # ---- figure ----
    have_tilt = odom is not None and "roll" in odom and \
        not np.all(np.isnan(np.array(odom["roll"])))
    nrows = 3 if have_tilt else 2
    fig = plt.figure(figsize=(11, 4.2 * nrows))
    ax = fig.add_subplot(nrows, 1, 1)
    if themap is not None:
        img, extent = themap
        ax.imshow(img, cmap="gray", extent=extent, origin="upper", zorder=0, alpha=0.85)
    ax.plot(C["x"], C["y"], "-", color="green", lw=1.6, label="cartographer (map->base)", zorder=3)
    for label, ((mx, my), color, _P) in aligned.items():
        ax.plot(mx, my, "-", color=color, lw=1.0, alpha=0.9, label=label, zorder=2)
    ax0 = interp_pose(np, t0, C["t"], C["x"], C["y"], C["yaw"])
    ax.plot([ax0[0]], [ax0[1]], "k*", ms=13, label="anchor", zorder=5)
    ax.set_aspect("equal", "datalim")
    ax.set_xlabel("map x (m)"); ax.set_ylabel("map y (m)")
    ax.legend(loc="best", fontsize=8)
    ax.set_title("%s -- traces%s (anchor t0=%.1fs)"
                 % (name, "" if themap is not None else " (no map background)", t0 - start_t))

    # drift vs time
    axd = fig.add_subplot(nrows, 1, 2)
    for label, ((mx, my), color, P) in aligned.items():
        cx = np.interp(P["t"], C["t"], C["x"])
        cy = np.interp(P["t"], C["t"], C["y"])
        d = np.hypot(np.array(mx) - cx, np.array(my) - cy)
        axd.plot(P["t"] - start_t, d, "-", color=color, lw=1.0, label=label)
    axd.set_xlabel("time (s)"); axd.set_ylabel("dist from cartographer (m)")
    axd.set_title("drift vs cartographer"); axd.legend(loc="best", fontsize=8); axd.grid(alpha=0.3)

    if have_tilt:
        axt = fig.add_subplot(nrows, 1, 3)
        ot = np.array(odom["t_stamp"]) - start_t
        axt.plot(ot, np.degrees(np.array(odom["roll"])), "-", color="purple", lw=0.9, label="roll")
        axt.plot(ot, np.degrees(np.array(odom["pitch"])), "-", color="teal", lw=0.9, label="pitch")
        axt.axhline(0, color="k", lw=0.4)
        axt.set_xlabel("time (s)"); axt.set_ylabel("odom tilt (deg)")
        axt.set_title("ZED odom roll/pitch (tilt)"); axt.legend(loc="best", fontsize=8); axt.grid(alpha=0.3)

    missing = [n for n in ("wheel",) if n not in aligned]
    if missing:
        fig.text(0.01, 0.005, "note: %s not in this log" % ", ".join(missing), fontsize=8)

    fig.tight_layout()
    out = args.out or os.path.join(navlog, "traces_%s.png" % name)
    fig.savefig(out, dpi=130)
    print("[plot_nav_traces] wrote %s" % out)
    print("[plot_nav_traces] traces: %s | map bg: %s | anchor t0=%.1fs from start"
          % (", ".join(["cartographer"] + list(aligned)),
             "yes" if themap is not None else "no", t0 - start_t))
    return 0


if __name__ == "__main__":
    sys.exit(main())
