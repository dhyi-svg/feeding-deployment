"""Lowering the held container onto the microwave floor in joint impedance, until contact, on SheppyArm.
Adapted from rammp-kitchen-capability-engines place_container (lower.py + plan.py, feature/7-place-container).

1. settle: an impedance hold on the line's start; the hand must then be still. The arm sags when it goes
   compliant (4-6 cm on this arm, 10-26), and that must be over before the descent or it reads as motion.
   The hold watches the height: a sag that reaches the expected contact stops it.
2. descent: the planned straight line down, timed minimum-jerk by move_joints. Contact = z velocity back up
   after moving down (VzContact), counted only while the commanded speed is up (contact_gate). There is no
   wrench on this driver. A hand held `stall_m` above where the descent has it also counts: a light load, or
   one already touching after the sag, never shows that velocity. The line is cut so the HAND, not the sagged
   setpoint, ends at contact_z - overtravel_m, and a monitor stops it there whatever the setpoint does. The
   monitor also stops on sideways deviation and on tilt.
3. hold a little below the contact point, gripper still closed.
Any ending before verified contact leaves impedance on the current joints, so the arm is not left compliant.
"""
from __future__ import annotations

import csv
import time

import numpy as np
from scipy.spatial.transform import Rotation as R

import sheppy_arm
import xyz_move as xm
from xyz_move import Refused

LOOSE_TOL_RAD = 0.5       # impedance path tolerance: the compliant arm lags and is held back; the monitor watches
Z_STILL_SPEED = 0.002     # m/s, |vz| under this = still
SETTLE_STILL_S = 0.5      # the hand must be still this long ...
SETTLE_WAIT_S = 5.0       # ... within this long after the settle
EXIT_SETTLE_S = 2.0
EXIT_HOLD_S = 0.5
CONTACT_GATE = 0.75       # contact counts only while the commanded speed is at least this x lower_speed
MAX_PEAK = 1.5            # the descent's peak speed is at most this x lower_speed


class VzContact:
    """Once the hand has moved down faster than `moving`, vz >= `stop` for `hits` new messages in a row."""

    def __init__(self, stop=-0.01, moving=0.01, hits=2):
        self.stop, self.moving, self.need = stop, moving, hits
        self.moved, self.count = False, 0

    def update(self, vz):
        self.moved = self.moved or vz < -self.moving
        self.count = self.count + 1 if self.moved and vz >= self.stop else 0
        return self.count >= self.need


def minjerk_distance(u):
    u = min(max(u, 0.0), 1.0)
    return 10 * u**3 - 15 * u**4 + 6 * u**5


def minjerk_speed(u):
    u = min(max(u, 0.0), 1.0)
    return 30 * u**2 * (1 - u) ** 2


def minjerk_time(d):
    lo, hi = 0.0, 1.0
    for _ in range(50):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if minjerk_distance(mid) < d else (lo, mid)
    return (lo + hi) / 2


def descent_speed_scale(meet, lower_speed):
    """move_joints speed_scale so the hand passes `meet` (0..1 of the way, the expected contact) at lower_speed."""
    u = minjerk_time(min(max(meet, 0.05), 0.95))
    peak = min(lower_speed / (minjerk_speed(u) / minjerk_speed(0.5)), MAX_PEAK * lower_speed)
    return peak / sheppy_arm.CART_SPEED


def contact_gate(u, length, duration, lower_speed):
    """At the ends of a minimum-jerk move a hand slowing with its command must not read as stopped."""
    return 0.0 < u < 1.0 and minjerk_speed(u) * length / duration >= CONTACT_GATE * lower_speed


def hold_point(line_q, line_z, z_cmd):
    """Joints on the line down (falling heights line_z) at height z_cmd, interpolated; None above its start."""
    if z_cmd >= line_z[0]:
        return None
    for i in range(len(line_z) - 1):
        if line_z[i] >= z_cmd >= line_z[i + 1]:
            s = (line_z[i] - z_cmd) / max(line_z[i] - line_z[i + 1], 1e-9)
            return np.asarray(line_q[i]) + s * (np.asarray(line_q[i + 1]) - np.asarray(line_q[i]))
    return np.asarray(line_q[-1])


class Recorder:
    """/ee_state rows per phase, for tuning vz_stop and reading the sag afterwards."""

    def __init__(self):
        self.rows, self.phase = [], "settle"

    def poll(self, arm):
        s = arm.get_state()
        ee, v = np.asarray(s["ee_pos"], float), np.asarray(s["ee_vel"], float)
        self.rows.append([time.time(), self.phase, *ee[:3], *v])
        return ee, v

    def write(self, directory):
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / "descent.csv"
        with open(path, "w", newline="") as f:
            out = csv.writer(f)
            out.writerow(["t", "phase", "x", "y", "z", "vx", "vy", "vz"])
            out.writerows(self.rows)
        return path


def wait_still(arm, timeout_s, hold_s=0.2, rec=None):
    t0, since = time.monotonic(), None
    while time.monotonic() - t0 < timeout_s:
        vz = rec.poll(arm)[1][2] if rec is not None else arm.get_state()["ee_vel"][2]
        still = abs(float(vz)) < Z_STILL_SPEED
        since = (since or time.monotonic()) if still else None
        if since is not None and time.monotonic() - since >= hold_s:
            return True
        time.sleep(0.02)
    return False


def exit_impedance(arm, log):
    """Back to position control with the setpoint ON the current joints, so nothing jumps."""
    wait_still(arm, EXIT_SETTLE_S)
    ok = arm.hold_joints(arm.raw_joints(), EXIT_HOLD_S)
    log("impedance off: position setpoint = the current joints" + ("" if ok else " (FAILED)"))


def gains(cfg):
    g = xm.impedance_gains(cfg.get("kq"), cfg.get("zeta"), cfg.get("torque_limit"))
    return {**g, "path_tol": LOOSE_TOL_RAD}


def lower(arm, sim, line, lengths, contact_z, rot_goal, cfg, log, log_dir):
    """Down `line` (joints, line[0] = the start) until contact; contact_z = tool height where the container should
    touch. Returns the hold's joints, gripper closed. Raises Refused; before contact the arm is left in position."""
    imp, rec = gains(cfg), Recorder()
    line_z = [float(xm.fk(sim, q)[0][2]) for q in line]
    start = np.asarray(xm.fk(sim, line[0])[0], float)

    def fail(why):
        exit_impedance(arm, log)
        return Refused(f"STOPPED: {why} -- container still held")

    try:
        # 1. settle
        log(f"impedance on (kq {imp['kq'][0]:g}..), holding the start {cfg['settle_s']:.1f} s ...")
        low = {}

        def settle_monitor():
            ee, _ = rec.poll(arm)
            low["z"] = float(ee[2])
            return ee[2] - contact_z < cfg["min_settle_gap_m"]

        ok = arm.hold_joints(line[0], cfg["settle_s"], imp, settle_monitor)
        if arm.monitor_stopped:
            raise fail(f"sagged to tool z {low['z']:.3f}, within {cfg['min_settle_gap_m'] * 1000:.0f} mm of the expected "
                       f"contact {contact_z:.3f}, before lowering")
        if not ok:
            raise fail("the impedance hold at the start failed")
        if not wait_still(arm, SETTLE_WAIT_S, SETTLE_STILL_S, rec):
            raise fail("the hand is not still in impedance -- not lowering")
        ee = np.asarray(arm.get_state()["ee_pos"], float)
        lag = float(ee[2]) - line_z[0]
        log(f"settled, hand {lag * 100:+.1f} cm {'above' if lag >= 0 else 'below'} the planned start")
        if -lag > cfg["max_settle_sag_m"]:
            raise fail(f"impedance sagged {-lag * 100:.1f} cm (max {cfg['max_settle_sag_m'] * 100:.1f})")
        tilt = xm.tilt_deg(R.from_quat(ee[3:7]), rot_goal)
        if tilt > cfg["max_tilt_deg"]:
            raise fail(f"container tilted {tilt:.1f} deg in impedance (max {cfg['max_tilt_deg']})")

        # 2. the line down, stopped on contact
        z_min = contact_z - cfg["overtravel_m"]         # the lowest the hand may go, measured
        z_cmd = z_min - lag                             # the setpoint is lag away from the hand
        keep = [q for q, z in zip(line, line_z) if z > z_cmd]
        cut = hold_point(line, line_z, z_cmd)
        if cut is None or len(keep) < 1:
            raise fail(f"no room to lower: the hand is {z_min - ee[2]:+.3f} m from its lowest allowed height")
        line = np.array(keep + [cut])
        line_z = [float(xm.fk(sim, q)[0][2]) for q in line]
        lengths = list(np.abs(np.diff(line_z)))
        z0, length = float(ee[2]), float(np.sum(lengths))
        meet = (z0 - contact_z) / max(z0 - line_z[-1], 1e-6)
        scale = descent_speed_scale(meet, cfg["lower_speed"])
        contact, state = VzContact(stop=cfg["vz_stop"]), dict(t0=None, z=None, why=None)
        settled_xy = ee[:2].copy()

        def monitor():
            ee, v = rec.poll(arm)
            state["t0"] = state["t0"] or time.monotonic()
            u = (time.monotonic() - state["t0"]) / max(arm.last_duration, 1e-3)
            dev, tilt = float(np.linalg.norm(ee[:2] - settled_xy)), xm.tilt_deg(R.from_quat(ee[3:7]), rot_goal)
            if dev > cfg["max_xy_dev_m"] or tilt > cfg["max_tilt_deg"]:
                state["why"] = f"{dev * 100:.1f} cm sideways, {tilt:.1f} deg tilt"
                return True
            if ee[2] < z_min - 0.003:
                state["why"] = f"tool z {ee[2]:.3f} under the lowest allowed {z_min:.3f}"
                return True
            behind = float(ee[2]) - (z0 - minjerk_distance(u) * length)
            if behind > cfg["stall_m"] or (contact.update(float(v[2])) and contact_gate(u, length, arm.last_duration, cfg["lower_speed"])):
                state["z"] = float(ee[2])
                rec.phase = "contact"
                log(f"contact at tool z {state['z']:.3f}: vz {v[2] * 100:+.2f} cm/s, {behind * 100:.1f} cm behind the "
                    f"command, {min(u, 1.0) * 100:.0f}% of the move")
                return True
            return False

        rec.phase = "descend"
        log(f"lowering: {length * 100:.1f} cm (impedance), ~{cfg['lower_speed'] * 100:.0f} cm/s at the expected "
            f"contact ({meet * 100:.0f}% down); contact = vz back to {cfg['vz_stop'] * 100:+.1f} cm/s")
        ok = arm.move_joints(list(line[1:]), cart_lengths=lengths, speed_scale=scale, impedance=imp, monitor=monitor)
        if state["why"]:
            raise fail(f"lowering went off line: {state['why']}")
        if state["z"] is None:
            raise fail(f"went down {length * 100:.1f} cm without contact" if ok else "the descent failed")
        early = state["z"] - contact_z
        if early > cfg["early_stop_m"]:
            raise fail(f"stopped {early * 100:.1f} cm above the expected contact -- it hit something else")

        # 3. hold just below the contact point
        q = hold_point(line, line_z, state["z"] - lag - cfg["hold_press_m"])
        hold_q = np.asarray(arm.raw_joints() if q is None else q, float)
        arm.hold_joints(hold_q, 0.5, imp)
        wait_still(arm, 3.0, rec=rec)
        rec.phase = "done"
        return hold_q
    finally:
        log(f"recorded {len(rec.rows)} states -> {rec.write(log_dir)}")
