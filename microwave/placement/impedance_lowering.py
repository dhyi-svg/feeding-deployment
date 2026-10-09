"""Final lowering of the held box onto the cavity floor with the arm's task impedance control.

Uses the existing interface unchanged: `ArmInterfaceClient.switch_to_task_compliant_mode()`,
`execute_command(CartesianCommand)` (-> `compliant_set_ee_pose` while compliant) and
`switch_out_of_compliant_mode()`; the controller is `compliant_controller.py` task mode.

Why the gate on J6: that controller runs a 6-DOF Pinocchio model with J6 FIXED at
`model_j6_rad` (-67.6 deg, hack_gen3_robotiq_2f_85.urdf) while the real J6 actuator just holds
its position. Gravity torques, FK and the Jacobian are only right when the real J6 is there
(checked offline: 0.03 mm FK error at -67.6, 53 cm at +67.6). The planner puts the arm on that
J6 for the legs inside the microwave; this module refuses to switch modes otherwise.

Termination (any one ends the descent; the command then holds):
  * contact: the measured tool z lags the commanded one by > contact_lag_m while descending
    slower than contact_stall_mps for contact_stall_s -> hold hold_preload_m below the
    measured pose, settle, done;
  * the command reached press_m below the expected contact without contact -> hold, done
    (reported: the floor was not where perception said);
Aborts (hold where it is, leave compliant mode, report): sideways drift > max_lateral_dev_m,
tracking error > max_track_err_m (before the controller's own 10 cm trip to gravity
compensation), timeout, or any RPC error (e-stop / controller tripped).

Everything is relative to the pose read right after the switch: in compliant mode get_state
returns the MODEL's tool pose, so absolute targets from the high-level frame are never mixed in.
"""
from __future__ import annotations

import time
from collections import deque

import numpy as np

from feeding_deployment.control.robot_controller.command_interface import CartesianCommand


class ImpedanceRefused(RuntimeError):
    """Pre-check failed: compliant mode was NOT entered."""


def j6_check(q, lcfg):
    off = float(np.degrees(abs((q[5] - lcfg.model_j6_rad + np.pi) % (2 * np.pi) - np.pi)))
    return off <= lcfg.j6_tol_deg, off


def lower_with_impedance(arm, expected_drop_m, lcfg, log=print, clock=time.time, sleep=time.sleep):
    """Descend up to expected_drop_m + press_m in task compliant mode. Returns a report dict with
    'ok', 'reason', 'contact', 'drop_m' (measured descent). Raises ImpedanceRefused before any
    mode switch if a pre-check fails."""
    st = arm.get_state()
    ok, off = j6_check(np.asarray(st["position"], float), lcfg)
    if not ok:
        raise ImpedanceRefused(f"J6 is {off:.1f} deg from the compliant model's fixed "
                               f"{np.degrees(lcfg.model_j6_rad):.1f} deg (max {lcfg.j6_tol_deg}) -- not entering "
                               "compliant mode")
    if getattr(arm, "in_compliant_mode", False):
        raise ImpedanceRefused("arm client already in compliant mode")
    if not 0.0 < expected_drop_m < 0.15:
        raise ImpedanceRefused(f"expected drop {expected_drop_m:.3f} m outside (0, 0.15)")
    max_drop = expected_drop_m + lcfg.press_m
    period = 1.0 / lcfg.command_hz
    report = {"ok": False, "reason": "", "contact": False, "drop_m": 0.0, "max_cmd_drop_m": max_drop}

    log(f"  impedance: switching to task compliant mode (J6 {off:.1f} deg from the model value)")
    arm.switch_to_task_compliant_mode()
    try:
        sleep(0.5)
        st = arm.get_state()
        x0 = np.asarray(st["ee_pos"][:3], float)
        quat0 = np.asarray(st["ee_pos"][3:7], float)
        cmd = x0.copy()
        hist = deque()
        t0 = clock()
        t_contact = t_limit = None
        while True:
            t = clock()
            st = arm.get_state()
            x = np.asarray(st["ee_pos"][:3], float)
            hist.append((t, x[2]))
            while hist and t - hist[0][0] > lcfg.contact_stall_s:
                hist.popleft()
            drop = float(x0[2] - x[2])
            lateral = float(np.linalg.norm(x[:2] - x0[:2]))
            lag = float(x[2] - cmd[2])
            report["drop_m"] = drop
            if lateral > lcfg.max_lateral_dev_m:
                report["reason"] = f"ABORT: sideways drift {lateral * 100:.1f} cm"
                break
            if abs(lag) > lcfg.max_track_err_m:
                report["reason"] = f"ABORT: tracking error {lag * 100:.1f} cm"
                break
            if t - t0 > lcfg.timeout_s:
                report["reason"] = f"ABORT: timeout after {lcfg.timeout_s:.0f} s"
                break
            if t_contact is None:
                span = hist[-1][0] - hist[0][0]
                vel = (hist[0][1] - hist[-1][1]) / span if span > 0.6 * lcfg.contact_stall_s else 1.0
                if lag > lcfg.contact_lag_m and vel < lcfg.contact_stall_mps:
                    t_contact = t
                    cmd = x - np.array([0.0, 0.0, lcfg.hold_preload_m])
                    report["contact"] = True
                    log(f"  impedance: contact after {drop * 100:.1f} cm (lag {lag * 100:.1f} cm) -- holding")
                else:
                    cmd_drop = min(max_drop, lcfg.speed_mps * (t - t0))
                    cmd = x0 - np.array([0.0, 0.0, cmd_drop])
                    if cmd_drop >= max_drop and t_limit is None:
                        t_limit = t
            if t_contact is not None and t - t_contact >= lcfg.settle_s:
                report.update(ok=True, reason=f"contact; settled {lcfg.settle_s:.1f} s")
                break
            if t_limit is not None and t - t_limit >= lcfg.settle_s:
                report.update(ok=True, reason=(f"NO CONTACT within {max_drop * 100:.1f} cm (expected "
                                               f"{expected_drop_m * 100:.1f} + press {lcfg.press_m * 100:.1f}) -- "
                                               "the floor is lower than perceived"))
                break
            arm.execute_command(CartesianCommand(cmd.tolist(), quat0.tolist()))
            sleep(period)
        report["final_model_pos"] = x.tolist()
    except Exception as e:  # noqa: BLE001 -- e-stop / controller trip surface as RPC errors
        report["reason"] = f"ABORT: {type(e).__name__}: {e}"
    finally:
        try:
            arm.switch_out_of_compliant_mode()
            log("  impedance: back in position mode")
        except Exception as e:  # noqa: BLE001
            report["ok"] = False
            report["reason"] += (f" | could NOT leave compliant mode ({e}) -- the controller may be in gravity "
                                 "compensation: support the arm, use the e-stop, restart arm_server")
    log(f"  impedance: {report['reason']} (descended {report['drop_m'] * 100:.1f} cm)")
    return report
