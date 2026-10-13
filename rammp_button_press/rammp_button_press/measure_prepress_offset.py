"""Measure the pre-press spot once and print the constants for press_button.py. NO MOTION.

press_button.py stores where the Kinova tool frame goes, relative to the detected button,
when the LEFT fingertip is STANDOFF_M (2 cm) in front of it. That depends on the fingers and on
the camera mount / hand-eye calibration, so re-run this on every rig and whenever they change.
RAMMP port of scripts/button_press/measure_prepress_offset.py (button-task).

  1. sheppy profile `arm-camera` up, the camera ~25 cm in front of the panel (press_button's
     staging range), all 5 domes in view. Measure from THIS range: the 3D chain (depth +
     hand-eye) read the same button ~1.5 cm lower from 39 cm than from 20 cm (2026-10-03).
  2. Run this. It detects the panel frame from that view (arm base frame, so it stays valid
     while the arm moves).
  3. When prompted: switch sheppy to `camera-only` (stops the arm driver -- never hand-guide or
     teleop with kinova_gen3_node up: WRONG_SERVOING_MODE crash, 2026-10-07), hand-guide the
     arm until the left fingertip is 2 cm straight out from the button centre (a 2 cm spacer
     helps), wrist roughly square to the panel, let go, switch back to `arm-camera` (the
     driver starts holding wherever the arm is), then press Enter. Do NOT move the microwave.
     With --touch, put the fingertip ON the button centre instead (lightly, without pressing
     it in); the script backs the recorded tool pose off STANDOFF_M along the panel normal,
     and the touch also shows this rig's hand-eye error directly (fingertip vs fitted plane).
  4. Paste the printed PREPRESS_EE_OFFSET_M / PREPRESS_EE_QUAT_PANEL into press_button.py.

    ros2 run rammp_button_press measure_prepress_offset --target timer_clock
"""
import argparse
import sys
import time

import numpy as np
import rclpy
from scipy.spatial.transform import Rotation

from rammp_button_press import Abort
from rammp_button_press.panel_frame import describe, from_panel, rot_to_panel, to_panel
from rammp_button_press.press_button import (FINGERTIP_PAST_TOOL_M, MIN_PREPRESS_FINGERTIP_CLEAR_M, STANDOFF_M, Run,
                                             build_parser, fingertip_out_of_panel, measure_panel_frame_domes)

DRIVER_BACK_TIMEOUT_S = 120.0


def wait_for_driver(run: Run) -> None:
    """Block until /joint_states and /ee_state are fresh again (sheppy back on `arm-camera`)."""
    t0 = time.time()
    while time.time() - t0 < DRIVER_BACK_TIMEOUT_S:
        try:
            run.arm.state()
            time.sleep(1.0)       # let a few ~100 Hz readings in after the restart
            run.arm.state()
            return
        except Abort:
            time.sleep(0.5)
    raise Abort(f"arm driver not back within {DRIVER_BACK_TIMEOUT_S:.0f} s -- is the `arm-camera` profile up?")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--target", required=True)
    ap.add_argument("--touch", action="store_true",
                    help=f"fingertip touching the button (not {STANDOFF_M*100:.0f} cm in front); backed off here")
    a, rest = ap.parse_known_args()
    run_args = build_parser().parse_args(["--target", a.target, *rest])  # never --execute
    if run_args.execute:
        print("measure_prepress_offset never moves the arm; drop --execute")
        return 2

    rclpy.init()
    run = None
    try:
        try:
            run = Run(run_args)
            run.preflight(require_ready=False)
            print("\n== 1/2: panel frame from this view ==")
            origin, R = measure_panel_frame_domes(run, a.target)
            where = "ON the centre of" if a.touch else f"{STANDOFF_M*100:.0f} cm in front of"
            input(f"\n== 2/2: sheppy -> camera-only, hand-guide the LEFT fingertip {where} "
                  f"{a.target}, let go, sheppy -> arm-camera, then press Enter ==")
            wait_for_driver(run)
            problems, _ = run.arm.status()
            tool = [pr for pr in problems if pr.startswith("tool frame")]
            if tool:
                raise Abort(tool[0])
        except Abort as e:
            print(f"\nABORT: {e}")
            return 2
        p, Rm = run.arm.ee_pose()
        local = to_panel(p, origin, R)
        if a.touch:
            tip = to_panel(p + Rm[:, 2] * FINGERTIP_PAST_TOOL_M, origin, R)
            print(f"\n  touch: fingertip {describe(tip)} from the detected button "
                  f"(the detection's error at this button, if the fingertip is really on its centre)")
            local = local + np.array([0.0, 0.0, STANDOFF_M])     # back off along the panel normal
            p = from_panel(local, origin, R)
        q_local = Rotation.from_matrix(rot_to_panel(Rm, R)).as_quat()
        tool_z = rot_to_panel(Rm, R)[:, 2]
        print(f"\n  tool frame: {describe(local)}")
        print(f"  fingertip {fingertip_out_of_panel(p, Rm, origin, R)*100:+.1f} cm out of the fitted panel plane "
              f"(aim: {STANDOFF_M*100:.0f}; press_button refuses below {MIN_PREPRESS_FINGERTIP_CLEAR_M*100:.0f})")
        print(f"  tool z axis in the panel frame {np.round(tool_z, 3)} "
              f"({np.degrees(np.arccos(np.clip(-tool_z[2], -1, 1))):.0f} deg off straight-in)")
        print("\nPaste into rammp_button_press/press_button.py:\n")
        print(f"PREPRESS_EE_OFFSET_M = np.array([{local[0]:.4f}, {local[1]:.4f}, {local[2]:.4f}])")
        print(f"PREPRESS_EE_QUAT_PANEL = np.array([{q_local[0]:.5f}, {q_local[1]:.5f}, "
              f"{q_local[2]:.5f}, {q_local[3]:.5f}])   # xyzw")
        run.log({"stage": "measure_prepress_offset", "target": a.target, "ee_local": local.tolist(), "touch": a.touch,
                 "quat_panel": q_local.tolist(), "panel_origin": origin.tolist(), "panel_R": R.tolist()})
        return 0
    finally:
        if run is not None:
            run.arm.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
