"""Measure the pre-press spot once and print the constants for press_button.py. NO MOTION.

press_button.py stores where the Kinova tool frame goes, relative to the detected button,
when the LEFT fingertip is STANDOFF_M (2 cm) in front of it. That depends on the fingers, so
re-run this whenever they change:

  1. Bring the stack up (scripts/button_press/bringup.sh) with the camera ~25 cm in front of
     the panel (press_button's staging range), all 5 domes in view (watch view_detection.py).
     Measure from THIS range, not further out: the 3D chain (depth + hand-eye) reads the same
     button ~1.5 cm lower from 39 cm than from 20 cm (2026-10-03), and press_button takes its
     final measurement at ~25 cm with the dome detector, as this script does -- teach and replay must see the panel from the same range.
  2. Run this. It detects the panel frame from that view.
  3. When prompted, hand-guide the arm until the left fingertip is 2 cm straight out from the
     button centre (a 2 cm spacer helps), wrist roughly square to the panel. Let go, then
     press Enter. Do NOT move the microwave in between.
  4. Paste the printed PREPRESS_EE_OFFSET_M / PREPRESS_EE_QUAT_PANEL into
     src/feeding_deployment/button_press/press_button.py.

Usage (env as in scripts/button_press/bringup.sh):
    python3 -u scripts/button_press/measure_prepress_offset.py --target timer_clock
"""
import argparse
import sys

import numpy as np
import rclpy
from scipy.spatial.transform import Rotation

from feeding_deployment.button_press import Abort
from feeding_deployment.button_press.panel_frame import describe, rot_to_panel, to_panel
from feeding_deployment.button_press.press_button import STANDOFF_M, Run, build_parser, measure_panel_frame_domes


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--target", required=True)
    a = ap.parse_args()
    run_args = build_parser().parse_args(["--target", a.target])  # no --execute

    rclpy.init()
    try:
        run = Run(run_args)
        try:
            run.preflight(require_ready=False)
            print("\n== 1/2: panel frame from this view ==")
            origin, R = measure_panel_frame_domes(run, a.target)
        except Abort as e:
            print(f"\nABORT: {e}")
            return 2
        input(f"\n== 2/2: hand-guide the LEFT fingertip {STANDOFF_M*100:.0f} cm in front of {a.target}, "
              "let go, then press Enter ==")
        p, Rm = run.arm.ee_pose()
        local = to_panel(p, origin, R)
        q_local = Rotation.from_matrix(rot_to_panel(Rm, R)).as_quat()
        tool_z = rot_to_panel(Rm, R)[:, 2]
        print(f"\n  tool frame: {describe(local)}")
        print(f"  tool z axis in the panel frame {np.round(tool_z, 3)} "
              f"({np.degrees(np.arccos(np.clip(-tool_z[2], -1, 1))):.0f} deg off straight-in)")
        print("\nPaste into src/feeding_deployment/button_press/press_button.py:\n")
        print(f"PREPRESS_EE_OFFSET_M = np.array([{local[0]:.4f}, {local[1]:.4f}, {local[2]:.4f}])")
        print(f"PREPRESS_EE_QUAT_PANEL = np.array([{q_local[0]:.5f}, {q_local[1]:.5f}, "
              f"{q_local[2]:.5f}, {q_local[3]:.5f}])   # xyzw")
        run.log({"stage": "measure_prepress_offset", "target": a.target, "ee_local": local.tolist(),
                 "quat_panel": q_local.tolist(), "panel_origin": origin.tolist(), "panel_R": R.tolist()})
        return 0
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
