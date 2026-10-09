"""Put the held OXO box into the OPEN microwave on `rchi-cpu-5` -- hardware entry point.

Dry run is the default everywhere; `--execute` (plus typing `go`, unless --yes) is the only way
to move the arm. Run from the repo root (the sim config path is relative), with the bring-up up
(`microwave/placement/bringup.sh`):

    P=microwave/placement/real_gen3_ros2_place_container_microwave.py
    python3 -u $P perceive                                   # detector + cavity only (no motion)
    python3 -u $P plan  --container-drop 0.09                # + every leg in the sim (no motion)
    python3 -u $P place --container-drop 0.09 --execute --stop-after pre-insert
    python3 -u $P place --container-drop 0.09 --execute --stop-after insert
    python3 -u $P place --container-drop 0.09 --execute      # ... through the lowering; hand stays on the box
    python3 -u $P release                                    # plan the release + retract (no motion)
    python3 -u $P release --execute                          # open, lift 5 mm, back out, park if possible
    python3 -u $P all   --container-drop 0.09 --execute      # place, then (after a second 'go') release

    python3 -u $P plan --replay ~/microwave_place_logs/<run>  # offline: a saved run's frames + arm state

Options: --lowering impedance|position (default from placement_config.yaml: impedance),
--config FILE, --device cpu (YOLO on CPU if the GPU is busy), --yes.
Every run writes frames, overlays, cavity.json and plan.json to ~/microwave_place_logs/<time>/.
Library use: `placement_workflow` (perceive / plan_placement / execute_placement / plan_release /
execute_release). The same steps as a ROS 2 node: `microwave_place_node.py`.
"""
import argparse
import json
import os
import sys
import time
import types
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))              # placement modules
sys.path.insert(0, str(HERE.parent))       # microwave/: microwave_common

import placement_workflow as wf  # noqa: E402
from placement_config import load_config  # noqa: E402


def _confirm(what, yes):
    if yes:
        return
    if input(f"\n{what}\nType 'go' to move the arm (anything else aborts): ").strip() != "go":
        raise wf.PlacementRefused("not confirmed -- nothing commanded")


def _state_stub(state):
    """Read-only arm state for --replay (planning only: there is no command method)."""
    st = {k: np.asarray(v) if isinstance(v, list) else v for k, v in state.items()}
    return types.SimpleNamespace(get_state=lambda: st)


def _build(args, cfg):
    """(arm, frame source, detector) for this run."""
    from microwave_detector import YoloMicrowaveDetector
    if args.replay:
        run = Path(args.replay).expanduser()
        state_file = run / "arm_state.json"
        if not state_file.exists():
            raise wf.PlacementRefused(f"{state_file} missing -- replay needs a run saved by this script")
        frames = sorted(run.glob("frame_*.npz"))
        return _state_stub(json.loads(state_file.read_text())), wf.ReplayFrameSource(frames), YoloMicrowaveDetector(cfg.detector)
    from feeding_deployment.control.robot_controller.arm_client import ArmInterfaceClient
    from feeding_deployment.perception.tf_interface import TFInterface
    from feeding_deployment.ros2.realsense_ros2_interface import RealSenseROS2Interface
    arm = ArmInterfaceClient()
    rs = RealSenseROS2Interface()
    if not rs.wait_for_frames(30.0):
        raise wf.PlacementRefused("no RGB-D frames -- is realsense2_camera up with align_depth.enable:=true?")
    source = wf.RosFrameSource(rs, TFInterface(), cfg.stabilizer.max_frame_age_s)
    print(f"loading {cfg.detector.model} ...")
    return arm, source, YoloMicrowaveDetector(cfg.detector)


def _perceive(arm, source, det, cfg, log_dir=None):
    st = arm.get_state()
    ee = np.asarray(st["ee_pos"], float)
    run_dir = Path(log_dir) if log_dir else wf.LOG_ROOT / time.strftime("%Y%m%d_%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / "arm_state.json").write_text(json.dumps({k: np.asarray(v).tolist() if not np.isscalar(v) else v
                                                        for k, v in st.items()}, indent=1))
    try:
        state_name = arm._arm_interface.get_arm_state()["name"]
        print(f"arm: {state_name}, gripper {float(st['gripper_pos']):.2f}, J {np.round(np.degrees(st['position']), 1).tolist()}")
    except AttributeError:
        pass
    return wf.perceive(source, det, cfg, tool_pose=(ee[:3], ee[3:7]), log_dir=run_dir)


def main():
    a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument("command", choices=["perceive", "plan", "place", "release", "all"])
    a.add_argument("--execute", action="store_true", help="command the arm (default: dry run)")
    a.add_argument("--stop-after", choices=wf.STAGES, default="lower", help="place: stop after this stage")
    a.add_argument("--container-drop", type=float, help="tool frame -> container bottom (m), measured")
    a.add_argument("--lowering", choices=["impedance", "position"])
    a.add_argument("--config", help="YAML overrides (default: placement/placement_config.yaml)")
    a.add_argument("--device", help="YOLO device, e.g. cpu")
    a.add_argument("--replay", help="a ~/microwave_place_logs/<run> dir: its frames + arm state (dry run only)")
    a.add_argument("--yes", action="store_true", help="skip the typed 'go' confirmations")
    args = a.parse_args()

    over = {}
    if args.container_drop is not None:
        over.setdefault("container", {})["drop"] = args.container_drop
    if args.lowering:
        over.setdefault("lowering", {})["mode"] = args.lowering
    if args.device:
        over.setdefault("detector", {})["device"] = args.device
    cfg = load_config(args.config, over)
    if args.replay and args.execute:
        sys.exit("--replay is planning only; refusing --execute")

    try:
        if args.command == "release":
            arm = _build(args, cfg)[0] if not args.replay else sys.exit("release has nothing to replay")
            rp = wf.plan_release(arm, cfg)
            if not args.execute:
                print("\nDRY RUN -- would open the gripper, lift, back out"
                      f"{' and park' if rp['park_leg'] is not None else ''}. Nothing commanded.")
                return
            _confirm("RELEASE the box and retract.", args.yes)
            ok, msg = wf.execute_release(arm, rp)
            print(f"\n{msg}")
            sys.exit(0 if ok else 1)

        arm, source, det = _build(args, cfg)
        per = _perceive(arm, source, det, cfg)
        if args.command == "perceive":
            print(f"\nPERCEIVE ONLY. {per.cavity.summary()}\noverlays/frames: {per.log_dir}")
            return
        plan = wf.plan_placement(arm, per, cfg)
        print("\nPLAN\n" + plan.summary() + f"\n(logged to {plan.log_dir})")
        if args.command == "plan" or not args.execute:
            print("\nDRY RUN -- nothing commanded.")
            return
        stop = args.stop_after if args.command == "place" else "lower"
        _confirm(f"Execute through '{stop}' ({plan.lowering} lowering), hand stays on the box.", args.yes)
        ok, msg = wf.execute_placement(arm, plan, cfg, stop_after=stop)
        print(f"\n{msg}")
        if not ok or args.command != "all":
            sys.exit(0 if ok else 1)
        rp = wf.plan_release(arm, cfg)
        _confirm("Placement done. RELEASE the box and retract.", args.yes)
        ok, msg = wf.execute_release(arm, rp)
        print(f"\n{msg}")
        sys.exit(0 if ok else 1)
    except wf.PlacementRefused as e:
        sys.exit(f"REFUSED: {e}")


def _clean_exit(code):
    """Shut the shared rclpy node down before exiting (see scripts/detect_handle_sam3.py)."""
    sys.stdout.flush()
    sys.stderr.flush()
    try:
        from feeding_deployment.ros2.node import shutdown
        shutdown()
    except Exception:  # noqa: BLE001
        pass
    os._exit(code)


if __name__ == "__main__":
    _code = 0
    try:
        main()
    except SystemExit as e:
        if isinstance(e.code, int) or e.code is None:
            _code = e.code or 0
        else:
            print(e.code, file=sys.stderr)
            _code = 1
    _clean_exit(_code)
