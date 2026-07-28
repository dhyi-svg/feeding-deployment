"""Sim-complete validation of the REAL OpenDoorHLA.open_microwave() (no hand-
written motion scripts): robot_interface=None, rviz_interface=None,
web_interface=None, perception in replay mode against a handle_opening_pos.pkl
seeded from a real handle pose (default: the 2026-07-14 teleop ground truth,
not perception -- the ~16cm depth bias is unfixed).

Runs execute_action() through the repo's own open_microwave.yaml behavior
tree (same construction pattern as integration/test_actions.py's
test_OpenDoorHLA), then, using the same live PyBullet client:
  1. screenshots the arm at every joint-space staging/retract config the HLA
     actually moved through (left_back_retract_pos, fridge_door_staging_pos,
     left_retract_pos -- discovered by instrumenting move_to_joint_positions,
     not hardcoded, so any new config the HLA starts using also gets caught);
  2. collision-checks (p.getClosestPoints) every state in sim.recorded_states
     -- the full trajectory plan_to_joint_positions/plan_to_ee_pose actually
     produced for the HLA's moves -- against arm self-collision and every
     other body in the scene (including the floor and the microwave, which
     the sim's own get_collision_ids() / motion planner do NOT check --
     that's the "collision-blindness gap" this script exists to close).

Usage:
  .venv/bin/python scripts/validate_open_microwave_hla_sim.py \\
      --log_dir src/feeding_deployment/integration/log/microwave_hla_sim_validation
(expects handle_opening_pos.pkl already in --log_dir; see
draw_microwave_waypoints.py --handle-pos to generate one)
"""
import argparse
import json
import queue
import shutil
from pathlib import Path

import cv2
import numpy as np
import pybullet as p
from pybullet_helpers.inverse_kinematics import add_fingers_to_joint_positions
from relational_structs import Object

from feeding_deployment.actions.base import appliance_type
from feeding_deployment.actions.open_door import OpenDoorHLA
from feeding_deployment.integration.data_logger import DataLogger
from feeding_deployment.interfaces.perception_interface import PerceptionInterface
from feeding_deployment.simulation.scene_description import (
    create_scene_description_from_config,
)
from feeding_deployment.simulation.simulator import FeedingDeploymentPyBulletSimulator

CLEARANCE_THRESH_M = 0.02  # flag anything closer than 2cm
SEARCH_RADIUS_M = 0.05     # how far getClosestPoints looks for a nearest point

# Staging/retract configs open_microwave() moves through (attribute names on
# scene_description) -- recorded by instrumenting move_to_joint_positions
# below rather than hardcoded, so this list is a record of what was actually
# observed, not an assumption.
_staging_configs_seen: list[str] = []


def _instrument_staging_capture(hla: OpenDoorHLA) -> None:
    """Wrap move_to_joint_positions to record which named scene_description
    config (if any) each call corresponds to, in call order, deduplicated."""
    sd = hla.sim.scene_description
    name_by_value = {}
    for attr in dir(sd):
        if attr.endswith("_pos") and not attr.startswith("_"):
            try:
                val = getattr(sd, attr)
            except Exception:
                continue
            if isinstance(val, (list, tuple)) and len(val) == 7:
                name_by_value[tuple(np.round(val, 6))] = attr

    orig = hla.move_to_joint_positions

    def wrapped(joint_positions):
        key = tuple(np.round(joint_positions, 6))
        name = name_by_value.get(key, f"<unnamed {np.round(joint_positions, 3)}>")
        if not _staging_configs_seen or _staging_configs_seen[-1] != name:
            _staging_configs_seen.append(name)
        return orig(joint_positions)

    hla.move_to_joint_positions = wrapped


def _camera_matrices(target, eye, fov=55, aspect=1024 / 768):
    view = p.computeViewMatrix(cameraEyePosition=eye, cameraTargetPosition=target,
                                cameraUpVector=[0, 0, 1])
    proj = p.computeProjectionMatrixFOV(fov=fov, aspect=aspect, nearVal=0.05, farVal=3.0)
    return view, proj


def _screenshot(physics_client_id, out_path: Path, target, eye):
    view, proj = _camera_matrices(target, eye)
    w, h, rgb, _, _ = p.getCameraImage(
        1024, 768, viewMatrix=view, projectionMatrix=proj,
        renderer=p.ER_TINY_RENDERER, physicsClientId=physics_client_id,
    )
    img = np.reshape(rgb, (h, w, 4))[:, :, :3].astype(np.uint8)
    cv2.imwrite(str(out_path), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))


def _env_body_ids(sim) -> dict[str, int]:
    ids = {
        "floor": sim.floor_id,
        "table": sim.table_id,
        "robot_holder": sim.robot_holder_id,
        "wheelchair": sim._wheelchair_id,
        "microwave": sim._microwave_id,
        "refrigerator": sim._refridgerator_id,
        "sink": sim._sink_id,
        "conservative_bb": sim.conservative_bb_id,
    }
    for i, wid in enumerate(sim.wall_ids):
        ids[f"wall_{i}"] = wid
    return {k: v for k, v in ids.items() if v is not None}


def _closest_distance(physics_client_id, body_a, body_b, link_a=None, link_b=None):
    kwargs = dict(bodyA=body_a, bodyB=body_b, distance=SEARCH_RADIUS_M,
                  physicsClientId=physics_client_id)
    if link_a is not None:
        kwargs["linkIndexA"] = link_a
    if link_b is not None:
        kwargs["linkIndexB"] = link_b
    pts = p.getClosestPoints(**kwargs)
    if not pts:
        return None
    return min(pt[8] for pt in pts)  # contactDistance


def _check_config(sim, label, env_ids) -> list[dict]:
    """Collision-check the arm's CURRENT pose (caller must have already set
    joints) against the environment and itself. Returns flagged findings."""
    findings = []
    robot_id = sim.robot.robot_id
    pcid = sim.physics_client_id

    for env_name, env_id in env_ids.items():
        d = _closest_distance(pcid, robot_id, env_id)
        if d is not None and d < CLEARANCE_THRESH_M:
            findings.append({"config": label, "kind": "env", "against": env_name,
                              "distance_m": round(float(d), 4)})

    num_joints = p.getNumJoints(robot_id, physicsClientId=pcid)
    link_ids = list(range(-1, num_joints))
    for i_idx, i in enumerate(link_ids):
        for j in link_ids[i_idx + 2:]:  # skip self and directly-adjacent link
            d = _closest_distance(pcid, robot_id, robot_id, link_a=i, link_b=j)
            if d is not None and d < CLEARANCE_THRESH_M:
                findings.append({"config": label, "kind": "self", "against": f"link{i}_vs_link{j}",
                                  "distance_m": round(float(d), 4)})
    return findings


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log_dir", type=Path,
                     default=Path("src/feeding_deployment/integration/log/microwave_hla_sim_validation"))
    ap.add_argument("--scene_config", type=str, default="vention")
    ap.add_argument("--transfer_type", type=str, default="skewer")
    args = ap.parse_args()

    log_dir = args.log_dir
    assert (log_dir / "handle_opening_pos.pkl").exists(), (
        f"{log_dir}/handle_opening_pos.pkl missing -- generate it first "
        "(scripts/draw_microwave_waypoints.py --handle-pos ...)"
    )

    run_behavior_tree_dir = log_dir / "behavior_trees"
    run_behavior_tree_dir.mkdir(exist_ok=True)
    original_bt_dir = Path(__file__).parents[1] / "src" / "feeding_deployment" / "actions" / "behavior_trees"
    for f in original_bt_dir.glob("*.yaml"):
        shutil.copy(f, run_behavior_tree_dir)

    gesture_detectors_dir = log_dir / "gesture_detectors"
    gesture_detectors_dir.mkdir(exist_ok=True)
    orig_gesture_file = (Path(__file__).parents[1] / "src" / "feeding_deployment" / "perception"
                          / "gestures_perception" / "synthesized_gesture_detectors.py")
    if orig_gesture_file.exists():
        shutil.copy(orig_gesture_file, gesture_detectors_dir)

    execution_log = log_dir / "execution_log.txt"
    execution_log.write_text("")

    scene_config_path = (Path(__file__).parents[1] / "src" / "feeding_deployment" / "simulation"
                          / "configs" / f"{args.scene_config}.yaml")
    scene_description = create_scene_description_from_config(str(scene_config_path), args.transfer_type)
    sim = FeedingDeploymentPyBulletSimulator(scene_description, use_gui=False)

    data_logger = DataLogger(state_dir=log_dir)
    perception_interface = PerceptionInterface(robot_interface=None, data_logger=data_logger)
    assert perception_interface.simulation is True, "expected replay-from-pkl mode (robot_interface=None)"

    hla_hyperparams = {"max_motion_planning_time": 10.0}
    hla = OpenDoorHLA(
        sim, None, perception_interface, None, None, hla_hyperparams, None, None,
        False, log_dir, run_behavior_tree_dir, execution_log, gesture_detectors_dir,
    )
    _instrument_staging_capture(hla)

    sim.held_object_name = None
    appliance_obj = Object("microwave", appliance_type)

    print("Executing OpenDoorHLA.execute_action(microwave) via the real behavior tree ...")
    hla.execute_action(objects=[appliance_obj], params={})
    print(f"HLA execution complete. {len(sim.recorded_states)} recorded states across the run.")
    print(f"Staging configs visited, in order: {_staging_configs_seen}")

    # --- staging-pose screenshots -------------------------------------------------
    shots_dir = log_dir / "staging_screenshots"
    shots_dir.mkdir(exist_ok=True)
    named_configs = [c for c in _staging_configs_seen if not c.startswith("<unnamed")]
    seen_names = []
    for name in named_configs:
        if name in seen_names:
            continue
        seen_names.append(name)
        joints = add_fingers_to_joint_positions(sim.robot, getattr(scene_description, name))
        sim.robot.set_joints(joints)
        ee_pos = sim.robot.get_end_effector_pose().position
        out_path = shots_dir / f"{name}.png"
        _screenshot(sim.physics_client_id, out_path,
                    target=list(ee_pos), eye=[ee_pos[0] + 1.1, ee_pos[1] - 0.9, ee_pos[2] + 0.6])
        print(f"  screenshot: {name} -> {out_path}")

    # --- collision checks: staging configs + full commanded trajectory ------------
    env_ids = _env_body_ids(sim)
    print(f"Collision-checking against env bodies: {sorted(env_ids)}")
    all_findings = []
    for name in seen_names:
        joints = add_fingers_to_joint_positions(sim.robot, getattr(scene_description, name))
        sim.robot.set_joints(joints)
        all_findings.extend(_check_config(sim, f"staging:{name}", env_ids))

    for i, state in enumerate(sim.recorded_states):
        sim.robot.set_joints(state.robot_joints)
        all_findings.extend(_check_config(sim, f"trajectory[{i}]", env_ids))

    findings_path = log_dir / "collision_findings.json"
    findings_path.write_text(json.dumps(all_findings, indent=2))
    print(f"\n{len(all_findings)} clearance findings under {CLEARANCE_THRESH_M * 100:.0f}cm "
          f"across {len(seen_names)} staging configs + {len(sim.recorded_states)} trajectory states")
    for f in all_findings:
        print(f"  {f['config']:30s} {f['kind']:4s} vs {f['against']:20s} {f['distance_m'] * 100:.1f}cm")
    print(f"\nFull findings -> {findings_path}")
    print(f"Staging screenshots -> {shots_dir}")


if __name__ == "__main__":
    main()
