"""The integrated software dry run of microwave/placement/place_move.py: staging -> perception -> target -> insertion
-> impedance lowering. No robot, no ROS, no GPU.

REAL: the 10-30 live frames' saved SAM 3 masks when present (else the synthetic microwave), perception, gates, the sim
FK/IK, lower.py. SIMULATED: the arm (FakeArm), cuRobo (test_xyz_move.LinePlanner), the floor contact, and the
microwave moved within reach.

    PYTHONPATH=src PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_place_move.py -q
"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "microwave"), str(ROOT / "microwave" / "placement"), str(ROOT / "tests")]
import cavity_segmentation as cs  # noqa: E402
import floor_target as ft  # noqa: E402
import lower as lw  # noqa: E402
import place_move as pm  # noqa: E402
import rgbd_source as rs  # noqa: E402
import xyz_move as xm  # noqa: E402
from scipy.spatial.transform import Rotation as R  # noqa: E402
from synthetic_scene import MicrowaveScene, default_K, look_at_camera, render  # noqa: E402
from test_xyz_move import LinePlanner, sim  # noqa: E402,F401
from test_placement_perception_part2 import opening_masks  # noqa: E402


class FakeArm:
    """Follows commands in the sim; a floor at `floor_tool_z` stops the tool; sags `sag` in impedance. Logs everything."""

    def __init__(self, sim, q, floor_tool_z, sag=0.03):
        self.sim, self.q, self.floor, self.sag = sim, np.asarray(q, float), floor_tool_z, sag
        self.grip, self.vel, self.log, self.monitor_stopped, self.last_duration = 0.01, np.zeros(3), [], False, 1.0
        self.dz, self.touched = 0.0, False

    def raw_joints(self):
        return self.q.copy()

    def get_state(self):
        p, rot = xm.fk(self.sim, self.q)
        p = p.copy()
        p[2] = max(p[2] + self.dz, self.floor)
        return {"ee_pos": np.concatenate([p, rot.as_quat()]), "ee_vel": self.vel.copy(), "gripper_pos": self.grip}

    def gripper(self, pos):
        self.log.append(("gripper", pos))
        self.grip = 0.6 if pos > 0.5 else 0.01

    def hold_joints(self, q, seconds, impedance=None, monitor=None):
        self.log.append(("hold", "impedance" if impedance else "position"))
        self.q = np.asarray(q, float)
        self.dz = -self.sag if impedance else 0.0
        self.vel = np.zeros(3)
        self.monitor_stopped = bool(monitor and monitor())
        return True

    def move_joints(self, knots, cart_lengths=None, speed_scale=1.0, impedance=None, monitor=None):
        self.log.append(("move", "impedance" if impedance else "position", len(knots)))
        self.monitor_stopped = False
        self.last_duration = 0.02 * len(knots) if not impedance else 0.3
        for u in np.linspace(0, 1, 40):
            q = np.asarray(knots[min(int(u * len(knots)), len(knots) - 1)], float)
            before = self.get_state()["ee_pos"][2]
            self.q = q
            hit = self.get_state()["ee_pos"][2] <= self.floor + 1e-6
            self.vel = np.array([0, 0, 0.0 if hit and before <= self.floor + 1e-6 else -0.02])
            if monitor and monitor():
                self.monitor_stopped = True
                self.vel = np.zeros(3)
                return True
            import time
            time.sleep(self.last_duration / 40)
        self.vel = np.zeros(3)
        return True


def scene_result(tmp_path):
    """The perception pipeline on a synthetic microwave the arm can reach (wrist view at roll 0)."""
    scene = MicrowaveScene(origin=(0.86, 0.0, 0.16), yaw_deg=0, height=0.26, turntable=0.0)
    cfg = ft.load_perception_config(overrides={"cavity_method": "opening"})
    T = look_at_camera(np.array([0.44, 0.0, 0.42]), np.array([1.04, 0.0, 0.25]))
    K = default_K()
    depth, bgr, _ = render(scene, T, K, noise_mm=1.0)
    fr = rs.RGBDFrame(bgr, depth / 1000.0, K, T, source="SIMULATED scene")
    res, _, _ = ft.run_pipeline([fr], None, cfg, min_looks=1, masks=[opening_masks(scene, fr)], log=lambda *_: None)
    assert res.frame == "arm_base_link" and res.grasp_offset is not None
    return res.to_dict(), cfg.container


def test_integrated_dry_run(sim, tmp_path):  # noqa: F811
    res, spec = scene_result(tmp_path)
    cfg = pm.load_config()
    staging = np.radians(cfg["staging_joints_deg"])
    planner = LinePlanner(sim)
    # target frame and grasp offset: centre = tool + R @ offset, and the tool sits below the centre by the offset
    rot = xm.target_orientation(res["yaw_deg"], cfg["roll_deg"])
    np.testing.assert_allclose(np.asarray(res["tool_center"]) + rot.apply(res["grasp_offset"]), res["container_center"], atol=1e-6)
    pts = pm.insertion_points(res, spec, cfg)
    assert all(v > 0.01 for v in pts["clear"].values()), pts["clear"]

    contact_z = pts["contact_z"]
    arm = FakeArm(sim, staging, contact_z)
    said = []
    ok = pm.run(arm, sim, planner, cfg, lambda: res, spec, True, lambda m: (said.append(m), "go")[1], tmp_path, log=lambda *_: None)
    assert ok
    kinds = [e[0:2] for e in arm.log]
    # order: closed before anything carries it, impedance only for the lowering, gripper never opened
    assert kinds.index(("gripper", 1.0)) < kinds.index(("hold", "impedance"))
    assert [e for e in arm.log if e[0] == "gripper"] == [("gripper", 1.0)]
    assert ("move", "impedance") in kinds
    assert all(e[1] == "position" for e in arm.log[:kinds.index(("hold", "impedance"))] if e[0] == "move")
    ee = arm.get_state()["ee_pos"]
    assert abs(ee[2] - contact_z) < 0.005 and np.linalg.norm(ee[:2] - np.asarray(res["tool_center"])[:2]) < 0.005
    assert xm.tilt_deg(R.from_quat(ee[3:7]), rot) < 1.0
    assert (tmp_path / "descent.csv").exists()

    # no go = nothing moves; an impedance sag onto the floor stops before lowering; no contact stops and leaves impedance
    arm = FakeArm(sim, staging, contact_z)
    try:
        pm.run(arm, sim, planner, cfg, lambda: res, spec, True, lambda m: "no", tmp_path, log=lambda *_: None)
    except xm.Refused:
        pass
    assert arm.log == []
    for sag, floor, msg in ((0.09, contact_z, "sagged"), (0.03, contact_z - 0.2, "without contact")):
        arm = FakeArm(sim, staging, floor, sag)
        try:
            pm.run(arm, sim, planner, cfg, lambda: res, spec, True, lambda m: "go", tmp_path, log=lambda *_: None)
            raise AssertionError("did not stop")
        except xm.Refused as e:
            assert msg in str(e) and "still held" in str(e)
        assert arm.log[-1] == ("hold", "position") and arm.grip > 0.2

    # the dry run commands nothing
    arm = FakeArm(sim, staging, contact_z)
    assert pm.run(arm, sim, planner, cfg, lambda: res, spec, False, lambda m: "go", tmp_path, log=lambda *_: None)
    assert arm.log == []
