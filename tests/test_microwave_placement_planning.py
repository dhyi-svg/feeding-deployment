"""Microwave container placement: planning in the PyBullet sim, the impedance lowering, and the
full perceive -> plan -> execute -> release sequence against a mocked ArmInterfaceClient
(records every command; nothing reaches a robot).

    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_microwave_placement_planning.py -v

Run from the repo root (the sim config path is relative). ~1-2 min (PyBullet).
"""
from __future__ import annotations

import os
import sys
import time
from pathlib import Path
from unittest import mock

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "microwave" / "placement"))
sys.path.insert(0, str(ROOT / "microwave"))
os.chdir(ROOT)

import impedance_lowering as il  # noqa: E402
import placement_workflow as wf  # noqa: E402
from microwave_detector import Detection  # noqa: E402
from placement_config import load_config  # noqa: E402
from placement_planner import PlacementSim  # noqa: E402
from synthetic_scene import MicrowaveScene, default_K, exterior_box_px, look_at_camera, render  # noqa: E402

K = default_K()
# 09-29 container-hold joints (J6 +80: the "J6 > 0" wrist branch) and the same hand pose with the
# wrist flipped and J6 pulled to the compliant model's -67.6 (found by the planner's own IK)
Q_HOLD = np.radians([28.92, 38.11, -174.13, -130.59, 43.67, 80.14, -6.90])
Q_HOLD_FLIPPED = np.radians([50.0, 41.0, 157.0, -130.0, -138.0, -68.0, -173.0])
SCENE = dict(origin=(0.80, -0.08, 0.13), yaw_deg=9, height=0.24)


def cfg_for(mode):
    return load_config(overrides={"container": {"drop": 0.085}, "lowering": {"mode": mode},
                                  "stabilizer": {"frame_period_s": 0.0}})


@pytest.fixture(scope="module")
def sim():
    return PlacementSim(cfg_for("position"))


class Source:
    def __init__(self, scene, T):
        self.scene, self.T, self.i = scene, T, 0

    def get(self):
        self.i += 1
        d, b, _ = render(self.scene, self.T, K, noise_mm=1.0, seed=self.i)
        return wf.Frame(b, d, K, self.T, time.time())


class BoxDetector:
    """Stands in for YOLO: the microwave shell's projected box."""

    def __init__(self, scene, T):
        self.box = exterior_box_px(scene, T, K)

    def detect(self, bgr, T, stamp):
        return Detection(self.box, 0.9, "microwave", stamp)


class ArmMock:
    """unittest.mock client: joint commands land instantly, compliant z stops at `floor_z`."""

    def __init__(self, sim, q, model_j6):
        self.sim, self.s = sim, {"q": np.array(q, float), "g": 0.95, "comp": False, "c": None, "floor": None}
        m = mock.MagicMock()
        m.in_compliant_mode = False
        m.get_state.side_effect = self.get_state
        m.execute_command.side_effect = self.execute
        m.switch_to_task_compliant_mode.side_effect = self.on
        m.switch_out_of_compliant_mode.side_effect = self.off
        self.m, self.model_j6 = m, model_j6

    def get_state(self):
        s = self.s
        if s["comp"]:
            q = s["q"].copy()
            q[5] = self.model_j6
            return {"position": q, "ee_pos": np.concatenate(s["c"]), "gripper_pos": s["g"]}
        p, qq = self.sim.fk(s["q"])
        return {"position": s["q"].copy(), "ee_pos": np.concatenate([p, qq]), "gripper_pos": s["g"]}

    def execute(self, cmd):
        n, s = type(cmd).__name__, self.s
        if n == "JointCommand":
            s["q"] = np.asarray(cmd.pos, float)
        elif n == "CartesianCommand":
            t = np.array(cmd.pos, float)
            if s["floor"] is not None:
                t[2] = max(t[2], s["floor"])
            s["c"][0] = s["c"][0] + 0.6 * (t - s["c"][0])
        elif n == "OpenGripperCommand":
            s["g"] = 0.01
        return True

    def on(self):
        self.s["comp"] = True
        self.s["c"] = list(self.sim.fk(self.s["q"]))
        self.m.in_compliant_mode = True

    def off(self):
        self.s["q"] = self.sim.ik_locked_j6(*self.s["c"], self.s["q"], self.s["q"][5])[0]
        self.s["comp"] = False
        self.m.in_compliant_mode = False


def perceive_and_plan(sim, q, mode, tmp_path):
    cfg = cfg_for(mode)
    scene = MicrowaveScene(**SCENE)
    pos, quat = sim.fk(q)
    T = look_at_camera(pos + [-0.08, 0.0, 0.12], [0.95, -0.05, 0.18], 90)    # camera mounted rolled 90 deg
    arm = ArmMock(sim, q, cfg.lowering.model_j6_rad)
    per = wf.perceive(Source(scene, T), BoxDetector(scene, T), cfg, tool_pose=(pos, quat), log=lambda *_: None,
                      log_dir=tmp_path, sleep=lambda s: None)
    return cfg, scene, arm, per


@pytest.fixture(autouse=True)
def _records(tmp_path, monkeypatch):
    monkeypatch.setattr(wf, "PLACE_RECORD", tmp_path / "place.json")
    monkeypatch.setattr(wf.execute_joint_plan.__globals__["time"], "sleep", lambda s: None)


# ---------------------------------------------------------------- the compliant model's J6
def test_compliant_model_j6_matches_the_hack_urdf():
    pin = pytest.importorskip("pinocchio")
    d = ROOT / "src/feeding_deployment/control/robot_controller/urdfs"
    full = pin.buildModelFromUrdf(str(d / "gen3_robotiq_2f_85.urdf"))
    hack = pin.buildModelFromUrdf(str(d / "hack_gen3_robotiq_2f_85.urdf"))

    def qpin(q, h):
        c = [np.cos(q[0]), np.sin(q[0]), q[1], np.cos(q[2]), np.sin(q[2]), q[3], np.cos(q[4]), np.sin(q[4])]
        return np.array(c + ([np.cos(q[6]), np.sin(q[6])] if h else [q[5], np.cos(q[6]), np.sin(q[6])]))

    j6 = load_config().lowering.model_j6_rad
    for q in np.random.default_rng(0).uniform(-1.5, 1.5, (5, 7)):
        q[5] = j6
        fd, hd = full.createData(), hack.createData()
        pin.framesForwardKinematics(full, fd, qpin(q, False))
        pin.framesForwardKinematics(hack, hd, qpin(q, True))
        err = np.linalg.norm(fd.oMf[full.getFrameId("tool_frame")].translation
                             - hd.oMf[hack.getFrameId("tool_frame")].translation)
        assert err < 1e-3


# ---------------------------------------------------------------- full sequence
@pytest.mark.parametrize("mode,q", [("impedance", Q_HOLD_FLIPPED), ("position", Q_HOLD)])
def test_full_sequence_offline(sim, tmp_path, mode, q):
    cfg, scene, arm, per = perceive_and_plan(sim, q, mode, tmp_path)
    plan = wf.plan_placement(arm.m, per, cfg, log=lambda *_: None)
    assert [leg.label for leg in plan.legs] == ["align + to pre-insert", "insert", "lower"]
    assert plan.lowering == mode
    for leg in plan.legs[1:]:
        assert leg.worst_tilt_deg <= cfg.placement.level_tol_deg
        assert leg.worst_clearance[0] >= cfg.planner.lower_min_clearance_m
        if mode == "impedance":
            assert all(abs(j[5] - cfg.lowering.model_j6_rad) < np.radians(0.5) for j in leg.joints)
    assert (tmp_path / "plan.json").exists() and (tmp_path / "scene_cloud.npy").exists()

    arm.s["floor"] = plan.target.contact[2]
    ok, msg = wf.execute_placement(arm.m, plan, cfg, log=lambda *_: None)
    assert ok, msg
    names = [type(c.args[0]).__name__ for c in arm.m.execute_command.call_args_list]
    assert "OpenGripperCommand" not in names                        # the hand stays on the box
    if mode == "impedance":
        arm.m.switch_to_task_compliant_mode.assert_called_once()
        arm.m.switch_out_of_compliant_mode.assert_called_once()
        zs = [c.args[0].pos[2] for c in arm.m.execute_command.call_args_list
              if type(c.args[0]).__name__ == "CartesianCommand"]
        assert min(zs) >= plan.target.contact[2] - cfg.lowering.press_m - 1e-6   # never commanded past the press limit
        assert "contact" in msg
    else:
        arm.m.switch_to_task_compliant_mode.assert_not_called()
    end = arm.get_state()["ee_pos"][:3]
    assert abs(end[2] - (plan.target.contact if mode == "impedance" else plan.target.place)[2]) < 0.005

    rp = wf.plan_release(arm.m, cfg, log=lambda *_: None)
    n_before = len(arm.m.execute_command.call_args_list)
    ok, msg = wf.execute_release(arm.m, rp, log=lambda *_: None)
    assert ok, msg
    after = [type(c.args[0]).__name__ for c in arm.m.execute_command.call_args_list[n_before:]]
    assert after[0] == "OpenGripperCommand" and set(after[1:]) == {"JointCommand"}
    out = arm.get_state()["ee_pos"][:3]
    assert out @ per.cavity.f < per.cavity.front                    # the hand is out of the microwave


def test_impedance_refused_on_the_wrong_wrist_branch(sim, tmp_path):
    cfg, scene, arm, per = perceive_and_plan(sim, Q_HOLD, "impedance", tmp_path)
    with pytest.raises(wf.PlacementRefused, match="wrist flipped"):
        wf.plan_placement(arm.m, per, cfg, log=lambda *_: None)
    arm.m.execute_command.assert_not_called()


def test_stop_after_and_moved_arm(sim, tmp_path):
    cfg, scene, arm, per = perceive_and_plan(sim, Q_HOLD, "position", tmp_path)
    plan = wf.plan_placement(arm.m, per, cfg, log=lambda *_: None)
    ok, msg = wf.execute_placement(arm.m, plan, cfg, stop_after="pre-insert", log=lambda *_: None)
    assert ok and "pre-insert" in msg
    np.testing.assert_allclose(arm.s["q"], plan.legs[0].joints[-1])
    with pytest.raises(wf.PlacementRefused, match="moved"):           # the plan's start is gone now
        wf.execute_placement(arm.m, plan, cfg, log=lambda *_: None)


def test_open_gripper_refused(sim, tmp_path):
    cfg, scene, arm, per = perceive_and_plan(sim, Q_HOLD, "position", tmp_path)
    arm.s["g"] = 0.01
    with pytest.raises(wf.PlacementRefused, match="open"):
        wf.plan_placement(arm.m, per, cfg, log=lambda *_: None)


def test_obstacle_at_the_start_is_refused(sim, tmp_path):
    cfg, scene, arm, per = perceive_and_plan(sim, Q_HOLD, "position", tmp_path)
    plan = wf.plan_placement(arm.m, per, cfg, log=lambda *_: None)
    mid = (plan.target.pre + plan.target.above) / 2
    per.scene_cloud = np.vstack([per.scene_cloud, mid + np.random.default_rng(0).uniform(-0.03, 0.03, (400, 3))])
    with pytest.raises(wf.PlacementRefused, match="starts"):
        wf.plan_placement(arm.m, per, cfg, log=lambda *_: None)


def test_scene_voxels_on_the_path_block_a_leg(sim):
    from placement_planner import PlanError
    cfg = cfg_for("position")
    s = PlacementSim(cfg)
    pos, quat = s.fk(Q_HOLD)
    s.attach_box(cfg.container, np.array([1.0, 0.0, 0.0]))
    p1 = pos + np.array([0.0, 0.0, 0.10])
    s.plan_leg("free", pos, quat, p1, quat, Q_HOLD, [], 0.03, level=False, log=lambda *_: None)
    s.add_voxels(pos + np.array([0.10, 0.0, 0.13]) + np.random.default_rng(0).uniform(-0.03, 0.03, (50, 3)), 0.03)
    with pytest.raises(PlanError, match="clearance"):
        s.plan_leg("blocked", pos, quat, p1, quat, Q_HOLD, s.voxels, 0.03, level=False, log=lambda *_: None)


# ---------------------------------------------------------------- impedance lowering alone
class Compliant:
    """Minimal compliant-mode client for lower_with_impedance."""

    def __init__(self, floor_z=None, j6=-1.18039928, drift=0.0, fail=False):
        self.x = np.array([0.8, 0.0, 0.30])
        self.q = np.zeros(7)
        self.q[5] = j6
        self.floor_z, self.drift, self.fail = floor_z, drift, fail
        self.in_compliant_mode = False
        self.cmds = []
        self.out_calls = 0

    def get_state(self):
        if self.fail and self.in_compliant_mode and len(self.cmds) > 3:
            raise RuntimeError("Emergency stop is active")
        return {"position": self.q, "ee_pos": np.concatenate([self.x, [0, 0, 0, 1]]), "gripper_pos": 0.9}

    def switch_to_task_compliant_mode(self):
        self.in_compliant_mode = True

    def switch_out_of_compliant_mode(self):
        self.out_calls += 1
        self.in_compliant_mode = False

    def execute_command(self, cmd):
        t = np.array(cmd.pos, float)
        if self.floor_z is not None:
            t[2] = max(t[2], self.floor_z)
        self.x = self.x + 0.6 * (t - self.x) + [self.drift, 0, 0]
        self.cmds.append(np.asarray(cmd.pos, float))


def run_lower(arm, drop=0.04, **over):
    lc = load_config(overrides={"lowering": over}).lowering
    t = [0.0]
    return il.lower_with_impedance(arm, drop, lc, log=lambda *_: None, clock=lambda: t[0],
                                   sleep=lambda s: t.__setitem__(0, t[0] + s)), lc


def test_impedance_stops_at_contact_and_holds_a_preload():
    arm = Compliant(floor_z=0.27)
    rep, lc = run_lower(arm)
    assert rep["ok"] and rep["contact"] and abs(rep["drop_m"] - 0.03) < 0.003
    assert arm.cmds[-1][2] == pytest.approx(0.27 - lc.hold_preload_m, abs=0.002)   # preload into the floor
    assert arm.out_calls == 1 and not arm.in_compliant_mode


def test_impedance_no_contact_stops_at_the_press_limit():
    arm = Compliant(floor_z=None)
    rep, lc = run_lower(arm, drop=0.04)
    assert rep["ok"] and not rep["contact"] and "NO CONTACT" in rep["reason"]
    assert min(c[2] for c in arm.cmds) >= 0.30 - 0.04 - lc.press_m - 1e-9


def test_impedance_aborts_on_sideways_drift():
    arm = Compliant(floor_z=0.27, drift=0.02)
    rep, _ = run_lower(arm)
    assert not rep["ok"] and "sideways" in rep["reason"] and arm.out_calls == 1


def test_impedance_leaves_compliant_mode_after_an_rpc_error():
    arm = Compliant(floor_z=0.27, fail=True)
    rep, _ = run_lower(arm)
    assert not rep["ok"] and "Emergency stop" in rep["reason"] and arm.out_calls == 1


def test_impedance_refuses_j6_off_the_model():
    arm = Compliant(j6=np.radians(80))
    with pytest.raises(il.ImpedanceRefused, match="J6"):
        run_lower(arm)
    assert not arm.in_compliant_mode and not arm.cmds
