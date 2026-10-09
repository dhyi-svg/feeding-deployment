"""Microwave container placement: detector stabilisation, camera roll, point cloud, cavity and
placement geometry -- offline, on ray-cast synthetic RGB-D frames (no robot, camera or ROS).

    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_microwave_placement_perception.py -v
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "microwave" / "placement"))
import camera_roll  # noqa: E402
import cavity_perception as cp  # noqa: E402
import point_cloud as pcm  # noqa: E402
from microwave_detector import Detection, DetectionStabilizer, YoloMicrowaveDetector, iou, wait_for_stable  # noqa: E402
from placement_config import load_config  # noqa: E402
from synthetic_scene import MicrowaveScene, default_K, exterior_box_px, look_at_camera, render  # noqa: E402

CFG = load_config(overrides={"container": {"drop": 0.09}})
K = default_K()


def view(scene, roll=0.0, back=0.30, up=0.20):
    o = np.asarray(scene.origin)
    cam = o - scene.f * back + np.array([0, 0, up]) + scene.l * 0.01
    return look_at_camera(cam, o + scene.f * 0.16 + np.array([0, 0, 0.05]), roll)


def roi_cloud(scene, T, noise=1.0, seed=0):
    depth, _, _ = render(scene, T, K, noise_mm=noise, seed=seed)
    pts, _, _ = pcm.deproject(depth, K, roi=exterior_box_px(scene, T, K), stride=2)
    return pcm.filter_cloud(pcm.transform_points(T, pts), CFG.cloud), depth


# ---------------------------------------------------------------- point cloud / camera roll
def test_deproject_project_roundtrip():
    depth = np.full((480, 640), 600.0, np.float32)
    pts, uv, n = pcm.deproject(depth, K, roi=(100, 50, 140, 90), stride=4)
    assert n == len(pts) == 100
    np.testing.assert_allclose(pcm.project(pts, K), uv, atol=1e-6)
    np.testing.assert_allclose(pts[:, 2], 0.6)


def test_deproject_drops_invalid_depth():
    depth = np.zeros((480, 640), np.float32)
    depth[10:20, 10:20] = 500.0
    depth[15, 15] = np.nan
    pts, _, _ = pcm.deproject(depth, K)
    assert len(pts) == 99


@pytest.mark.parametrize("k", [0, 1, 2, 3])
def test_rotated_box_maps_back_to_raw_pixels(k):
    raw = np.zeros((480, 640), np.uint8)
    raw[100:150, 300:420] = 1                        # a box at x 300..419, y 100..149
    rot = np.rot90(raw, k)
    ys, xs = np.nonzero(rot)
    box = pcm.rot_box_to_raw((xs.min(), ys.min(), xs.max(), ys.max()), k, raw.shape)
    np.testing.assert_allclose(box, (300, 100, 419, 149))


@pytest.mark.parametrize("roll,k", [(0, 0), (90, 3), (-90, 1), (180, 2)])
def test_upright_k_follows_camera_roll(roll, k):
    T = look_at_camera([0.3, 0.0, 0.4], [0.8, 0.0, 0.2], roll)
    assert pcm.upright_k(T) == k
    v, _ = pcm.world_up_in_image(T)
    assert pcm.rot90_vec(v, k)[1] < -0.9              # world-up is image-up after the rotation


def test_self_and_held_box_points_removed():
    tool = np.eye(4)
    tool[:3, 3] = [0.5, 0.0, 0.3]
    pts = np.array([[0.5, 0.0, 0.3], [0.5, 0.0, 0.35], [0.9, 0.0, 0.3]])
    kept = pcm.remove_self_points(pts, tool, CFG.planner.self_filter_box)
    assert len(kept) == 1 and kept[0][0] == 0.9
    kept = pcm.remove_obb(np.array([[1.0, 0, 0], [2.0, 0, 0]]), np.array([1.0, 0, 0]), np.eye(3), [0.1, 0.1, 0.1])
    assert len(kept) == 1


def test_rotate_calibration_composes_about_optical_axis():
    calib = {"parameters": {"calibration_type": "eye_in_hand"},
             "transform": {"translation": {"x": 0.02, "y": 0.06, "z": -0.05},
                           "rotation": dict(zip("xyzw", R.from_euler("xyz", [5, -10, 175], degrees=True).as_quat()))}}
    out = camera_roll.rotate_calibration(calib, 90)
    q0 = R.from_quat([calib["transform"]["rotation"][c] for c in "xyzw"])
    q1 = R.from_quat([out["transform"]["rotation"][c] for c in "xyzw"])
    # the optical axis is unchanged, x turned 90 deg about it, the optical centre stays put
    np.testing.assert_allclose(q1.as_matrix()[:, 2], q0.as_matrix()[:, 2], atol=1e-9)
    np.testing.assert_allclose(q1.as_matrix()[:, 0], q0.as_matrix()[:, 1], atol=1e-9)
    np.testing.assert_allclose([out["transform"]["translation"][c] for c in "xyz"], [0.02, 0.06, -0.05])
    back = camera_roll.rotate_calibration(out, -90)
    np.testing.assert_allclose(R.from_quat([back["transform"]["rotation"][c] for c in "xyzw"]).as_matrix(),
                               q0.as_matrix(), atol=1e-9)
    piv = camera_roll.rotate_calibration(calib, 180, pivot=(0.0, 0.02, 0.0))
    shift = np.array([piv["transform"]["translation"][c] for c in "xyz"]) - [0.02, 0.06, -0.05]
    np.testing.assert_allclose(shift, q0.as_matrix() @ [0, 0.04, 0], atol=1e-9)
    assert yaml.safe_load(yaml.safe_dump(out))["parameters"]["calibration_type"] == "eye_in_hand"


@pytest.mark.parametrize("true_roll,published_roll,expect", [(90, 0, 90), (90, 90, 0), (-90, 0, -90)])
def test_roll_check_finds_the_missing_correction(true_roll, published_roll, expect):
    scene = MicrowaveScene()
    T_true = view(scene, true_roll)
    depth, _, _ = render(scene, T_true, K, noise_mm=1.0)
    T_pub = T_true @ camera_roll.roll_transform(published_roll - true_roll)   # what the stale calibration says
    scores = camera_roll.roll_scores(depth, K, T_pub, CFG.cavity, CFG.cloud)
    assert camera_roll.recommend(scores) == expect


# ---------------------------------------------------------------- detector stabilisation
def det(box, t=0.0, conf=0.8):
    return Detection(tuple(map(float, box)), conf, "microwave", t)


def test_stabilizer_needs_three_consistent():
    s = DetectionStabilizer(CFG.stabilizer)
    s.add(det((100, 100, 300, 260)))
    s.add(det((102, 99, 301, 262)))
    assert s.stable()[0] is None
    s.add(det((99, 101, 299, 259)))
    box, inl = s.stable()
    assert box is not None and len(inl) == 3
    assert iou(box, (100, 100, 300, 260)) > 0.95


def test_stabilizer_rejects_outliers():
    s = DetectionStabilizer(CFG.stabilizer)
    for b in [(100, 100, 300, 260), (400, 50, 600, 200), (101, 100, 300, 261), (10, 300, 90, 400), (100, 101, 299, 260)]:
        s.add(det(b))
    box, inl = s.stable()
    assert len(inl) == 3 and all(d.box[0] < 110 for d in inl)


def test_stabilizer_refuses_inconsistent_stream():
    s = DetectionStabilizer(CFG.stabilizer)
    for i in range(6):
        s.add(det((50 * i, 50, 50 * i + 120, 200)))
    assert s.stable()[0] is None


def test_wait_for_stable_times_out():
    t = [0.0]
    with pytest.raises(TimeoutError, match="no stable microwave detection"):
        wait_for_stable(lambda: None, CFG.stabilizer, log=lambda *_: None,
                        sleep=lambda s: t.__setitem__(0, t[0] + s), clock=lambda: t[0])


def test_wait_for_stable_ignores_missed_frames():
    seq = iter([None, det((100, 100, 300, 260)), None, det((101, 100, 300, 260)), det((100, 99, 300, 260))])
    box, inl = wait_for_stable(lambda: next(seq), CFG.stabilizer, log=lambda *_: None, sleep=lambda s: None)
    assert len(inl) == 3


def test_yolo_detector_rotates_upright_and_maps_back(monkeypatch):
    """The detector runs on the upright image and returns RAW pixels (model stubbed)."""
    d = YoloMicrowaveDetector.__new__(YoloMicrowaveDetector)
    d.cfg = CFG.detector
    seen = {}

    def fake_predict(img, classes):
        seen["shape"] = img.shape
        ys, xs = np.nonzero(img[:, :, 0])
        return np.array([xs.min(), ys.min(), xs.max(), ys.max()], float), 0.9, "microwave"

    d._predict = fake_predict
    raw = np.zeros((480, 640, 3), np.uint8)
    raw[200:260, 100:400] = 255
    T = look_at_camera([0.3, 0.0, 0.4], [0.8, 0.0, 0.2], 90)          # camera rolled 90 deg
    out = d.detect(raw, T, stamp=1.0)
    assert seen["shape"] == (640, 480, 3) and out.k_upright == 3
    np.testing.assert_allclose(out.box, (100, 200, 399, 259))


# ---------------------------------------------------------------- cavity from depth
@pytest.mark.parametrize("roll", [0, 90, -90])
@pytest.mark.parametrize("scene_kw", [{}, {"yaw_deg": -12, "door_side": -1.0}, {"turntable": 0.0, "width": 0.36},
                                      {"origin": (0.70, -0.10, 0.20), "height": 0.24}])
def test_cavity_bounds_from_depth(roll, scene_kw):
    scene = MicrowaveScene(**scene_kw)
    cloud, _ = roi_cloud(scene, view(scene, roll))
    cav = cp.estimate_cavity(cloud, view(scene, roll)[:3, 3], CFG.cavity)
    t = scene.truth()
    shell_front = t["front"] - 0.02                    # the synthetic shell starts 2 cm before the opening
    assert np.degrees(np.arccos(cav.f @ scene.f)) < 2.0
    assert abs(cav.floor_z - t["floor_z"]) < 0.005
    assert cav.floor_tilt_deg() < 2.0
    assert abs(cav.back - t["back"]) < 0.01
    assert shell_front - 0.01 < cav.front < t["front"] + 0.04
    assert abs(cav.left - t["left"]) < 0.015 and abs(cav.right - t["right"]) < 0.015
    assert t["top"] - 0.04 < cav.top <= t["top"] + 0.01   # never above the real top


def test_wrong_camera_roll_is_refused():
    scene = MicrowaveScene()
    T_true = view(scene, 90)
    depth, _, _ = render(scene, T_true, K, noise_mm=1.0)
    T_wrong = T_true @ camera_roll.roll_transform(-90)    # the calibration without the remount
    pts, _, _ = pcm.deproject(depth, K, roi=exterior_box_px(scene, T_true, K), stride=2)
    cloud = pcm.filter_cloud(pcm.transform_points(T_wrong, pts), CFG.cloud)
    with pytest.raises(cp.CavityError):
        cp.estimate_cavity(cloud, T_wrong[:3, 3], CFG.cavity)


def test_closed_door_is_refused():
    scene = MicrowaveScene(door_open=False)
    o, f = np.asarray(scene.origin), scene.f
    from synthetic_scene import OBB
    scene.boxes.append(OBB(o + f * -0.04 + np.array([0, 0, 0.1]), np.column_stack([f, scene.l, [0, 0, 1]]),
                           np.array([0.02, 0.25, 0.16]), "closed door"))
    cloud, _ = roi_cloud(scene, view(scene))
    with pytest.raises(cp.CavityError):
        cp.estimate_cavity(cloud, view(scene)[:3, 3], CFG.cavity)


def test_too_few_points_refused():
    with pytest.raises(cp.CavityError, match="only"):
        cp.estimate_cavity(np.random.default_rng(0).normal(size=(100, 3)), np.zeros(3), CFG.cavity)


def test_merge_requires_agreeing_looks():
    scene = MicrowaveScene()
    looks = [cp.estimate_cavity(roi_cloud(scene, view(scene), seed=s)[0], view(scene)[:3, 3], CFG.cavity, seed=s)
             for s in range(3)]
    merged, inl = cp.merge_cavities(looks, CFG.cavity)
    assert len(inl) == 3 and merged.width <= min(c.width for c in looks) + 1e-9
    bad = cp.Cavity(**{**looks[0].__dict__, "floor_z": looks[0].floor_z + 0.05})
    with pytest.raises(cp.CavityError, match="agree"):
        cp.merge_cavities([looks[0], looks[1], bad], CFG.cavity)


# ---------------------------------------------------------------- placement target
LEVEL_HOLD = R.from_matrix(np.column_stack([[0, 0, 1], [0, -1, 0], [1, 0, 0]])).as_quat()   # approach +x, tool x up


def target_for(scene, quat=LEVEL_HOLD, cfg=CFG):
    T = view(scene)
    cloud, depth = roi_cloud(scene, T)
    cav = cp.estimate_cavity(cloud, T[:3, 3], cfg.cavity)
    allp, _, _ = pcm.deproject(depth, K, stride=3)
    return cav, cp.placement_target(cav, pcm.transform_points(T, allp), quat, cfg.container, cfg.placement, cfg.cavity)


def test_placement_target_inside_cavity_and_level():
    scene = MicrowaveScene(origin=(0.62, 0.02, 0.12), yaw_deg=8, height=0.24)
    cav, tgt = target_for(scene)
    c = CFG.container
    # aligned: approach along the cavity axis and horizontal; the tool's up stays up
    Rm = R.from_quat(tgt.quat).as_matrix()
    np.testing.assert_allclose(Rm[:, 2], cav.f, atol=1e-9)
    assert Rm[:, 0] @ [0, 0, 1] > 0.999
    # box footprint inside the walls with the clearances, on the support (turntable)
    bc = tgt.box_center
    assert abs(bc @ cav.l - (cav.left + cav.right) / 2) < 1e-6
    assert bc @ cav.f - (c.far_past_tool - c.near_past_tool) / 2 >= cav.front + CFG.placement.front_inside_m - 1e-6
    assert bc @ cav.f + (c.far_past_tool - c.near_past_tool) / 2 <= cav.back - CFG.placement.back_clearance_m + 1e-6
    assert abs(tgt.support_z - scene.truth()["support_z"]) < 0.006
    assert abs(tgt.contact[2] - (tgt.support_z + c.drop)) < 1e-9
    assert tgt.above[2] - tgt.contact[2] == pytest.approx(CFG.placement.insert_clearance_m)
    # pre-insert: same height and line as 'above', box far end in front of the opening
    np.testing.assert_allclose(np.cross(tgt.above - tgt.pre, cav.f), 0, atol=1e-9)
    assert tgt.pre @ cav.f + c.far_past_tool <= cav.front - CFG.placement.pre_insert_standoff_m + 1e-6


@pytest.mark.parametrize("over,match", [
    ({"container": {"drop": -1.0}}, "not set"),
    ({"container": {"drop": 0.09, "width": 0.40}}, "side walls"),
    ({"container": {"drop": 0.09, "height": 0.25}}, "too tall"),
    ({"container": {"drop": 0.09, "far_past_tool": 0.45}}, "too long"),
    ({"container": {"drop": 0.09}, "placement": {"max_reach_m": 0.5}}, "reach"),
])
def test_placement_target_refusals(over, match):
    cfg = load_config(overrides=over)
    with pytest.raises(cp.CavityError, match=match):
        target_for(MicrowaveScene(origin=(0.62, 0.02, 0.12), height=0.24), cfg=cfg)


def test_alignment_limits():
    scene = MicrowaveScene(origin=(0.62, 0.02, 0.12), height=0.24)
    yawed = (R.from_euler("z", 40, degrees=True) * R.from_quat(LEVEL_HOLD)).as_quat()
    with pytest.raises(cp.CavityError, match="off the cavity axis"):
        target_for(scene, yawed)
    tilted = (R.from_euler("y", -15, degrees=True) * R.from_quat(LEVEL_HOLD)).as_quat()
    with pytest.raises(cp.CavityError, match="tilted"):
        target_for(scene, tilted)


def test_something_in_the_microwave_is_refused():
    scene = MicrowaveScene(origin=(0.62, 0.02, 0.12), height=0.24)
    from synthetic_scene import OBB
    o = np.asarray(scene.origin)
    scene.boxes.append(OBB(o + scene.f * 0.15 + np.array([0, 0, 0.06]), np.column_stack([scene.f, scene.l, [0, 0, 1]]),
                           np.array([0.04, 0.04, 0.05]), "mug"))
    with pytest.raises(cp.CavityError, match="something is in the microwave"):
        target_for(scene)


def test_config_rejects_unknown_keys_and_bad_mode():
    with pytest.raises(ValueError, match="unknown config key"):
        load_config(overrides={"container": {"dorp": 0.1}})
    with pytest.raises(ValueError, match="lowering.mode"):
        load_config(overrides={"lowering": {"mode": "force"}})
