"""Part 2 placement perception, software only (no robot, no camera, no ROS, no GPU): depth processing,
transforms, floor estimation, target selection, Part 1 compatibility, error handling.

SAM 3 is replaced by a fake that returns the true interior pixels of a ray-cast synthetic microwave
(synthetic_scene.py), so the 3D side is checked against known geometry; the real 2026-10-20 rig frame is
checked with the mask SAM 3 actually produced on it (tests/data/..._sam3mask.npz, score 0.91).

    PYTHONPATH=src PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_placement_perception_part2.py -q
"""
import json
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "microwave"))
sys.path.insert(0, str(ROOT / "microwave" / "placement"))
import cavity_segmentation as cs  # noqa: E402
import floor_target as ft  # noqa: E402
import point_cloud as pcm  # noqa: E402
import rgbd_source as rs  # noqa: E402
import xyz_move as xm  # noqa: E402
from synthetic_scene import MicrowaveScene, default_K, look_at_camera, render  # noqa: E402

DATA = ROOT / "tests" / "data"
K9 = default_K()


def quiet(*_):
    pass


def cfg_for(**container):
    cfg = ft.load_perception_config(overrides={"container": {"grasp_offset": [0.0, 0.0, 0.10], **container}})
    return cfg


def wrist_view(scene, roll=90.0, back=0.30, up=0.20, side=0.01):
    o = np.asarray(scene.origin)
    cam = o - scene.f * back + np.array([0, 0, up]) + scene.l * side
    return look_at_camera(cam, o + scene.f * 0.16 + np.array([0, 0, 0.05]), roll)


def synth_frame(scene, T, noise=1.0, seed=0):
    depth, bgr, _ = render(scene, T, K9, noise_mm=noise, seed=seed)
    return rs.RGBDFrame(bgr, depth / 1000.0, K9, T, source="synthetic")


def interior_mask(scene, frame):
    """What a perfect "inside of open microwave" segmentation returns: pixels whose surface lies in the cavity."""
    t = scene.truth()
    pts, uv, _ = pcm.deproject(frame.depth_m * 1000.0, K9)
    P = pcm.transform_points(frame.base_T_cam, pts)
    pf, pl = P @ scene.f, P @ scene.l
    inside = ((pf > t["front"] + 0.002) & (pf < t["back"] + 0.01) & (pl > t["right"] - 0.01) & (pl < t["left"] + 0.01)
              & (P[:, 2] > t["floor_z"] - 0.01) & (P[:, 2] < t["top"] + 0.01))
    m = np.zeros(frame.depth_m.shape, bool)
    m[uv[inside, 1], uv[inside, 0]] = True
    return m


class FakeSam:
    """Returns a raw-image mask the way SAM 3 would: in the coordinates of the (upright) image it was given."""

    def __init__(self, raw_mask, k, score=0.9):
        self.raw, self.k, self.score, self.seen = raw_mask, k, score, []

    def segment(self, rgb):
        self.seen.append(rgb.shape)
        return np.rot90(self.raw, self.k), self.score


def fake_segmenter(scene, frame, score=0.9, **cfg):
    sam = FakeSam(interior_mask(scene, frame), pcm.upright_k(frame.base_T_cam), score)
    return cs.CavitySegmenter(cs.SegmentationConfig(**cfg), model=sam), sam


def run(scene, Ts, cfg=None, min_looks=None, seed=0):
    cfg = cfg or cfg_for()
    frames = [synth_frame(scene, T, seed=seed + i) for i, T in enumerate(Ts)]
    masks = [fake_segmenter(scene, fr)[0].segment(fr) for fr in frames]
    return ft.run_pipeline(frames, None, cfg, min_looks=min_looks, masks=masks, log=quiet)


# ---------------------------------------------------------------- acquisition / transforms
def test_frame_rejects_misaligned_depth_and_bad_rotation():
    T = np.eye(4)
    with pytest.raises(rs.AcquisitionError, match="aligned"):
        rs.RGBDFrame(np.zeros((480, 640, 3), np.uint8), np.zeros((240, 320)), K9, T)
    T[0, 0] = 2.0
    with pytest.raises(rs.AcquisitionError, match="orthonormal"):
        rs.RGBDFrame(np.zeros((480, 640, 3), np.uint8), np.zeros((480, 640)), K9, T)


def test_recorded_formats_roundtrip(tmp_path):
    import cv2
    scene = MicrowaveScene()
    fr = synth_frame(scene, wrist_view(scene))
    fr.save(tmp_path / "a.npz")                                       # RGBDFrame / placement_workflow.Frame format
    a = rs.load_recorded(tmp_path / "a.npz")
    np.testing.assert_allclose(a.depth_m, fr.depth_m, atol=1e-6)
    np.testing.assert_allclose(a.base_T_cam, fr.base_T_cam)
    # microwave_common.save_frame format: uint16 mm depth, 3x3 K, colour in <stem>_rgb.jpg
    np.savez_compressed(tmp_path / "b.npz", depth_mm=(fr.depth_m * 1000).astype(np.uint16), K=fr.K,
                        base_T_cam=fr.base_T_cam)
    cv2.imwrite(str(tmp_path / "b_rgb.jpg"), fr.bgr)
    b = rs.load_recorded(tmp_path / "b.npz")
    assert np.abs(b.depth_m - fr.depth_m).max() < 0.0011             # mm quantisation
    # teleop capture frame dir
    d = tmp_path / "000001"
    d.mkdir()
    cv2.imwrite(str(d / "frame_rgb.jpg"), fr.bgr)
    cv2.imwrite(str(d / "frame_depth.png"), (fr.depth_m * 1000).astype(np.uint16))
    (d / "frame_detection_inputs.json").write_text(json.dumps({
        "camera_info": {"frame_id": "cam_opt", "K": fr.K.reshape(-1).tolist(), "stamp": {"secs": 5, "nsecs": 0}},
        "transforms": {"x": {"parent": "arm_base_link", "child": "cam_opt", "matrix": fr.base_T_cam.tolist()}}}))
    c = rs.load_recorded(d)
    assert c.frame_id == "cam_opt" and c.stamp == 5.0
    np.testing.assert_allclose(c.base_T_cam, fr.base_T_cam)
    with pytest.raises(rs.AcquisitionError):
        rs.load_recorded(tmp_path / "missing.npz")


def test_deprojection_puts_a_known_point_in_the_base_frame():
    T = look_at_camera([0.3, 0.1, 0.4], [0.8, 0.1, 0.4], 0.0)            # looking along +x, level
    depth = np.zeros((480, 640), np.float32)
    depth[240, 320] = 0.5                                                 # principal point, 0.5 m
    fr = rs.RGBDFrame(np.zeros((480, 640, 3), np.uint8), depth, K9, T)
    cfg = ft.PerceptionConfig().cloud
    cfg.min_points, cfg.outlier_k = 1, 0
    mask = np.zeros((480, 640), bool)
    mask[240, 320] = True
    c = ft.cavity_cloud(fr, mask, cfg)
    np.testing.assert_allclose(c.points[0], [0.8, 0.1, 0.4], atol=1e-6)
    cfg.depth_corr_m = 0.01                                               # along the optical axis
    np.testing.assert_allclose(ft.cavity_cloud(fr, mask, cfg).points[0], [0.81, 0.1, 0.4], atol=1e-6)


def test_cloud_refuses_missing_depth():
    scene = MicrowaveScene()
    fr = synth_frame(scene, wrist_view(scene))
    seg, _ = fake_segmenter(scene, fr)
    cm = seg.segment(fr)
    fr.depth_m[:] = 0.0                                                   # glare / no aligned depth
    with pytest.raises(ft.PerceptionError, match="have depth"):
        ft.cavity_cloud(fr, cm.mask, ft.PerceptionConfig().cloud)


# ---------------------------------------------------------------- segmentation
def test_segmenter_runs_upright_and_returns_raw_pixels():
    scene = MicrowaveScene()
    fr = synth_frame(scene, wrist_view(scene, roll=90.0))
    seg, sam = fake_segmenter(scene, fr, dilate_px=0)
    cm = seg.segment(fr)
    assert cm.k_upright in (1, 3) and sam.seen[0] == (640, 480, 3)       # SAM saw the upright (portrait) image
    assert cm.mask.shape == (480, 640)
    np.testing.assert_array_equal(cm.sam_mask, sam.raw)
    T0 = wrist_view(scene, roll=0.0)                                      # an upright camera: no turn
    fr0 = synth_frame(scene, T0)
    seg0, sam0 = fake_segmenter(scene, fr0)
    assert seg0.segment(fr0).k_upright == 0 and sam0.seen[0] == (480, 640, 3)


def test_segmenter_filters_and_refuses():
    scene = MicrowaveScene()
    fr = synth_frame(scene, wrist_view(scene))
    seg, sam = fake_segmenter(scene, fr, dilate_px=0)
    sam.raw = sam.raw.copy()
    sam.raw[0:5, 0:5] = True                                              # a stray island
    cm = seg.segment(fr)
    assert not cm.mask[0:5, 0:5].any() and any("largest" in n for n in cm.notes)
    sam.score = 0.4
    with pytest.raises(cs.SegmentationError, match="score"):
        seg.segment(fr)
    sam.score, sam.raw = 0.9, np.zeros_like(sam.raw)
    sam.raw[10:20, 10:20] = True
    with pytest.raises(cs.SegmentationError, match="covers"):
        seg.segment(fr)

    class Nothing:
        def segment(self, rgb):
            return None
    with pytest.raises(cs.SegmentationError, match="found no"):
        cs.CavitySegmenter(model=Nothing()).segment(fr)


# ---------------------------------------------------------------- floor + target on known geometry
@pytest.mark.parametrize("turntable", [0.0, 0.012])
def test_floor_and_target_match_the_scene(turntable):
    scene = MicrowaveScene(origin=(0.62, 0.02, 0.12), yaw_deg=8, height=0.24, turntable=turntable)
    t = scene.truth()
    res, obs, cav = run(scene, [wrist_view(scene)], min_looks=1)
    assert abs(res.floor_z - t["floor_z"]) < 0.005 and res.floor_tilt_deg < 1.5
    assert np.degrees(np.arccos(res.insertion_axis @ scene.f)) < 2.0
    assert abs(res.support_z - t["support_z"]) < 0.005
    assert res.support_kind == ("floor" if turntable == 0 else "raised support (turntable?)")
    spec, tc = cfg_for().container, cfg_for().target
    # the three points: contact on the support, centre above it, tool behind the centre along the approach
    np.testing.assert_allclose(res.floor_contact[:2], res.container_center[:2])
    assert res.floor_contact[2] == pytest.approx(res.support_z)
    assert res.container_center[2] - res.support_z == pytest.approx(tc.place_gap_m + spec.height_m / 2, abs=0.004)
    np.testing.assert_allclose(res.container_center - res.tool_center, 0.10 * res.insertion_axis, atol=1e-9)
    # inside the true cavity with the clearances (lateral: centred between the walls)
    cf, cl = res.container_center @ scene.f, res.container_center @ scene.l
    assert t["front"] + tc.front_inside_m + spec.length_m / 2 - 0.02 < cf < t["back"] - tc.back_clearance_m - spec.length_m / 2 + 0.02
    assert abs(cl - (t["left"] + t["right"]) / 2) < 0.02
    assert res.yaw_deg == pytest.approx(np.degrees(np.arctan2(res.insertion_axis[1], res.insertion_axis[0])))
    assert "single look" in " ".join(res.warnings)


def test_three_looks_agree_and_a_bad_calibration_is_refused():
    scene = MicrowaveScene(origin=(0.62, 0.02, 0.12), yaw_deg=8, height=0.24)
    Ts = [wrist_view(scene, side=s) for s in (-0.03, 0.0, 0.03)]
    res, _, _ = run(scene, Ts, min_looks=3)
    assert abs(res.floor_z - scene.truth()["floor_z"]) < 0.005
    assert not any("single look" in w for w in res.warnings)
    # a calibration rolled 20 deg the wrong way: the "floor" is not level -> refused, never a guess
    from scipy.spatial.transform import Rotation as R
    bad = wrist_view(scene)
    fr = synth_frame(scene, bad)
    seg, _ = fake_segmenter(scene, fr)
    cm = seg.segment(fr)
    fr.base_T_cam = bad.copy()
    fr.base_T_cam[:3, :3] = R.from_euler("x", 20, degrees=True).as_matrix() @ bad[:3, :3]
    with pytest.raises(ft.PerceptionError, match="no level upward-facing surface"):
        ft.run_pipeline([fr], None, cfg_for(), min_looks=1, masks=[cm], log=quiet)


def test_camera_independence_fixed_camera_gives_the_same_target():
    """A camera not on the arm (upright, elsewhere): only base_T_cam differs; the target must not."""
    scene = MicrowaveScene(origin=(0.62, 0.02, 0.12), yaw_deg=8, height=0.24)
    wrist, _, _ = run(scene, [wrist_view(scene, roll=90.0)], min_looks=1)
    o = np.asarray(scene.origin)
    fixed = look_at_camera(o - scene.f * 0.45 + scene.l * 0.06 + np.array([0, 0, 0.28]),
                           o + scene.f * 0.15 + np.array([0, 0, 0.03]), 0.0)
    other, _, _ = run(scene, [fixed], min_looks=1)
    assert np.linalg.norm(wrist.container_center - other.container_center) < 0.01
    assert abs(wrist.floor_z - other.floor_z) < 0.003


def test_target_refusals_and_missing_grasp_offset():
    scene = MicrowaveScene(origin=(0.62, 0.02, 0.12), yaw_deg=8, height=0.24)
    with pytest.raises(ft.PerceptionError, match="between the side walls"):
        run(scene, [wrist_view(scene)], cfg=cfg_for(width_m=0.29), min_looks=1)
    with pytest.raises(ft.PerceptionError, match="cavity depth"):
        run(scene, [wrist_view(scene)], cfg=cfg_for(length_m=0.27), min_looks=1)
    with pytest.raises(ft.PerceptionError, match="too tall"):
        run(scene, [wrist_view(scene)], cfg=cfg_for(height_m=0.22), min_looks=1)
    cfg = ft.load_perception_config()                                     # grasp_offset not measured
    res, _, _ = run(scene, [wrist_view(scene)], cfg=cfg, min_looks=1)
    assert res.tool_center is None and any("grasp_offset" in w for w in res.warnings)
    assert "grasp_offset" not in res.part1_config()


def test_something_on_the_floor_is_refused():
    from synthetic_scene import OBB
    scene = MicrowaveScene(origin=(0.62, 0.02, 0.12), yaw_deg=8, height=0.24, turntable=0.0)
    c = np.asarray(scene.origin) + scene.f * 0.15 + np.array([0, 0, 0.04])
    scene.boxes.append(OBB(c, np.column_stack([scene.f, scene.l, [0, 0, 1]]), np.array([0.04, 0.04, 0.04]), "mug"))
    with pytest.raises(ft.PerceptionError, match="something is in the microwave"):
        run(scene, [wrist_view(scene)], min_looks=1)


def test_support_surface_ignores_a_rim_but_takes_a_turntable():
    class Cav:
        f, l = np.array([1.0, 0, 0]), np.array([0, 1.0, 0])

        def floor_z_at(self, xy):
            return 0.10
    rng = np.random.default_rng(0)
    xy = rng.uniform(-0.05, 0.05, (400, 2))
    floor = np.column_stack([xy, 0.10 + rng.normal(0, 0.001, 400)])
    rim = np.column_stack([xy[:30], np.full(30, 0.122)])                  # 7%: a rim / noise
    s = ft.support_surface(Cav(), np.vstack([floor, rim]), 0.0, 0.0, 0.06, 0.06, 0.05)
    assert s["height"] == 0.0 and s["n_above"] == 30
    tt = np.column_stack([xy[:300], 0.115 + rng.normal(0, 0.001, 300)])   # a turntable over most of it
    s = ft.support_surface(Cav(), np.vstack([floor[300:], tt]), 0.0, 0.0, 0.06, 0.06, 0.05)
    assert s["height"] == pytest.approx(0.015, abs=0.002)


# ---------------------------------------------------------------- Part 1 compatibility
def test_part1_config_is_what_place_move_consumes(tmp_path):
    import yaml
    import place_move as pm
    scene = MicrowaveScene(origin=(0.62, 0.02, 0.12), yaw_deg=8, height=0.24)
    res, _, _ = run(scene, [wrist_view(scene)], min_looks=1)
    p = tmp_path / "place_move.yaml"
    p.write_text(yaml.safe_dump(res.part1_config()))
    args = pm.parse_args(["--config", str(p)])
    assert args.target == "container" and args.roll_deg == -90.0
    np.testing.assert_allclose(pm.placement_point(args), res.container_center, atol=1e-4)
    # Part 1's own goal for that input is exactly the tool target computed here
    rot = xm.target_orientation(args.yaw_deg, args.roll_deg)
    tool = xm.tool_goal(args.xyz, "base", "container", rot, np.zeros(3), rot, args.grasp_offset)
    np.testing.assert_allclose(tool, res.tool_center, atol=2e-4)
    assert abs(rot.as_matrix()[2, 2]) < 1e-9                              # approach level (HOLD_LEVEL-compatible)
    np.testing.assert_allclose(rot.as_matrix()[:, 2], res.insertion_axis, atol=1e-3)   # approach = insertion axis


def test_config_rejects_unknown_keys():
    with pytest.raises(ValueError, match="unknown config key"):
        ft.load_perception_config(overrides={"container": {"lenght_m": 0.1}})
    with pytest.raises(ValueError, match="grasp_offset"):
        ft.load_perception_config(overrides={"container": {"grasp_offset": [0.1, 0.2]}})


# ---------------------------------------------------------------- the real rig frame
def test_real_rig_frame_with_its_sam3_mask():
    """rchi-cpu-5 2026-10-20: wrist camera (rolled 90 deg on its mount) looking into the open Comfee; the
    mask is SAM 3's own output on this frame ("inside of open microwave", 0.91)."""
    from perceive_placement import load_mask
    cfg = cfg_for()
    fr = rs.load_recorded(DATA / "microwave_lookin_2026-10-20.npz")
    cm = load_mask(DATA / "microwave_lookin_2026-10-20_sam3mask.npz", fr, cfg.segmentation)
    assert cm.k_upright == 1 and cm.score > 0.9
    res, _, cav = ft.run_pipeline([fr], None, cfg, min_looks=1, masks=[cm], log=quiet)
    assert abs(res.floor_z - 0.051) < 0.01 and res.floor_tilt_deg < 2.0 and res.floor_rms_m < 0.004
    assert 0.25 < cav.depth < 0.31 and 0.28 < cav.width < 0.33 and 0.18 < cav.height < 0.27
    assert res.support_kind == "floor"                                    # the rim points are not a support
    u = res.usable_floor
    cf, cl = res.container_center @ res.insertion_axis, res.container_center @ cav.l
    assert u["front"] < cf < u["back"] and u["right"] < cl < u["left"]
    assert 0.08 < res.container_center[2] < 0.16


def test_cli_on_the_real_frame_writes_everything_and_refuses_with_exit_2(tmp_path, capsys):
    import perceive_placement as pp
    import yaml
    out = tmp_path / "ok"
    rc = pp.main(["--recorded", str(DATA / "microwave_lookin_2026-10-20.npz"),
                  "--mask", str(DATA / "microwave_lookin_2026-10-20_sam3mask.npz"),
                  "--grasp-offset", "0", "0", "0.10", "--out", str(out)])
    text = capsys.readouterr().out
    assert rc == 0 and "FRAME: arm_base_link" in text and "CONTAINER CENTRE" in text
    assert {p.name for p in out.iterdir()} >= {"overlay_1.jpg", "top_down.jpg", "mask_1.npz", "result.json",
                                               "place_move.yaml"}
    r = json.loads((out / "result.json").read_text())
    assert r["ok"] and r["frame"] == "arm_base_link" and len(r["tool_center"]) == 3
    assert yaml.safe_load((out / "place_move.yaml").read_text())["xyz"] == pytest.approx(r["container_center"], abs=1e-4)
    # a mask that is not a cavity: refused, exit 2, the overlay still written
    bad = tmp_path / "bad_mask.npz"
    m = np.zeros((480, 640), bool)
    m[100:110, 100:110] = True
    np.savez_compressed(bad, sam_mask=m, score=0.9)
    rc = pp.main(["--recorded", str(DATA / "microwave_lookin_2026-10-20.npz"), "--mask", str(bad),
                  "--out", str(tmp_path / "bad")])
    assert rc == 2 and "REFUSED" in capsys.readouterr().out
