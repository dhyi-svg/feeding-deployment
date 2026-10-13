"""press_button.check_fingertip: the stored pre-press offset must keep the fingertip out of the panel."""
import numpy as np
import pytest

pb = pytest.importorskip("rammp_button_press.press_button")   # needs rclpy + rammp interfaces
from rammp_button_press import Abort  # noqa: E402
from rammp_button_press.panel_frame import panel_frame  # noqa: E402

ORIGIN, R = panel_frame(np.array([0.6, 0.05, 0.25]), np.array([-1.0, 0.0, 0.0]))


RCHI_OFFSET = np.array([-0.0049, -0.0114, -0.0131])
RCHI_QUAT = np.array([-0.04947, 0.99522, -0.08381, -0.00846])


def test_rchi_offset_is_refused_here(monkeypatch):
    """The rchi-cpu-5 constants put the fingertip ~7 cm into the plane with this URDF's fingertip."""
    monkeypatch.setattr(pb, "PREPRESS_EE_OFFSET_M", RCHI_OFFSET)
    monkeypatch.setattr(pb, "PREPRESS_EE_QUAT_PANEL", RCHI_QUAT)
    p, Rt = pb.prepress_pose(ORIGIN, R)
    assert pb.fingertip_out_of_panel(p, Rt, ORIGIN, R) < -0.06
    with pytest.raises(Abort, match="INTO the panel"):
        pb.check_fingertip(p, Rt, ORIGIN, R, "pre-press spot")


def test_fingertip_2cm_out_passes():
    Rt = pb.prepress_pose(ORIGIN, R)[1]
    tip = ORIGIN + 0.02 * R[:, 2]
    p = tip - Rt[:, 2] * pb.FINGERTIP_PAST_TOOL_M
    assert pb.fingertip_out_of_panel(p, Rt, ORIGIN, R) == pytest.approx(0.02)
    pb.check_fingertip(p, Rt, ORIGIN, R, "pre-press spot")


def test_stored_offset_passes():
    """The offset measured on this rig (2026-10-12): fingertip ~3 cm out of the detected plane."""
    p, Rt = pb.prepress_pose(ORIGIN, R)
    assert pb.fingertip_out_of_panel(p, Rt, ORIGIN, R) == pytest.approx(0.030, abs=0.003)
    pb.check_fingertip(p, Rt, ORIGIN, R, "pre-press spot")
