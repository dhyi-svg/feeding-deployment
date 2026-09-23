"""The approach contact rule and the tool-force press detector (button_press).

The contact rule is what stops the arm on the button instead of pressing air or
pushing through the panel, so these pin its decisions against force sequences of the
shapes measured on hardware 2026-09-21/22 (see the constants in
``button_press/contact.py``). Pure logic: no arm, no ROS.
"""

import numpy as np
import pytest

from feeding_deployment.button_press import contact as K
from feeding_deployment.button_press.contact import ApproachContactMonitor
from feeding_deployment.button_press.press_detector import PressDetector


def _monitor():
    return ApproachContactMonitor(
        contact_n=K.CONTACT_N, jump_n=K.JUMP_N, window=K.CONTACT_WINDOW,
        confirm_rise_n=K.CONFIRM_RISE_N, abort_n=K.FORCE_ABORT_N,
        coarse_abort_n=K.COARSE_ABORT_N)


def test_unarmed_tracks_drift_and_never_calls_contact():
    """Before the contact zone, multi-newton pose-dependent wander is not a press."""
    m = _monitor()
    m.start(0.1)
    # +3.39 N phantom jump from a single 1 cm step (2026-09-21), then more wander.
    for f in [0.3, 3.7, 1.2, 5.9, 2.0]:
        v = m.step(f, armed=False)
        assert v.kind == "continue"
    # The reference follows the bias, so the next step's jump is measured from 2.0.
    assert m.step(2.5, armed=False).jump == pytest.approx(0.5)


def test_unarmed_coarse_collision_aborts():
    m = _monitor()
    m.start(0.1)
    assert m.step(K.COARSE_ABORT_N + 0.5, armed=False).kind == "abort"


def test_abort_above_limit_even_when_armed():
    m = _monitor()
    m.rebaseline(0.1)
    assert m.step(K.FORCE_ABORT_N + 0.1, armed=True).kind == "abort"


def test_sharp_contact_is_candidate_then_confirmed():
    """Real contact 2026-09-22: rose +3.71 then +3.01 N -- keeps loading up."""
    m = _monitor()
    m.rebaseline(0.2)
    v = m.step(0.2 + 3.71, armed=True)
    assert v.kind == "candidate"
    assert m.confirm(v.force, v.force + 3.01) == "contact"


def test_gradual_ramp_becomes_candidate_over_the_window():
    """No single step jumps JUMP_N, but the rise over CONTACT_WINDOW steps does."""
    m = _monitor()
    m.rebaseline(0.12)
    kinds = [m.step(f, armed=True).kind for f in [1.0, 1.9, 2.8]]
    assert kinds == ["continue", "continue", "candidate"]


def test_oscillating_phantom_is_rejected_on_confirm():
    """Worst phantom seen: +6.30 then -2.96 ... -- a candidate that does not keep rising."""
    m = _monitor()
    m.rebaseline(0.1)
    v = m.step(6.4, armed=True)
    assert v.kind == "candidate"
    assert m.confirm(v.force, v.force - 2.96) == "phantom"


def test_confirm_step_over_limit_aborts():
    m = _monitor()
    assert m.confirm(10.0, K.FORCE_ABORT_N + 1.0) == "abort"


def test_rebaseline_restarts_the_window():
    """After a rejected phantom, pre-baseline readings must not count toward a rise."""
    m = _monitor()
    m.rebaseline(0.1)
    m.step(1.5, armed=True)
    m.rebaseline(2.0)  # detector re-zeroed; this is the new rest level
    v = m.step(2.4, armed=True)
    assert v.kind == "continue"
    assert v.window_rise == pytest.approx(0.4)


def test_lateral_move_is_not_counted_as_a_jump():
    m = _monitor()
    m.rebaseline(0.1)
    m.step(0.5, armed=True)
    m.note_rest(2.0)  # bias shifted by a re-servo move
    assert m.step(2.3, armed=True).jump == pytest.approx(0.3)


# -- PressDetector (the /press_detector/pressed signal) -------------------------------------
def _run(det, samples, dt=0.02):
    events = []
    for i, f in enumerate(samples):
        e = det.update(i * dt, np.array(f, dtype=float))
        if e:
            events.append((round(i * dt, 2), e))
    return events


def _baselined(**kw):
    det = PressDetector(threshold=3.0, ema_s=0.0, adapt_s=0.0, **kw)
    det.set_baseline(np.tile([0.5, -0.2, 1.0], (50, 1)))
    return det


def test_press_needs_the_hold_time():
    """Motion spikes (<= 0.12 s) are rejected; a sustained push (>= 0.4 s) fires."""
    det = _baselined(hold_s=0.30)
    spike = [[0.5, -0.2, 1.0]] * 5 + [[0.5, -0.2, 11.0]] * 6 + [[0.5, -0.2, 1.0]] * 5
    assert _run(det, spike) == []
    det = _baselined(hold_s=0.30)
    push = [[0.5, -0.2, 1.0]] * 5 + [[0.5, -0.2, 6.0]] * 25 + [[0.5, -0.2, 1.0]] * 5
    kinds = [e for _, e in _run(det, push)]
    assert kinds == ["press", "release"]
    assert det.peak == pytest.approx(5.0)


def test_release_has_hysteresis():
    det = _baselined(hold_s=0.0, release_frac=0.5)
    # Above threshold -> press; dropping to 2 N (between 1.5 and 3) must NOT release.
    kinds = [e for _, e in _run(det, [[0.5, -0.2, 5.0]] * 3 + [[0.5, -0.2, 3.0]] * 5)]
    assert kinds == ["press"]
    assert det.pressed


def test_press_detector_lookup_never_returns_the_caller(tmp_path, monkeypatch):
    """SIGUSR1 kills a process with no handler, so the driver must never find itself."""
    import os  # pylint: disable=import-outside-toplevel

    from feeding_deployment.button_press import press_detector

    pidfile = tmp_path / "press_detector.pid"
    pidfile.write_text(f"{os.getpid()}\n")
    monkeypatch.setattr(press_detector, "PIDFILE", pidfile)
    assert os.getpid() not in press_detector.find_running_press_detector()
