"""RammpArm's trajectory timing (pure functions; needs rclpy + rammp interfaces importable)."""
import numpy as np
import pytest

rammp_arm = pytest.importorskip("rammp_button_press.rammp_arm")


def test_unwrap_goes_the_short_way():
    q0 = np.zeros(7)
    q0[0] = 3.1
    k = np.zeros(7)
    k[0] = -3.1                                     # J1 continuous: +0.083 rad, not -6.2
    out = rammp_arm.unwrap_from(q0, [k])[0]
    assert out[0] == pytest.approx(3.1 + (2 * np.pi - 6.2))


def test_min_jerk_endpoints_and_rest():
    path = np.array([np.zeros(7), np.full(7, 0.1)])
    t, q, qd = rammp_arm.min_jerk_samples(path, np.array([0.0, 1.0]), 2.0)
    assert np.allclose(q[0], 0) and np.allclose(q[-1], 0.1)
    assert np.allclose(qd[0], 0) and np.allclose(qd[-1], 0)
    assert np.all(np.diff(t) <= rammp_arm.DT + 1e-9)


@pytest.mark.parametrize("dist", [0.01, 0.3])
def test_leg_respects_speed_caps(dist):
    """Peak tool speed <= CART_SPEED and peak joint speed <= JOINT_SPEED (linear joint/Cartesian map)."""
    R0 = np.eye(3)
    n = max(1, int(round(dist / 0.01)))
    poses = [(np.array([0.5 + dist * (i + 1) / n, 0.0, 0.3]), R0) for i in range(n)]
    knots = [np.full(7, 0.002 * (i + 1)) for i in range(n)]     # 0.2 deg per cm
    path, s, dur = rammp_arm.leg_timing(np.zeros(7), knots, poses, np.array([0.5, 0.0, 0.3]), R0)
    t, q, qd = rammp_arm.min_jerk_samples(path, s, dur)
    s_t = np.interp(q[:, 0], path[:, 0], s)                     # arc length along the leg vs time
    assert np.max(np.gradient(s_t, t)) <= rammp_arm.CART_SPEED * 1.02
    assert np.max(np.abs(qd)) <= rammp_arm.JOINT_SPEED * 1.02
    assert dur >= rammp_arm.MIN_DURATION_S


def test_impedance_profile_shape():
    prof = rammp_arm.impedance_profile(0.01, hold_s=0.2)
    t = np.array([p[0] for p in prof])
    d = np.array([p[1] for p in prof])
    phases = [p[2] for p in prof]
    assert np.all(np.diff(t) > 0)
    assert np.allclose(np.diff(t), 1.0 / rammp_arm.IMP_RATE_HZ)          # one setpoint every 10 ms
    assert 1.0 / rammp_arm.IMP_RATE_HZ < rammp_arm.IMP_TIMEOUT_S           # always inside the session timeout
    assert phases[0] == "settle" and phases[-1] == "end"
    assert d.max() == pytest.approx(0.01) and d.min() == 0.0
    assert np.all(d[[p == "settle" for p in phases]] == 0.0)
    assert d[-1] == 0.0                                                   # back at the spot before closing
    assert np.max(np.abs(np.diff(d))) < 0.0005                            # no jumps: < 0.5 mm per 10 ms


def test_impedance_profile_hold_test_never_moves():
    prof = rammp_arm.impedance_profile(0.0)
    assert {p[2] for p in prof} == {"settle", "end"}
    assert all(p[1] == 0.0 for p in prof)
