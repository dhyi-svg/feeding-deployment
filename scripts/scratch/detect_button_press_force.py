"""READ-ONLY press detector: fires when the tool feels a sustained external force.

Watches the Kinova-reported external tool wrench (``tool_external_wrench_force_*``, the
``ee_force`` key of :class:`KortexReadOnlyFeedback.get_state`) and reports PRESS when it
departs from a rest baseline by more than a threshold for a minimum hold time, and
RELEASE when it drops back under a lower (hysteresis) threshold. Nothing here commands
the arm: it opens the read-only session from ``kortex_readonly.py`` and only ever calls
``RefreshFeedback()``.

Why the tool wrench and not the joint torques: the repo's ``CollisionSensor`` compares
joint torques against a Pinocchio RNEA model and trips on the max per-joint error (15 N.m
default). A button press is a few newtons at the fingertip, which spreads across seven
joints to well under a N.m each -- invisible to that check. The wrench is Kinova's own
gravity/dynamics-compensated estimate at the tool, so a few N shows up directly.

The arm RPC cannot be used for this: ``KinovaArm.get_state()`` reads the wrench and then
drops it from the returned dict (``kinova.py``), and ``ArmInterfaceClient`` only relays
that dict. Add ``ee_force`` there before wiring this into ``press_microwave_button.py``.

Typical use, in three steps:

1. Characterise the signal first (arm parked, nothing running that holds the arm --
   this opens its own Kortex session, same caveat as ``arm_probe_state.py``)::

       $PY -u scripts/scratch/detect_button_press_force.py --log /tmp/press.jsonl

   Baseline for 2 s, then push the (closed) gripper by hand: watch the ``|dF|`` column
   and per-axis deviations. Press the actual button with the fingertip the way the arm
   would, and note the peak the script prints on RELEASE -- that is the number to set
   ``--threshold`` from (aim for ~60% of it, well above the rest noise it also prints).

2. Tune offline against that recording, no arm needed::

       $PY scripts/scratch/detect_button_press_force.py --replay /tmp/press.jsonl --threshold 3

   ``--replay`` also accepts ``state.jsonl`` from ``record_teleop_demo.py``.

3. Use it live while a press is executed by another process (it is a second, read-only
   session -- verify on this rig that the arm grants it alongside ``arm_server.py``;
   ``TELEOP_TESTING.md`` records that this is inferred, not hardware-checked).

4. Show it on the button-detector overlay: add ``--publish`` and the script also puts
   every sample on ROS 2 topics ``/press_detector/force_dev`` (geometry_msgs/Vector3Stamped,
   the baseline-subtracted force ``dF``) and ``/press_detector/pressed`` (std_msgs/Bool).
   ``button_detector_node.py`` subscribes to both and draws a CONTACT banner on its
   ``debug_image``. Needs ``rclpy``, so on rchi-cpu-5 launch with the *prepend* form --
   ``PYTHONPATH=$PWD/src:$PYTHONPATH`` -- never ``PYTHONPATH=src``, which drops ``rclpy``.

Re-baseline without restarting: ``kill -USR1 <pid>`` (arm parked, hands off for 2 s). The
compensated wrench is pose-dependent, so the baseline goes stale after any move; restarting
the script instead opens fresh Kortex sessions, and on 2026-09-21 every such restart killed
``arm_server.py``'s session (INVALID_USER_SESSION_ACCESS) -- the arm seems to evict the
oldest sessions when new ones pile up. For the same reason this opens ONE session (UDP
feedback only; ``GetArmState`` is skipped).

Ctrl-C stops cleanly and prints a summary. Exit code is 0 if at least one press was seen,
1 otherwise -- so ``--once`` (exit on the first press) works as a yes/no probe.

Frame note: Kinova does not say clearly whether the tool wrench is expressed in the base or
the tool frame, so the decision uses ``|dF|`` (frame-independent). ``--axis x|y|z`` restricts
it to one axis once you have watched which one moves for a straight-on press.
"""

from __future__ import annotations

import argparse
import json
import math
import signal
import sys
import time
from collections import deque

import numpy as np

# Defaults chosen from the physics, not measured on this rig yet -- step 1 above.
DEFAULT_THRESHOLD_N = 3.0  # |dF| that counts as contact
DEFAULT_RELEASE_FRAC = 0.5  # release when |dF| < threshold * this (hysteresis)
# Must stay above threshold this long. 0.30 s, not the 0.10 s first guessed: on rchi-cpu-5
# (2026-09-21) teleop/hand motion alone produced 8-15 N spikes lasting 0.02-0.12 s (inertial /
# compensation lag, any direction), while real pushes lasted >= 0.4 s -- so 0.3 s rejected all
# 9 motion spikes in a 4.5 min log and kept all 29 pushes. Direction gating added nothing.
DEFAULT_HOLD_S = 0.30
DEFAULT_BASELINE_S = 2.0
DEFAULT_ADAPT_S = 10.0  # baseline drift time constant while not in contact; 0 = frozen
DEFAULT_RATE_HZ = 50.0
DEFAULT_EMA_S = 0.04  # low-pass on the wrench; ~2 samples at 50 Hz
# Arm counts as moving above this on any joint (same number record_teleop_demo.py uses).
STILL_JOINT_VEL_RAD_S = 0.02


class PressDetector:
    """Threshold-with-hysteresis contact detector on a 3-vector force signal.

    Pure numpy, no hardware, so it can be replayed and unit-tested. Feed it
    ``update(t, force)`` in time order; it returns ``"press"``, ``"release"`` or ``None``.
    """

    def __init__(
        self,
        threshold: float = DEFAULT_THRESHOLD_N,
        release_frac: float = DEFAULT_RELEASE_FRAC,
        hold_s: float = DEFAULT_HOLD_S,
        adapt_s: float = DEFAULT_ADAPT_S,
        ema_s: float = DEFAULT_EMA_S,
        axis: int | None = None,
    ) -> None:
        assert threshold > 0
        assert 0 < release_frac < 1
        self.threshold = threshold
        self.release_threshold = threshold * release_frac
        self.hold_s = hold_s
        self.adapt_s = adapt_s
        self.ema_s = ema_s
        self.axis = axis

        self.baseline: np.ndarray | None = None
        self.baseline_noise = 0.0  # std of |dF| during the baseline window
        self.filtered: np.ndarray | None = None
        self.pressed = False
        self.above_since: float | None = None
        self.press_started_at: float | None = None
        self.peak = 0.0
        self.peak_vec = np.zeros(3)
        self.last_t: float | None = None
        self.deviation = 0.0
        self.deviation_vec = np.zeros(3)

    # -- baseline -----------------------------------------------------------------
    def set_baseline(self, samples: np.ndarray) -> None:
        """``samples``: (N, 3) forces captured with the tool touching nothing."""
        samples = np.asarray(samples, dtype=float)
        assert samples.ndim == 2 and samples.shape[1] == 3 and len(samples) >= 5
        # Median: a stray bump during the baseline window should not shift it.
        self.baseline = np.median(samples, axis=0)
        self.baseline_noise = float(np.std(np.linalg.norm(samples - self.baseline, axis=1)))
        self.filtered = self.baseline.copy()

    def baseline_is_sane(self) -> tuple[bool, str]:
        """Warn if the threshold is inside the rest noise -- it would fire constantly."""
        if self.baseline is None:
            return False, "no baseline"
        if self.threshold < 3.0 * self.baseline_noise:
            return False, (
                f"threshold {self.threshold:.2f} N is < 3x rest noise "
                f"({self.baseline_noise:.2f} N std); raise --threshold"
            )
        return True, ""

    # -- per-sample -----------------------------------------------------------------
    def _measure(self, vec: np.ndarray) -> float:
        if self.axis is None:
            return float(np.linalg.norm(vec))
        return float(abs(vec[self.axis]))

    def update(self, t: float, force: np.ndarray) -> str | None:
        assert self.baseline is not None, "call set_baseline() first"
        force = np.asarray(force, dtype=float)
        dt = 0.0 if self.last_t is None else max(0.0, t - self.last_t)
        self.last_t = t

        # Low-pass the raw wrench. alpha from dt so the cutoff does not depend on the
        # (unsteady) RefreshFeedback rate.
        alpha = 1.0 if self.ema_s <= 0 or dt <= 0 else 1.0 - math.exp(-dt / self.ema_s)
        self.filtered = self.filtered + alpha * (force - self.filtered)

        self.deviation_vec = self.filtered - self.baseline
        self.deviation = self._measure(self.deviation_vec)

        event = None
        if not self.pressed:
            if self.deviation > self.threshold:
                if self.above_since is None:
                    self.above_since = t
                if t - self.above_since >= self.hold_s:
                    self.pressed = True
                    self.press_started_at = self.above_since
                    self.peak = self.deviation
                    self.peak_vec = self.deviation_vec.copy()
                    event = "press"
            else:
                self.above_since = None
                # Track slow bias drift (pose-dependent compensation error) only while
                # the tool is clearly free, so a slow press cannot be absorbed into it.
                if self.adapt_s > 0 and dt > 0 and self.deviation < self.release_threshold:
                    beta = 1.0 - math.exp(-dt / self.adapt_s)
                    self.baseline = self.baseline + beta * (self.filtered - self.baseline)
        else:
            if self.deviation > self.peak:
                self.peak = self.deviation
                self.peak_vec = self.deviation_vec.copy()
            if self.deviation < self.release_threshold:
                self.pressed = False
                self.above_since = None
                event = "release"
        return event


# -- sample sources ---------------------------------------------------------------------
def _live_samples(ip: str, rate_hz: float):
    """Yield ``(t, state)`` from a read-only Kortex session at ~rate_hz."""
    from feeding_deployment.control.robot_controller.kortex_readonly import (  # noqa: PLC0415
        KortexReadOnlyFeedback,
    )

    period = 1.0 / rate_hz
    # with_base_client=False: one Kortex session (UDP feedback), not two. See module doc.
    with KortexReadOnlyFeedback(ip=ip, with_base_client=False) as arm:
        print("read-only feedback session open (single session; arm state not queried)")
        next_t = time.monotonic()
        while True:
            state = arm.get_state()
            yield time.monotonic(), state
            next_t += period
            sleep_for = next_t - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:  # fell behind; do not try to catch up in a burst
                next_t = time.monotonic()


def _replay_samples(path: str):
    """Yield ``(t, state)`` from this script's ``--log`` or record_teleop_demo's state.jsonl."""
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            rec = json.loads(line)
            if rec.get("ee_force") is None:
                continue
            t = rec.get("t", rec.get("t_mono", rec.get("time")))
            if t is None:
                raise ValueError(f"{path}: record has no timestamp key (t / t_mono / time)")
            state = {
                "ee_force": np.asarray(rec["ee_force"], dtype=float),
                "velocity": np.asarray(rec.get("velocity", [0.0] * 7), dtype=float),
                "effort": np.asarray(rec.get("effort", [0.0] * 7), dtype=float),
            }
            yield float(t), state


# -- optional ROS 2 output -----------------------------------------------------------------
class _RosPublisher:
    """Publish dF + pressed flag so the button-detector overlay can show contact.

    Publish-only: no spin, no subscriptions. Both topics are sent every sample so a
    consumer can treat silence as "detector not running" rather than "not pressed".
    """

    def __init__(self, namespace: str) -> None:
        import rclpy  # noqa: PLC0415
        from geometry_msgs.msg import Vector3Stamped  # noqa: PLC0415
        from std_msgs.msg import Bool  # noqa: PLC0415

        self._rclpy = rclpy
        self._Vector3Stamped = Vector3Stamped
        self._Bool = Bool
        rclpy.init()
        self.node = rclpy.create_node("press_detector")
        ns = namespace.rstrip("/")
        self.pub_force = self.node.create_publisher(Vector3Stamped, f"{ns}/force_dev", 10)
        self.pub_pressed = self.node.create_publisher(Bool, f"{ns}/pressed", 10)
        print(f"publishing {ns}/force_dev and {ns}/pressed")

    def publish(self, dev_vec: np.ndarray, pressed: bool) -> None:
        # rclpy installs its own SIGINT/SIGTERM handler that tears the context down
        # underneath us; surface that as the same clean stop Ctrl-C takes.
        if not self._rclpy.ok():
            raise KeyboardInterrupt
        msg = self._Vector3Stamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "tool_wrench"  # Kinova does not document base vs tool; see module doc
        msg.vector.x, msg.vector.y, msg.vector.z = (float(v) for v in dev_vec)
        self.pub_force.publish(msg)
        b = self._Bool()
        b.data = bool(pressed)
        self.pub_pressed.publish(b)

    def close(self) -> None:
        self.node.destroy_node()
        if self._rclpy.ok():
            self._rclpy.shutdown()


# -- main -------------------------------------------------------------------------------
def _fmt_vec(v: np.ndarray) -> str:
    return "[" + " ".join(f"{x:+6.2f}" for x in v) + "]"


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--ip", default="192.168.1.10")
    p.add_argument("--replay", metavar="JSONL", help="run offline over a recorded log instead of the arm")
    p.add_argument("--log", metavar="JSONL", help="append every sample here (for --replay tuning)")
    p.add_argument("--threshold", type=float, default=DEFAULT_THRESHOLD_N, help="|dF| in N (default %(default)s)")
    p.add_argument("--release-frac", type=float, default=DEFAULT_RELEASE_FRAC)
    p.add_argument("--hold-s", type=float, default=DEFAULT_HOLD_S)
    p.add_argument("--baseline-s", type=float, default=DEFAULT_BASELINE_S)
    p.add_argument("--adapt-s", type=float, default=DEFAULT_ADAPT_S, help="0 freezes the baseline")
    p.add_argument("--ema-s", type=float, default=DEFAULT_EMA_S)
    p.add_argument("--rate-hz", type=float, default=DEFAULT_RATE_HZ)
    p.add_argument("--axis", choices=["x", "y", "z"], help="decide on one axis instead of |dF|")
    p.add_argument("--once", action="store_true", help="exit 0 on the first press")
    p.add_argument("--print-hz", type=float, default=5.0, help="status line rate; 0 = events only")
    p.add_argument("--publish", action="store_true", help="also publish dF + pressed on ROS 2 topics")
    p.add_argument("--topic-ns", default="/press_detector", help="namespace for --publish topics")
    args = p.parse_args()

    axis = None if args.axis is None else "xyz".index(args.axis)
    det = PressDetector(
        threshold=args.threshold,
        release_frac=args.release_frac,
        hold_s=args.hold_s,
        adapt_s=args.adapt_s,
        ema_s=args.ema_s,
        axis=axis,
    )

    rebaseline = {"asked": False}
    signal.signal(signal.SIGUSR1, lambda *_: rebaseline.__setitem__("asked", True))
    samples = _replay_samples(args.replay) if args.replay else _live_samples(args.ip, args.rate_hz)
    log_f = open(args.log, "a") if args.log else None  # noqa: SIM115 -- closed in finally
    ros = _RosPublisher(args.topic_ns) if args.publish else None

    # Joint-effort deviation from its rest value is shown alongside as a second opinion:
    # at (near) rest it needs no dynamics model, and a real contact moves both signals.
    effort_baseline: np.ndarray | None = None
    baseline_buf: list[np.ndarray] = []
    effort_buf: list[np.ndarray] = []
    baseline_moving = 0
    t0: float | None = None
    presses: list[dict] = []
    last_print = -math.inf
    print_period = math.inf if args.print_hz <= 0 else 1.0 / args.print_hz
    rate_win: deque[float] = deque(maxlen=50)

    print(
        f"threshold {args.threshold:.2f} N, release < {det.release_threshold:.2f} N, "
        f"hold {args.hold_s * 1e3:.0f} ms, baseline {args.baseline_s:.1f} s"
        + (f", axis {args.axis}" if args.axis else ", |dF|")
    )
    try:
        for t, state in samples:
            force = np.asarray(state["ee_force"], dtype=float)
            vel = np.asarray(state.get("velocity", []), dtype=float)
            effort = np.asarray(state.get("effort", []), dtype=float)
            moving = bool(len(vel)) and float(np.max(np.abs(vel))) > STILL_JOINT_VEL_RAD_S
            if t0 is None:
                t0 = t
            rate_win.append(t)

            if log_f is not None:
                log_f.write(
                    json.dumps(
                        {
                            "t": t,
                            "ee_force": force.tolist(),
                            "ee_torque": np.asarray(state.get("ee_torque", []), dtype=float).tolist(),
                            "effort": effort.tolist(),
                            "velocity": vel.tolist(),
                            "gripper_pos": state.get("gripper_pos"),
                        }
                    )
                    + "\n"
                )

            if rebaseline["asked"]:
                rebaseline["asked"] = False
                det.baseline = None
                det.filtered = None
                det.pressed = False
                det.above_since = None
                baseline_buf, effort_buf, baseline_moving = [], [], 0
                effort_baseline = None
                t0 = t
                print(f"\n[{t:.1f}] SIGUSR1: re-baselining for {args.baseline_s:.1f} s -- keep the arm still")

            # -- baseline window ----------------------------------------------------
            if det.baseline is None:
                baseline_buf.append(force)
                if len(effort):
                    effort_buf.append(effort)
                baseline_moving += int(moving)
                if t - t0 >= args.baseline_s:
                    if len(baseline_buf) < 5:
                        print("baseline: too few samples -- is RefreshFeedback stalling?", file=sys.stderr)
                        return 1
                    det.set_baseline(np.array(baseline_buf))
                    if effort_buf:
                        effort_baseline = np.median(np.array(effort_buf), axis=0)
                    print(
                        f"baseline F {_fmt_vec(det.baseline)} N  rest noise {det.baseline_noise:.3f} N std"
                        f"  ({len(baseline_buf)} samples"
                        + (f", ARM WAS MOVING in {baseline_moving} of them" if baseline_moving else "")
                        + ")"
                    )
                    ok, why = det.baseline_is_sane()
                    if not ok:
                        print(f"WARNING: {why}")
                    if baseline_moving:
                        print("WARNING: baseline taken while moving -- park the arm and rerun")
                continue

            # -- detection ---------------------------------------------------------
            event = det.update(t, force)
            if ros is not None:
                ros.publish(det.deviation_vec, det.pressed)
            effort_dev = (
                float(np.max(np.abs(effort - effort_baseline)))
                if effort_baseline is not None and len(effort) == len(effort_baseline)
                else float("nan")
            )

            if event == "press":
                print(
                    f"\n[{t - t0:8.3f}s] PRESS   |dF| {det.deviation:.2f} N  dF {_fmt_vec(det.deviation_vec)}"
                    f"  max joint dTau {effort_dev:.2f} N.m" + ("  (arm moving)" if moving else "")
                )
                presses.append({"t_press": t - t0, "t_release": None, "peak": None})
                if args.once:
                    return 0
            elif event == "release":
                dur = t - t0 - presses[-1]["t_press"] if presses else float("nan")
                presses[-1].update(t_release=t - t0, peak=det.peak, peak_vec=det.peak_vec.tolist())
                print(
                    f"[{t - t0:8.3f}s] RELEASE after {dur:.2f}s  peak |dF| {det.peak:.2f} N"
                    f"  dF at peak {_fmt_vec(det.peak_vec)}\n"
                )

            if print_period != math.inf and t - last_print >= print_period:
                last_print = t
                hz = (len(rate_win) - 1) / (rate_win[-1] - rate_win[0]) if len(rate_win) > 1 and rate_win[-1] > rate_win[0] else 0.0
                bar = "#" * min(40, int(40 * det.deviation / (2 * det.threshold)))
                sys.stdout.write(
                    f"\r{t - t0:8.2f}s {hz:5.1f}Hz {'PRESSED' if det.pressed else '       '} "
                    f"|dF| {det.deviation:5.2f} N dF {_fmt_vec(det.deviation_vec)} "
                    f"dTau {effort_dev:5.2f} {'mov' if moving else '   '} |{bar:<40}|"
                )
                sys.stdout.flush()
    except KeyboardInterrupt:
        print()
    finally:
        if log_f is not None:
            log_f.close()
        if ros is not None:
            ros.close()

    print(f"\n{len(presses)} press(es) seen")
    for i, pr in enumerate(presses, 1):
        rel = f"{pr['t_release']:.3f}s" if pr["t_release"] is not None else "(still pressed at exit)"
        pk = f"{pr['peak']:.2f} N" if pr["peak"] is not None else "-"
        print(f"  {i}: press {pr['t_press']:.3f}s  release {rel}  peak {pk}")
    if det.baseline is not None:
        print(f"rest noise was {det.baseline_noise:.3f} N std; final baseline {_fmt_vec(det.baseline)} N")
    return 0 if presses else 1


if __name__ == "__main__":
    sys.exit(main())
