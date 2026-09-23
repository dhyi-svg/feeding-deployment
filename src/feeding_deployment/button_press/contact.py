"""Contact decision for the button approach, from at-rest tool-force readings.

Pure numpy-free bookkeeping, split out of ``autonomous_press.Run.approach`` so the
decision rule can be unit-tested and replayed against logged force sequences without
an arm. The motion (the extra confirming step, the re-baseline) stays in the caller;
this class only says what a reading means.

The measurements behind every constant are recorded at the constants below. In short,
after each approach step (arm at rest):

  * above ``abort_n``                          -> abort, always
  * not armed (contact geometrically impossible):
      above ``coarse_abort_n``                 -> abort (a real collision)
      otherwise the reading becomes the new reference -- Kinova's wrench estimate is
      pose-dependent, so the bias drifts with every step and must be tracked, not read
      as a press
  * armed:
      rise over the last ``window`` steps > ``contact_n``, or a single-step jump
      > ``jump_n``                             -> CANDIDATE
  * a candidate is confirmed only if one more fine step adds >= ``confirm_rise_n``
    (a fingertip driven into a rigid panel keeps loading up; bias wander reverses)
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

# Contact = BOTH of: the at-rest |dF| has risen CONTACT_N above its value at the start of
# the approach, AND it rose at least JUMP_N within the last single step. The second test
# is what separates a fingertip meeting a rigid panel (sharp rise inside one 3 mm step)
# from the slow ramps seen on 2026-09-21: Kinova's compensated wrench drifted ~2.6 N for a
# 2 cm move and another ~2.6 N over 9 mm of approach (0.5-0.75 N per step) with the
# fingertip verifiably touching nothing. An absolute threshold alone called that contact.
# Contact detection is only ARMED once contact is geometrically possible: within
# CONTACT_ARM_MARGIN_M of the expected fingertip-to-panel distance (s_panel - tip_dist).
# Outside that zone a force jump cannot be the button. On 2026-09-21 the approach declared
# CONTACT at 1.0 cm travel when the panel was 6.8 cm away -- a +3.39 N phantom jump from a
# single 1 cm step (4.4 deg of joint motion; Kinova's external-wrench ESTIMATE is strongly
# pose-dependent). The arm never touched anything and the "press" pressed air. The earlier
# successful presses never hit this because they started 1.6 cm out and stepped 2 mm at a
# time, which barely moves the joints. This is the same reasoning the far phase already
# uses ("no contact is geometrically possible there").
CONTACT_ARM_MARGIN_M = 0.03
# While unarmed, only a genuine collision should stop us. Phantom jumps observed at 3-5 N,
# a real fingertip-on-panel contact at 2.25-5.2 N, a hand push 20-50 N. 8 N is above the
# phantom band and well below anything that could damage the panel.
COARSE_ABORT_N = 8.0
# Contact is declared only on a MONOTONIC ramp, never on magnitude alone. Measured
# 2026-09-22 with the arm at rest and a fresh baseline, the wrench itself resolves to
# 0.074 +- 0.042 N (max excursion 0.277 N over 30 s, drift 0.003 N) -- it is 30-200x more
# precise than the contact forces we care about. The multi-newton "noise" we kept tripping
# on is not sensor noise at all but a POSE-DEPENDENT BIAS, which is why filtering does not
# help (1 s of averaging only takes std 0.075 -> 0.021 N; there is nothing high-frequency
# to remove). Magnitude cannot separate the two cases:
#     real contact   +7.80 N then +5.97 N   (rising, rising)
#     worst phantom  +6.30 N then -2.96 then +4.01 then -4.74   (oscillating)
# Bias wander reverses; a fingertip driven into a rigid panel never does. So a candidate
# contact must be CONFIRMED by one more fine step that rises again.
# The candidate test is a rise over a WINDOW of steps, not a single-step jump. Contact
# here builds gradually -- on 2026-09-22 the force climbed from the very first 1 mm step
# (0.12 -> 2.33 -> 2.28 -> 3.74 -> 4.75 -> 4.51 -> 4.49 -> 5.55 -> 7.58 -> 8.00 -> 11.11 N)
# so no SINGLE step jumped 3 N until 11 N, and confirming took it to 14.6 and the press to
# 16.4 N (abort). Summing over 3 steps sees the same ramp at 7.6 N instead, while still
# rejecting the phantom wander, which cancels itself over a window rather than accumulating.
CONTACT_WINDOW = 3           # steps to sum the rise over
CONTACT_N = 2.5              # rise over that window to become a candidate
JUMP_N = 3.0                 # kept: a single step this big is also a candidate
CONFIRM_RISE_N = 1.5         # the confirming step must add at least this much again
FORCE_ABORT_N = 15.0         # anything above this is not a button


@dataclass
class StepVerdict:
    """What one at-rest force reading means. ``kind`` is abort / candidate / continue."""

    kind: str
    force: float
    rise: float  # since the approach (or last re-baseline) reference
    jump: float  # since the previous reading
    window_rise: float  # over the last ``window`` steps; 0 when not armed yet
    reason: str = ""


class ApproachContactMonitor:
    """Stateful contact detector driven one approach step at a time."""

    def __init__(
        self,
        *,
        contact_n: float,
        jump_n: float,
        window: int,
        confirm_rise_n: float,
        abort_n: float,
        coarse_abort_n: float,
    ) -> None:
        self.contact_n = contact_n
        self.jump_n = jump_n
        self.confirm_rise_n = confirm_rise_n
        self.abort_n = abort_n
        self.coarse_abort_n = coarse_abort_n
        self.f_ref = 0.0
        self.f_prev = 0.0
        self.hist: deque[float] = deque(maxlen=window + 1)

    def start(self, f_rest: float) -> None:
        """Reference taken at the start of the approach (before contact is armed)."""
        self.f_ref = self.f_prev = f_rest
        self.hist.clear()

    def rebaseline(self, f_rest: float) -> None:
        """Fresh reference after the press detector re-baselined (arming, phantom).

        The window must not straddle a re-baseline, so it restarts from this reading.
        """
        self.f_ref = self.f_prev = f_rest
        self.hist.clear()
        self.hist.append(f_rest)

    def note_rest(self, f_rest: float) -> None:
        """A non-approach move (lateral re-servo) shifted the bias: not a jump."""
        self.f_prev = f_rest

    def step(self, f: float, armed: bool) -> StepVerdict:
        """Classify the at-rest reading taken after one approach step."""
        rise, jump = f - self.f_ref, f - self.f_prev
        if f > self.abort_n:
            return StepVerdict("abort", f, rise, jump, 0.0,
                               f"|dF| {f:.1f} N > {self.abort_n} -- that is not a button")
        if not armed and f > self.coarse_abort_n:
            return StepVerdict("abort", f, rise, jump, 0.0,
                               f"|dF| {f:.1f} N > {self.coarse_abort_n} before contact is "
                               "possible -- hit something unexpected")
        if not armed:
            # Too far out for contact: track the drifting bias.
            self.f_ref = self.f_prev = f
        self.hist.append(f)
        win_rise = f - self.hist[0] if len(self.hist) > 1 else 0.0
        if not armed:
            return StepVerdict("continue", f, rise, jump, win_rise)
        if (win_rise > self.contact_n and len(self.hist) > 1) or jump > self.jump_n:
            return StepVerdict("candidate", f, rise, jump, win_rise)
        self.f_prev = f
        reason = "level rose but no step jump -- treating as drift" if rise > self.contact_n else ""
        return StepVerdict("continue", f, rise, jump, win_rise, reason)

    def confirm(self, f_candidate: float, f_after: float) -> str:
        """Verdict on the extra fine step after a candidate: abort / contact / phantom.

        On ``phantom`` the caller re-baselines the press detector and calls
        :meth:`rebaseline` with the fresh rest reading.
        """
        if f_after > self.abort_n:
            return "abort"
        if f_after - f_candidate >= self.confirm_rise_n:
            return "contact"
        return "phantom"
