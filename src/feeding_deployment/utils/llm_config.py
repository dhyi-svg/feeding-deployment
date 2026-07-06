"""Shared LLM configuration."""

DEFAULT_CLAUDE_MODEL = "claude-haiku-4-5"

# Preference learning (bundle prediction + LTM updates) runs on Opus: these run
# once per user correction / once per meal, and Haiku reasons correctly about
# cross-dimension correlations but hedges on committing to the implied values
# (see docs/preference_learning.md and scripts/replay_pref_stresstest.py).
# Everything else (FLAIR planning, transparency, adaptability) stays on
# DEFAULT_CLAUDE_MODEL.
#
# Effort "medium": a 2026-07-03 sweep of low/medium/high/xhigh over the recorded
# 3-day stress test (scratchpad effort_sweep; see docs §2.5) found prediction
# quality FLAT across levels -- corrections/day 12/11/12/13, identical day-3
# convergence, full explanation coverage, equivalent color propagation by the
# next pickup -- while mean call latency scaled 23s/30s/38s/68s. Medium halves
# every user-visible wait vs the old xhigh at no measured quality cost. If a
# future scenario shows under-thinking (propagation misses, shallow latent
# inference), raise to "high" before "xhigh".
PREDICTION_CLAUDE_MODEL = "claude-opus-4-8"
PREDICTION_EFFORT = "medium"

# Serve the prediction call with fast mode (research preview: same Opus 4.8
# weights at up to 2.5x output tokens/sec, at 2x price -- $10/$50 per MTok vs
# $5/$25). Access is gated (account manager / claude.com/fast-mode waitlist);
# without it the fast request fails and PredictionModel falls back to standard
# speed automatically, so this is safe to leave on before access is granted.
# The meal-end LTM update always runs at standard speed (nobody waits on it).
PREDICTION_FAST_MODE = True
