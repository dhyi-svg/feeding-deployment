"""Shared LLM configuration."""

DEFAULT_CLAUDE_MODEL = "claude-haiku-4-5"

# Preference learning (bundle prediction + LTM updates) runs on Opus, with the
# prediction call at xhigh effort: these run once per user correction / once per
# meal, and Haiku reasons correctly about cross-dimension correlations but
# hedges on committing to the implied values (see docs/preference_learning.md
# and scripts/replay_pref_stresstest.py). Everything else (FLAIR planning,
# transparency, adaptability) stays on DEFAULT_CLAUDE_MODEL.
PREDICTION_CLAUDE_MODEL = "claude-opus-4-8"
PREDICTION_EFFORT = "xhigh"

# Serve the prediction call with fast mode (research preview: same Opus 4.8
# weights at up to 2.5x output tokens/sec, at 2x price -- $10/$50 per MTok vs
# $5/$25). Access is gated (account manager / claude.com/fast-mode waitlist);
# without it the fast request fails and PredictionModel falls back to standard
# speed automatically, so this is safe to leave on before access is granted.
# The meal-end LTM update always runs at standard speed (nobody waits on it).
PREDICTION_FAST_MODE = True
