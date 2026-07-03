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
