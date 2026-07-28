# Preference-learning day report — LLM prompt

You are writing an interpretive report for ONE day (one meal) of a robot-assisted
feeding preference-learning system. All the facts have already been computed by
`analyze_day.py`. **Your input is `metrics.json`** (and the four PNGs beside it).
**Do not recompute or invent any number** — every figure you cite must come from
`metrics.json`. Your job is judgment and language, not arithmetic.

## Input

Read `<analysis_dir>/metrics.json`. Its structure:

- `meta` — meal context, model, `prior_memory_present` (false = user's first day),
  `parse_failures`, distance tolerances.
- `ground_truth` — final accepted value per dim; `unresolved_dims` — dims never
  pinned this meal (exclude from accuracy; mention if notable, e.g. a location
  never visited).
- `trajectory[]` — per LLM call: `cat_accuracy`/`cat_denom`, `remaining_corrections`
  (open dims still != GT — the quantity the system minimizes), `event`
  (INIT vs correction), `latent_inference`, confirmed/corrected field lists.
- `transitions[]` — per correction: `trigger_fields`/`trigger_kinds`,
  `direct` (the corrected dim: old→new), `correlated` (other OPEN dims the model
  changed, each classified `POSITIVE` wrong→correct / `NEGATIVE` correct→wrong /
  `LATERAL` wrong→wrong vs final GT), `acc_delta_direct`, `acc_delta_correlated`,
  `self_check_ok`.
- `ledger` — total positive/negative/lateral correlated changes.
- `findings.self_inflicted` — dims predicted CORRECTLY at init, drifted wrong via a
  correlated change, then required their own correction (pure model-induced work).
- `findings.non_bearing_drift` — NEGATIVE correlated drift triggered by a COLOR or
  NAV correction, which shares no latent factor with categoricals.
- `findings.re_corrections` — dims the user confirmed at one value, then corrected
  to another (premature confirmation).
- `findings.volatility` — per dim, `n_changes` and `flip_flopped`.
- `continuous_drift` — color/nav distance-to-GT per step.
- `hedge_candidates` — explanations that hedge ("default", "seed", "not directly
  tied", ...) on a dim that nonetheless drifted. **You adjudicate these**: quote
  the explanation and say whether the model's stated reasoning contradicts its own
  output.
- `explanations_by_file` — the model's per-dim reasons, for quoting.

## Output

Write `<analysis_dir>/report.md`. Reference the plots by filename
(`g1_accuracy.png`, `g2_dim_heatmap.png`, `g3_ledger.png`, `g4_color_nav.png`).

Sections, in this order:

1. **Header** — user, date, meal context; and whether this is the first day
   (`prior_memory_present`). If there were `parse_failures` or `unresolved_dims`,
   say so up front.
2. **Summary** — initial vs final accuracy, total corrections split by kind
   (categorical / color / nav / text), # re-corrections, # self-inflicted,
   # non-bearing drifts, most volatile dims. One line on the net effect of the
   correlation mechanism (from `ledger` and the acc deltas): did propagating
   corrections help or hurt on balance, and by how much?
3. **Trajectory** — walk the accuracy / remaining-corrections curve (ref g1),
   calling out any dip and its cause.
4. **Per-correction table** — step | trigger (field: old→new) | stage | correlated
   changes (+/−/lateral) | acc before→after. (Straight from `transitions`.)
5. **Findings**, most important first. Prioritize, in this order when present:
   self-inflicted corrections, non-bearing (color/nav-triggered) drift, premature
   confirmations / re-corrections, volatile ordinal dims (timing/countdown/speed).
   For each, quote the model's own `latent_inference` or `explanation` where it
   contradicts its output (use `hedge_candidates`).
6. **Recommendations** — tie each to a concrete lever, only where THIS day's data
   supports it: (a) tighten ordinal/timing tier definitions *by latent factor*
   (do not lump microwave_time — a food-temperature dim — with the trust/distraction
   countdown timers); (b) gate categorical re-prediction on color/nav corrections;
   (c) feed ordered corrections + the model's prior prediction so open dims stay
   stable unless the latest correction bears on them.

## Guardrails

- Every number traces to `metrics.json`. If something isn't in the data, don't claim it.
- Be honest about limits: this is a single day, and some drift is LLM sampling noise,
  not signal — say which findings are robust (e.g. a self-inflicted correction is a
  hard fact) vs suggestive (a one-off lateral flip).
- If `transitions[].self_check_ok` is false anywhere, flag it as a data-quality issue
  rather than reasoning over it.
- Keep it tight and skimmable: tables and short paragraphs, no filler.
