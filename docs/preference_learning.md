# Preference Learning Pipeline

This document explains the personalization ("preference learning") pipeline of the feeding
deployment: the problem it solves, the method, how it is implemented file-by-file, how the
perception stack grounds it, and the corner cases / bugs we know about.

Everything below was verified against the working tree on 2026-07-07 (commit `e582a9b`
plus uncommitted changes). Code references are `path:line`.

---

## 1. TL;DR

At the start of each meal (one meal = one "deployment day"), the system predicts the
user's **entire preference bundle** — 27 dimensions covering robot speed, transfer style,
interaction cues, confirmation-page modes, bite ordering, microwave time, plate-handle
detection colors, and navigation parking offsets — with a memory-augmented LLM. The prediction is *actuated
immediately* (written into the per-user behavior-tree YAMLs and planner state), then
**corrected just-in-time**: a few dimensions are shown on the iPad at natural pause points,
colors are corrected with a live camera color picker during plate pickups, and parking
offsets are corrected by physically teleoperating the base after arrival. **Every
correction triggers a fresh re-prediction of all still-open dimensions**, conditioned on
the corrections made so far — so one correction can move many open dims at once, and the
LLM is called once at meal start plus once per correction. Whatever the user ends up
accepting is the day's ground truth; at meal end it is folded **exactly once** into two
cross-day memories (an LLM long-term profile and an embedding-based episodic store) that
condition the next day's prediction. The learning objective is to need fewer and fewer
corrections over days.

---

## 2. Problem formulation

### 2.1 Setting

- A single user `u` with a fixed natural-language **physical capability description**
  `φ_u` (from `--physical_profile_file`, else `DEFAULT_PHYSICAL_PROFILE`,
  `integration/preference_session.py:104`). Five reference profiles live in
  `preference_learning/config/physical_capabilities.py` and are used by the offline
  synthetic pipeline; at deployment the profile is free text.
- A sequence of meals ("days") `t = 1, 2, …` (deployments target ~30). Days must be run
  strictly sequentially with no gaps (`PredictionModel.validate_sequential_day`,
  `methods/prediction_model.py:275`).
- An **observable context** per meal
  `c_t = (meal, setting, time_of_day)` drawn from fixed vocabularies
  (`config/mealtime_context.py`): an 11-meal catalog (each meal lists its solid items,
  sauces, storage condition, intended serving temperature), 10 dining settings
  (personal / social / TV / laptop × left/front/right), and 3 times of day. Context is
  validated against these vocabularies (`integration/preference_context.py`).
- A **latent, context-dependent preference function** `y*_u : C → Y`. The user's transient
  affective state (Neutral / Hurried / Fatigued / … , `config/affective_state.py`) also
  influences preferences but is **not observed** at deployment — it exists only in the
  synthetic data generator. The deployed system must infer its effects indirectly from the
  user's corrections during the meal (the bundle-prediction prompt says exactly this,
  `methods/prompts/bundle_prediction.txt:34`).

### 2.2 The decision space: the preference bundle

`Y = ∏_d Y_d` over `D = 27` dimensions, declared centrally in
`preference_learning/config/preference_bundle.py` (`PREFERENCE_BUNDLE`, a list of
`PreferenceDim(field, label, options, description, kind)`):

| kind | dims | value space | corrected via |
|---|---|---|---|
| `categorical` (19) | `microwave_time`, `robot_speed`, `skewering_axis`, `confirm_feeding_pickup`, `confirm_navigation_arrival`, `confirm_manipulation`, `transfer_mode`, `outside_mouth_distance`, `convey_robot_ready_for_initiating_transfer`, `detect_user_ready_for_initiating_transfer_{feeding,drinking,wiping}`, `convey_robot_ready_for_completing_transfer`, `detect_user_completed_transfer_{feeding,drinking,wiping}`, `retract_between_bites`, `bite_dipping_preference`, `wait_before_autocontinue_seconds` | finite option list | iPad "ask" pages + settings overlay |
| `text` (1) | `bite_ordering` | free natural-language sentence | iPad ask page, "Other…" editor (typed or speech) |
| `color` (3) | `plate_color_{fridge,microwave,table}` | HSV + tolerance `{h∈[0,179], s,v∈[0,255], range∈[0,1]}` | live camera color picker during the pickup |
| `nav_offset` (4) | `nav_offset_{table,microwave,sink,fridge}` | SE(2) `{dx,dy∈[−0.5,0.5] m, dyaw∈[−45°,45°]}` in the stored goal's local frame | physical teleop adjustment after arrival |

Two structural facts the method exploits:

- The three color dims are the **same physical plate handle** seen under three lightings —
  a correction to one is strong evidence for the others (stated to the LLM,
  `bundle_prediction.txt:43`).
- The four nav offsets are **independent** per location — a correction at one is only weak
  evidence for the others.

**Confirmation-page modes.** The three `confirm_*` dims share one option vocabulary —
`"no"` (page skipped), `"yes (with auto-continue countdown)"` (page counts down for the global wait pref,
then proceeds), `"yes (without any auto-continue)"` (page blocks until answered) — and gate the
confirmation surfaces per skill family: `confirm_feeding_pickup` the bite/drink/wipe
pickup-verification pages, `confirm_navigation_arrival` the post-arrival position
check/adjust page, `confirm_manipulation` the detection-confirmation pages (plate
handle at pickups, fridge/microwave door handles at opening, microwave button,
sink/table placement spots) plus the plate-release confirms. The bundle stays flat: their
relatedness (a shared supervision/trust latent — users relax confirmations together as
trust grows) is expressed through the dim descriptions and the correlated-preferences
prompt guidance, exactly like the color dims. Two learning side effects are deliberate:
`confirm_navigation_arrival = "no"` freezes nav-offset learning (the adjust page is the
teaching channel) and `confirm_manipulation = "no"` freezes color learning (the
Correct-Color picker lives on the detection page); re-enabling from the settings overlay
resumes both.

### 2.3 Interaction protocol (what the learner observes)

At meal `t`:

1. **Predict**: the system outputs a full bundle `ŷ_t ∈ Y`, immediately actuated.
2. **Staged reveal**: dimensions are surfaced for confirmation at the moment they become
   behaviorally relevant (see §4). At each reveal the user either **confirms**
   (`y_{t,d} = ŷ_{t,d}` — including by letting an autocontinue countdown expire) or
   **corrects** (`y_{t,d} ≠ ŷ_{t,d}`).
3. **Conditional reprediction**: after every correction, all still-open (not yet finalized)
   dims are repredicted *conditioned on the corrections so far* — corrections are treated
   as evidence about the current meal (including the unobserved affective state), so one
   correction can move many open dims at once.
4. **Ground truth**: the finalized bundle `y_t` (every dim either explicitly
   confirmed/corrected or implicitly confirmed at meal end) is the day's label.

The learning signal distinguishes `corrected` (dims the user actively changed) from merely
confirmed dims, and the *number of corrections* is the cost we want to drive down.

### 2.4 Objective

There is no parametric training. The system performs **online, in-context preference
learning**: choose the predictor so that, as days accumulate,

- `m*_t` — the number of user corrections needed at meal `t` — decreases toward 0, and
- `acc@m0` — the fraction of dims predicted correctly *before any correction* — increases,

under the hard constraint that predictions never violate the user's physical capabilities
or the meal's structure. These are exactly the metrics the offline harness computes
(`methods/evaluate_prediction_model.py`: `acc_m0`/`acc_m1` per day, mean `m*`,
per-dimension m0 accuracy, zero-correction meal counts overall and for the final week
`day ≥ 24`).

### 2.5 Method: memory-augmented LLM prediction

The predictor (`PredictionModel.predict_bundle`, `methods/prediction_model.py:424`) is one
Claude call (`claude-opus-4-8` with adaptive thinking at `medium` effort — a
low/medium/high/xhigh sweep over the recorded 3-day stress test found quality flat
across levels while latency scaled 23→68 s/call, so medium is the default;
`PREDICTION_CLAUDE_MODEL`/`PREDICTION_EFFORT` in `utils/llm_config.py`; the LTM update
also runs on Opus, while FLAIR planning/transparency stay on `claude-haiku-4-5`) per
**prediction round**. The request prefers **fast mode** (`PREDICTION_FAST_MODE`,
research preview: same Opus weights at up to 2.5× output tokens/sec for 2× price —
the call is output-dominated, so this roughly halves the prediction round — ~30 s at
medium effort today, ~15 s with fast mode — where the latency is still user-visible:
`start()`, mid-batch ask joins, late `record_color` joins). Access is gated; `_create_prediction_message` falls back to standard speed on
any fast-request failure. The API reports "access not granted" as a 429 with a
fast-mode quota of **0** (`anthropic-fast-input-tokens-limit: 0`, verified live) — that
and any hard 4xx latch fast mode off for the run, while a genuine capacity 429
(non-zero quota) retries fast on the next call. The served speed is recorded in each
`prediction_model_llm_calls/*.txt` header, and the meal-end LTM update always runs at
standard speed. Each round's prompt is assembled from three memory systems plus
guardrails. Rounds are indexed by the
number of corrections so far: round `m = 0` runs at meal start
(`PreferenceSession.start()`), and one further round runs after every user correction
(from an ask page, a settings-overlay edit, a changed pickup color, or a changed nav
offset), re-predicting all still-open dims:

```
ŷ_t^{(m)} = Validate( f_LLM( φ_u,  S_{t−1},  E_k(c_t, F_t^{(m)}),  W_t^{(m)} ) ),   m = 0, 1, 2, …
```

where `F_t^{(m)}` is the set of dims already finalized after `m` corrections. Finalized
dims are **pinned** (they override the LLM output verbatim), but they reach the prompt as
two separate blocks: dims the user **CORRECTED** (actively changed — passed via the
`corrected` argument, and the only signal used in the episodic-retrieval query) and dims
the user **CONFIRMED** (accepted as-predicted — passed via `confirmed`). The prompt also
explains the confirm-or-correct interaction protocol, so the model can weigh corrections
as evidence about the meal's latent factors while treating confirms as weaker signal. A
confirmation finalizes a dim without triggering a new round, so the number of LLM
prediction calls per meal is exactly `1 + (number of corrections)`. Memory (`S`, episodic
history) is read-only during all rounds — it is written once at meal end (§2.6).

- **Semantic / long-term memory `S`** (`methods/long_term_memory.py`): a cumulative JSON
  user profile — for every dim a `default` plus `user_tendencies` written as explicit
  conditional rules (`IF <context> AND <affective state> AND <food> THEN <option> BECAUSE
  <reason>`, `methods/prompts/ltm_update.txt`). Updated **once per meal** at finalize by a
  second LLM call that folds the day's episode into the previous summary:
  `S_t = g_LLM(S_{t−1}, e_t)`. Cold start uses `previous = "N/A"`.
- **Episodic memory `E`** (`methods/episodic_memory.py`): every prior day is stored as an
  *episode text* `e_t` = `day; meal; setting; time_of_day` + the full final bundle
  (`methods/utils.py:_episode_text`). At prediction time the current context **and the
  corrections so far** form a query text; episodes are ranked by cosine similarity of
  OpenAI `text-embedding-3-small` embeddings and the top `k = 5` (deployment default;
  eval default 10) are pasted into the prompt.
- **Working memory `W`**: the current context, the meal's concrete solids/dips (so
  `bite_ordering` can be grounded in real food names), and the user's responses so far,
  split into a CONFIRMED block and a CORRECTED block. The prompt carries an explicit
  correlated-preferences instruction — corrections are observations of latent factors
  (user state, environment/lighting, the meal) and should move related open dims,
  continuous ones included — and instructs the model to prioritize `φ_u` (feasibility) >
  working > episodic > semantic memory. It also requires an `"explanations"` object in
  the response: one sentence of evidence per OPEN dim (kept on
  `PredictionModel.last_explanations`, logged in the `preference_predicted` event, and
  printed by the terminal emulator).
- **Seeding for continuous dims**: each color / nav-offset line in the prompt carries a
  *seed* — the currently saved value read from the per-user behavior-tree YAML
  (`PreferenceSession._read_color_seed` / `_read_nav_offset_seed`) — framed as "one piece
  of evidence among the others", decisive only when neither this meal's corrections nor
  similar prior meals bear on the dimension. Day 1 the
  seeds are factory defaults (`DEFAULT_COLOR = {h:12,s:223,v:169,range:0.1}`,
  zero offsets); later days they are the last accepted values, because the per-user BT
  YAMLs persist across days. So for continuous dims the learner is effectively a
  *carry-forward prior with LLM-proposed deltas*.
- **Deterministic guardrails** (validation loop `prediction_model.py:538-575`, hard rules
  `:83-97`):
  - each categorical output is validated against its option list; invalid/missing values
    fall back to the correction if one exists, else the **first option**;
  - color / nav-offset outputs are parsed and clipped into their boxes
    (`parse_color`/`parse_nav_offset`), falling back to the seed;
  - text output falls back to the correction, else `"no particular order"`;
  - **hard rules** (`_apply_hard_rules`): `transfer_mode = inside` forces
    `outside_mouth_distance = "not applicable"`; a catalog meal without dippables or
    without sauces forces `bite_dipping_preference = "do not dip"` (both skipped for dims
    the user explicitly corrected);
  - **corrections always override** the LLM output verbatim.

### 2.6 Learning update

Exactly one memory write per meal (`PreferenceSession.finalize_meal` →
`PredictionModel.update`, `methods/prediction_model.py:324`): build the episode text from
the finalized bundle, fold it into the LTM summary (LLM call), append it to episodic
history, and write three per-day JSON records (working / episodic / long-term memory).
Repredictions during the meal only *read* memory. Cross-day state is re-hydrated at process
start (`load_prior_memory`): the latest prior LTM summary (it is cumulative) plus every
prior episode text.

---

## 3. Code map

```
src/feeding_deployment/
├─ preference_learning/
│  ├─ config/
│  │  ├─ preference_bundle.py      # the 27 dims + color/nav-offset codecs & bounds
│  │  ├─ mealtime_context.py       # meal catalog, settings, times; FLAIR food-item derivation
│  │  ├─ physical_capabilities.py  # 5 reference capability profiles (offline pipeline)
│  │  └─ affective_state.py        # affective vocab (synthetic data only)
│  ├─ methods/
│  │  ├─ prediction_model.py       # PredictionModel: predict_bundle / update / persistence
│  │  ├─ long_term_memory.py       # LongTermMemoryModel (cumulative JSON profile)
│  │  ├─ episodic_memory.py        # EpisodicMemoryModel + EmbeddingCache
│  │  ├─ utils.py                  # episode text, retries, truth extraction (offline)
│  │  ├─ prompts/                  # bundle_prediction.{py,txt}, ltm_update.{py,txt}
│  │  ├─ evaluate_prediction_model.py, metrics.py, compare_metrics.py   # offline eval
│  │  └─ reports/                  # eval outputs (note: predate the current bundle)
│  └─ data_generation/             # synthetic users + 30-day datasets (LLM-generated)
├─ integration/
│  ├─ run.py                       # executive: staged meal, PDDL planning, resume, --day
│  ├─ preference_session.py        # PreferenceSession: the per-meal state machine
│  ├─ apply_preferences.py         # bundle → BT YAML writes + planner/FLAIR side effects
│  ├─ preference_context.py        # context validation
│  ├─ terminal_preferences.py      # stdin/stdout stand-in for the web pages
│  ├─ checkpoint.py                # CheckpointStore (+ pref_session.json snapshot)
│  └─ data_logger.py               # per-day release record (events, images, json)
├─ interfaces/
│  ├─ web_interface.py             # iPad message contract (asks, picker, settings, nav adjust)
│  └─ perception_interface.py      # camera/head/attachment/appliance perception hub
├─ perception/
│  ├─ attachment_perception/       # HSV handle detection (grounds plate_color_*)
│  ├─ appliance_perception/        # door handles / microwave button (GroundedSAM + Molmo)
│  ├─ head_perception/             # DECA head mesh + landmarks + tool-tip target
│  └─ gestures_perception/         # mouth_open / head_nod + LLM-synthesized gestures
└─ actions/
   ├─ base.py                      # behavior-tree machinery (params → skill kwargs)
   ├─ navigate.py                  # PositionOffset apply + post-arrival teleop learning
   ├─ pick_plate.py                # HandleColor/ColorRange consumption + write-back
   ├─ acquisition.py               # skewering / dipping / confirmation consumption
   ├─ transfer_tool.py             # interaction-cue and readiness-detection consumption
   └─ flair/                       # bite planner (consumes bite_ordering, allow_dip)
webapp/src/views/                  # preference_context, preference_correction,
                                   # color_correction, nav_adjust_confirm, settings overlay
tests/                             # test_preference_session / _integration / _apply / _checkpoint
```

Persistent per-user state (survives across days) lives under
`src/feeding_deployment/integration/log/<user>/`:

- `behavior_trees/*.yaml` — the actuated preference state (all BT params incl. colors,
  offsets); seeded from `actions/behavior_trees/` for a new user (`run.py:212-221`).
- `preference_learning/<user>/{working,episodic,long_term}_memory/day_NNNN.json` and
  `…/prediction_model_llm_calls/*.txt`, `…/long_term_memory_llm_calls/*.txt`.
- `flair_history.txt` (FLAIR bite history + food labels + bite-ordering preference),
  `nav_offset_log.jsonl` (every adjustment event), `llm_cache/`, `day_<NN>/` release logs.
- `saved_states/` (repo-level): `NN_<skill>.p` checkpoints, `after_*_pickup.p`,
  `last_state.p`, `pref_session.json`.

---

## 4. Runtime flow of one meal

Entry point: `python -m feeding_deployment.integration.run --user U --day N
--use_interface --run_on_robot …` (`--day` is mandatory and is the single source of truth
for both release logging and the memory day). `--pref_mode` defaults to `interface` when
`--use_interface` is set.

1. **Start gate** — wait for "Start Meal" on the iPad home page (`run.py:787`).
2. **Context** — the `preference_context` page collects `(meal, setting, time_of_day)`
   (`web_interface.get_meal_context`), unless preset via `--pref_meal` (debug). Validated
   against the vocabularies.
3. **Model build** — `PredictionModel` is constructed; `validate_sequential_day(N)` rejects
   gaps; `load_prior_memory(N)` re-hydrates LTM + episodic history from days `< N`
   (`run.py:584-603`).
4. **`PreferenceSession.start()`** (`preference_session.py:633`) — one LLM call predicts
   all 27 dims (seeded from the BT YAMLs); FLAIR gets the meal's food items (derived
   deterministically from the catalog with singularization,
   `mealtime_context.food_items_for_flair`); predictions for open color/nav dims are
   **written into the BT YAMLs**; all categorical dims are applied
   (`_apply_non_planning`: BT writes + transfer-object re-init + FLAIR dip/ordering).
5. **Initial ask** — `ask(INITIAL_PREF_DIMS)` = `robot_speed`,
   `confirm_navigation_arrival`, `confirm_manipulation`,
   `wait_before_autocontinue_seconds`, in that order: the two confirmation-mode dims fire
   first during the fridge leg (arrival page, handle-detection page) so they must be
   finalized up front, and they are asked BEFORE the wait pref so the user learns what
   pages exist (and what "autocontinue" refers to) before choosing its duration. The
   finalized wait then drives every later correction page's autocontinue countdown AND
   every confirmation page in autocontinue mode (`PreferenceSession.wait_seconds`;
   default 10 s before it is finalized).
6. **Prep pipeline** (`_run_meal_preparation`, fixed indices for resume):
   1. `PlacePlateOnHolder` — the fridge pickup runs inside this plan; **fridge color** is
      corrected/confirmed during the pickup, then `record_color("fridge")` finalizes the
      dim from the YAML (`run.py:1126-1136`). Navigations run inside plans too; each
      arrival offers the position-adjust prompt and then `record_nav_offset(loc)`.
   2. `CloseDoor(fridge)`.
   3. `ask(["microwave_time"])` then `apply_microwave(...)` — `"no microwave"` adds the
      `FoodHeated` atom so the PDDL planner skips the whole microwave detour; a duration
      leaves it unset and the BT `MicrowaveDuration` was already written. `microwave_time`
      is then **locked** in the settings overlay (the routing is committed).
   4. `PlacePlateOnTable` — routes through the microwave if needed (microwave color
      corrected during that pickup).
   5. `ask(_TABLE_PREF_DIMS)` — the 15 remaining categorical/text dims, one page each.
7. **Feeding loop** — task selection (bite / sip / wipe / gestures / transparency /
   adaptability / teleop). Transfer skills consume the interaction dims (§6). Before each
   HLA, the executive stalls while the settings overlay is open, joins the background
   reprediction if the skill's BT consumes predictions (`bt_consumes_predictions`), and
   flushes any deferred transfer re-init (run.py before-skill block).
8. **Finish** — "finish feeding" plans `PickPlateFromTable` (table color correction) then
   `PlacePlateInSink`; afterwards `finalize_meal(day)` performs the single memory update
   (any never-asked dims are finalized at their predicted values as implicit confirms) and
   the settings accessor is cleared (`run.py:760-771`, `_finalize_preference_session`).

**Correction semantics inside `ask()`** (`preference_session.ask`): each dim is
shown with its prediction highlighted, under a plain-language headline (`dim.label`) and a
one-sentence subtitle (`dim.short_description`, sent as `description` in the
`preference_correction_data` message — the long `description` is written for the LLM and
appears only in the settings overlay); the page auto-confirms the prediction when the
countdown expires without interaction (`webapp/src/views/preference_correction.vue` — user
interaction cancels the countdown; the predicted value is prepended to the options if
missing). A returned value ≠ prediction is a correction: it is finalized synchronously,
recorded in `corrected`, and a **background reprediction** of every still-open dim is
scheduled (finalized dims are pinned via the `corrected`/`confirmed` arguments of
`predict_bundle`); the pass also re-writes open color/nav predictions to the YAMLs.
`ask()` joins the worker before showing each step, so a mid-batch correction still waits
for its reprediction (the page must show values conditioned on it) while the batch-final
correction's LLM call overlaps the following robot motion. A `bite_ordering` correction is
first cleaned for grammar/grounding by FLAIR's meal parser (raw text kept on any failure).

**Settings overlay** (`web_interface.py:372-427`, `preference_session.settings_view/edit`):
a latched topic publishes the finalized categorical dims (colors/text/nav excluded; locked
dims non-editable). Edits arrive on a dedicated worker thread and are treated as
corrections: the edited value is finalized and applied to the BTs **synchronously** (fast,
no LLM — the next skill sees it even before any reprediction lands), a background
reprediction of the open dims is scheduled, and a `transfer_mode` edit defers the
transfer-object reconstruction to the main thread (`flush_pending_inmemory`) so it never
swaps under an in-flight motion. The executive and the ask pages stall while the panel is
open.

**Threading model** (`preference_session.py` module docstring): all correction-triggered
repredictions run on a single **coalescing daemon worker** — at most one LLM call in
flight; triggers arriving mid-pass mark the state dirty and get exactly one follow-up pass
carrying the newest corrections (`_schedule_repredict`/`_repredict_worker`). Consumers
join via `wait_for_reprediction()`: `ask()` before each step, `record_color`/
`record_nav_offset` on entry (the observed-vs-predicted comparison runs against the
settled prediction), `finalize_meal` on entry, and run.py before every skill whose BT
reads prediction-produced parameters (`bt_consumes_predictions`: `pick_plate_from_*`,
`navigate_to_*`, `transfer_*`, `acquire_bite` — run.py's before-skill order is
settings-closed stall → join → `flush_pending_inmemory`). All BT-YAML writers serialize
on a dedicated mutex acquired *before* the session lock. Two guards close the write-back
races: **external writes win** — the worker skips any color/nav field whose YAML changed
(vs. the seeds its prediction was computed from) while the LLM call was in flight, so a
picker/teleop write-back is never clobbered by a stale prediction — and `record_*`
re-assert the observed YAML value after finalizing. A failed background pass is swallowed
(previous predictions stay applied; joiners never wedge). `capture_state` deliberately
does **not** join — a crash between a correction and the worker finishing resumes with
the pre-repredict open predictions, which is safe (merely re-correctable).

**Crash/resume**: every locked correction immediately overwrites
`saved_states/pref_session.json` (`_finalize` → `on_change` → `CheckpointStore.save_pref`);
every completed sub-skill checkpoints the sim state + atoms + a session snapshot
(`run.py:_save_state`). `--resume_from_state <name>` restores atoms/sim, prefers the
standalone pref snapshot over the embedded copy, rebuilds the prediction model (memory is
never pickled), re-applies the bundle **without predicting or asking**, and re-enters the
prep pipeline at the recorded step; `ask()` skips finalized dims. Profile and day
mismatches versus the checkpoint prompt for confirmation (`run.py:496-524`).

---

## 5. Actuation: how each dimension changes behavior

`apply_preferences.py` declares the mapping bundle-field → (YAML files, BT parameter,
translator) in `_BT_MAPPING`. `apply_bundle_to_behavior_trees` loads each affected YAML
once, overwrites `value` entries, and writes back **atomically** (temp file + `os.replace`;
justified because the settings worker thread may write while the executive reads,
`apply_preferences.py:260-281`). BT YAMLs are re-read fresh at each skill execution
(`base.py:execute_action` → `load_behavior_tree`), and YAML parameters bind **positionally**
to the `!hla <fn>` skill function's arguments (`base.py:783-831`) — parameter order in the
YAML must match the function signature.

| bundle field | BT parameter (files) | consumed by |
|---|---|---|
| `robot_speed` | `Speed` (all 29 YAMLs), slow/medium/fast → low/medium/high | `robot_interface.set_speed` in every skill |
| `confirm_feeding_pickup` | `TransferAskForConfirmation` (acquire_bite), `AskForConfirmationInitiatingTransferSequence` (transfer_drink/wipe) — mode-coded 0/1/2 via `_CONFIRM_MODE_MAP` | bite/drink/wipe pickup-verification pages: 0 = skip, 1 = countdown then auto-confirm, 2 = block |
| `confirm_navigation_arrival` | `AskForArrivalConfirmation` (navigate_to_*) — mode 0/1/2 | post-arrival "Position OK / Adjust" page (`navigate.py:_offer_position_adjustment`); 0 skips it and freezes nav-offset learning |
| `confirm_manipulation` | `AskForManipulationConfirmation` (pick_plate_from_{fridge,microwave,table}, place_plate_{in_microwave,on_table,in_sink}, press_microwave_button, gaze_at_table, open_{fridge,microwave}) — mode 0/1/2 | detection-confirmation pages (plate handle / door handle / button / sink / plate; 0 auto-accepts successful detections and freezes color learning) + the plate-release confirms |
| `wait_before_autocontinue_seconds` | `TimeToWaitBeforeAutocontinue` (acquire_bite, transfer_utensil, transfer_drink) | bite-selection / re-selection page countdowns; also (in memory, via the injected `get_autocontinue_seconds` provider) all correction pages and every confirmation page in autocontinue mode |
| `outside_mouth_distance` | `OutsideMouthDistance` (3 transfer YAMLs), near/medium/far → 0.07/0.10/0.13 m; "not applicable" skips the write | outside-mouth transfer stop distance |
| `convey_robot_ready_for_initiating_transfer` | `ReadyToInitiateTransferInteraction` → silent/voice/led/voice_led | `transfer_tool.relay_ready_to_initiate_transfer` (speech text adapts to the readiness mode; LED via serial) |
| `detect_user_ready_for_initiating_transfer_*` | `InitiateTransferInteraction` → open_mouth/button/auto_timeout (per tool YAML) | `detect_initiate_transfer` (`transfer_tool.py:71`): mouth-open detector / physical button relayed by the webapp (`web_interface.detect_button_press`, armed only while waiting; skipped without a web interface) / **fixed 5 s sleep**; may also be an LLM-synthesized gesture name |
| `convey_robot_ready_for_completing_transfer` | `ReadyForTransferInteraction` | `relay_ready_for_transfer` |
| `detect_user_completed_transfer_*` | `TransferCompleteInteraction` → sense/button/auto_timeout | `detect_transfer_complete`: fork → FT bite-down trigger (|torque_x| > 0.1), drink/wipe → head nod; webapp-relayed button; fixed 5 s |
| `skewering_axis` | `SkeweringOrientation` (acquire_bite) → horizontal/vertical | `acquisition.py:264-265`: horizontal adds π/2 to the detected major-axis angle |
| `bite_dipping_preference` | `FoodDippingDepth` (acquire_bite) → 0.01/0.03 m; "do not dip" skips the write | dip skill depth; suppression is done separately via `flair.set_allow_dip(False)` which strips any LLM-planned dip (`inference_class.py:1066-1068`) |
| `microwave_time` | `MicrowaveDuration` (press_microwave_button) → 60/120/180 s | microwave button skill; "no microwave" is handled at the *planner* level via the `FoodHeated` atom (`apply_microwave_preference`) |
| `retract_between_bites` | `RetractAfterTransfer` (transfer_utensil) | retract move after bite transfer |
| `transfer_mode` | (no BT param) | `scene_description.transfer_type` + reconstruction of the `TransferToolHLA.transfer` object (Inside/OutsideMouthTransfer, `apply_preferences.py:416-468`) |
| `bite_ordering` | (no BT param) | `flair.set_preference(text)` → pasted into the FLAIR preference-planner prompt |
| `plate_color_*` | `HandleColor` `[h,s,v]` + `ColorRange` (pick_plate_from_*) | attachment perception (§7.2) |
| `nav_offset_*` | `PositionOffset` `[dx,dy,dyaw]` (navigate_to_*) | goal composition (§7.4) |

Write-backs in the *other* direction (perception/action → bundle) go through
`process_behavior_tree_parameter_update` (`base.py:274-307`), which validates the value
against the parameter's declared space (`EnumSpace`/`Box`) — the `PositionOffset` Box in the
navigate YAMLs must match `NAV_OFFSET_BOUNDS` (`preference_bundle.py:353`), and both match
`NavigateHLA._MAX_OFFSET_*` (`navigate.py:95-96`).

---

## 6. The two "physical" correction channels

These are the parts that make this more than a form-filling exercise: two bundle dimensions
are corrected through *acting in the world*, and the pipeline turns those physical
interactions into the same finalize/repredict/learn cycle as a button press.

### 6.1 Plate-handle colors

- Before any pickup, the session wrote the (possibly LLM-updated) color prediction into the
  pickup's YAML. The pickup skill passes `HandleColor`/`ColorRange` into
  `perceive_attachment_poses` (`pick_plate.py:85,129,288`).
- The user sees the detection overlay (matched pixels, kept cluster, corner brackets, pose
  gizmo) and chooses **Confirm / Redo / Correct Color**. Correct Color opens the live
  picker: tap a pixel → RGB → HSV, adjust the tolerance slider, **Rerun** re-detects on a
  fresh frame (fresh TF stamp), **Confirm** locks the pose *from the exact frame the user
  approved* (`_interactive_color_correction`, `perception_interface.py:830`). The page itself is gated by
  `confirm_manipulation`: "no" auto-accepts a successful detection (picker unreachable →
  color learning frozen), "yes (with auto-continue countdown)" adds a countdown that auto-confirms.
- If detection fails outright (all 20 attempts return `None`, typically a stale stored
  color), the run no longer aborts: `perceive_attachment_poses` routes the user straight
  into the same live picker with the "No detection" badge and no pre-populated result;
  Confirm stays gated until a Rerun succeeds, and the confirmed color persists to the YAML
  identically. The fatal `RuntimeError` remains only in terminal mode (no web interface)
  or when the camera never produced a valid frame.
- A changed color/range is written back into the YAML by the skill
  (`pick_plate.py:95-98`), and after the HLA completes the executive calls
  `record_color(location)`: the session joins any in-flight background reprediction, reads
  the YAML back, finalizes the dim, and — if it changed — schedules a background
  reprediction so the correction propagates to the other two pickup colors (same physical
  handle) while the robot keeps moving; the next prediction-consuming skill joins before
  its BT loads.

### 6.2 Navigation offsets

- `NavigateHLA` composes the learned `PositionOffset` onto the nominal mapped goal in the
  goal's local frame, clamped to ±0.5 m / ±45° (`navigate.py:626-650`).
- After a successful arrival, the iPad asks **"Position OK / Adjust"** — except after a
  recovery-teleop rescue, and except when the user completed the leg themselves by parking
  the base and pressing **Done** in teleop: a human park is final, so that leg skips the
  goal-confirm replan, the refinement window, and the adjust prompt entirely and records no
  offset event (`_last_leg_human_completed`, `navigate.py:402-425,566,1059-1067`). When
  shown, the prompt is gated by `confirm_navigation_arrival`: "no" skips the page
  entirely (offsets frozen at the learned totals), "yes (with auto-continue countdown)" shows it with the
  user's autocontinue wait (timeout ⇒ OK), "yes (without any auto-continue)" shows it with no countdown
  (`web_interface.get_nav_position_adjust_choice`; `autocontinue_seconds <= 0` = no
  countdown). On Adjust, the base is handed to the
  webapp joystick (no active move_base goal), the user parks it, presses Done; after a 10 s
  localization settle the new **total** offset is measured as the user's final localized
  pose expressed in the *nominal* goal frame (`navigate.py:1152-1153`) — accumulation is by
  construction, and nav parking noise the user just drove out is not double-counted. A
  movement gate (2 cm / 2°, measured on the user's actual motion) keeps TEB parking scatter
  out of the learned offset. The clamped total is written into the YAML and logged to
  `nav_offset_log.jsonl`; `record_nav_offset(loc)` then finalizes the dim (re-finalizing on
  later navigations in the same meal — the latest total is the meal's ground truth,
  `preference_session.py:803-841`).

---

## 7. Perception pipeline (thorough)

The perception stack is what grounds half the bundle. Hub:
`interfaces/perception_interface.py` (`PerceptionInterface`), which owns a single
`RealSenseInterface` (RGB + 32FC1 depth in **millimeters** + `CameraInfo`), a shared
`GroundedSAM`, `HeadPerceptionROSWrapper` (DECA), `AppliancePerception`,
`AttachmentPerception`, `DrinkPerception`, the FT sensor subscriber
(`/forque/forqueSensor`), the legacy transfer-button subscriber (`/transfer_button` — no
longer used by the transfer skills; the physical button is now relayed by the webapp, §7.3),
speech (`/speak`), and
the LED (serial). With `robot_interface=None` it runs in simulation mode and replays
pickled perception results from the user log dir.

### 7.1 Camera & frames

- All 3D math uses the pinhole model with `CameraInfo.K` (`pixel2World`/`world2Pixel`,
  `attachment_perception.py:540-578`). Depth is validated to **0.05–1.0 m** — anything
  farther than 1 m is treated as invalid, which bounds where handles can be detected.
- Poses are lifted from `camera_color_optical_frame` to `arm_base_link` via TF
  (`perception/tf_interface.py`); the detection publishes an `attachment` TF frame and
  RViz markers.
- The physical camera is mounted **upside down**; the central 180° flip is applied once in
  `WebInterface._send_image`. The microwave-inside gaze physically flips the camera, so
  those captures pass `camera_flipped=True` to suppress the flip
  (`pick_plate.py:126-129`).

### 7.2 Attachment (plate-handle) perception — grounds `plate_color_*`

`AttachmentPerception.detect_attachment` (`attachment_perception.py:48`):

1. **HSV mask** (`detect_attachment_color:422-461`): tolerance is *asymmetric by design* —
   hue half-width `= range·90` (kept tight: hue is the discriminative channel; at the
   default `range = 0.1` that's ±9 hue units ≈ ±18°), S/V half-widths `= range·255`
   (±25.5). Hue is circular: bands straddling 0/179 are split into two `inRange` calls and
   OR-ed (red handles work); `h_tol ≥ 90` degenerates to S/V-only matching. The factory
   default `[12, 223, 169]` matches `DEFAULT_COLOR` in the bundle. (`clean_mask` -
   morphological open/close - exists but its call is commented out, line 78.)
2. **3D lift + clustering**: every mask pixel with valid depth → 3D; DBSCAN
   (eps = 7 cm, min_samples = 5) and the **largest** cluster wins — a larger
   similarly-colored background blob can out-vote the real handle (the overlay shows
   rejected matches in white / kept cluster in red exactly so the user can catch this).
3. **Plane + rectangle**: RANSAC plane (3 mm threshold) on the cluster; the planar inliers
   are projected into plane coordinates; `cv2.minAreaRect` gives center + 4 corners; all
   are back-projected to 3D.
4. **Orientation**: the full perceived rotation is *not* trusted. A hand-tuned nominal
   quaternion per mounting (`front` for fridge/microwave, `left` for table,
   `_NOMINAL_HANDLE_QUAT:306`) bakes in roll/pitch and approach direction; only the **yaw**
   of the detected face normal (heading in the horizontal base-frame plane) is applied,
   clamped to ±60° and skipped when the normal is near-vertical
   (`_perceived_face_yaw:311-352`).
5. **Grasp poses**: fixed per-handle-type offsets in the attachment frame produce
   `pickup / pre_pickup / above_pickup / post_pickup` poses
   (`perception_interface.py:1088-1127`); results are pickled for sim replay.

Detection loop robustness (`perceive_attachment_poses`, `perception_interface.py:952`): 5 s auto-exposure settle,
then up to 20 frames at 10 Hz; **if all 20 fail it raises `RuntimeError`**, which the
executive converts into a `FatalSkillFailure` (meal over, operator restart) — see corner
case C1.

### 7.3 Head & gesture perception — grounds `detect_user_ready/completed_*`

- `HeadPerceptionROSWrapper` + `deca_perception.py` fit a DECA head model per frame and
  produce `head_perception_data = {face_keypoints (68-landmark), head_pose (x,y,z,r,p,y),
  tool_tip_target_pose}`. A dedicated thread (`run_head_perception_thread`,
  `perception_interface.py:247-301`) refreshes it (intended 50 Hz; see C13), with a
  10-frame warmup. `--simulate_head_perception` replays a pickled frame instead.
- **Open-mouth** (`static_gesture_detectors.mouth_open`): mouth aspect ratio
  `(‖p51−p59‖+‖p53−p57‖)/(2‖p49−p55‖) > 0.45`, polled at 10 Hz, timeout 600 s.
- **Head nod** (`head_nod`): pitch swings — 3 direction changes of > 15° — timeout 600 s.
- **Force "sense"** for the fork: `|torque_x| > 0.1` on the FT sensor
  (`perception_interface.ft_callback:191-197`). Button: the physical button is detected in
  the browser (App.vue) and relayed by the webapp's `robot_executing` page **only while the
  robot has armed it** (`WebInterface.detect_button_press`, `web_interface.py:533-569`;
  `button_arm`/`button_press` messages — stray presses while unarmed are dropped). The old
  `/transfer_button` perception path is no longer used by the transfer skills; with no web
  interface the button wait is skipped entirely.
- **Synthesized gestures**: the personalization "gesture" flow records positive/negative
  landmark examples, asks the LLM to synthesize a detector function, appends it to the
  per-user `gesture_detectors/synthesized_gesture_detectors.py`, registers the function
  name into every `InitiateTransferInteraction`/`TransferCompleteInteraction` EnumSpace
  (`run.py:register_gesture_detector`), and executes it later via `exec()` of the per-user
  file (`load_synthesized_gestures`). So the option lists of two preference dims can grow
  at runtime beyond the bundle's canonical options (the settings overlay would reject
  such values — bundle validation only knows the canonical three).

### 7.4 Navigation localization — grounds `nav_offset_*`

Offset measurement is closed-loop on the map→base TF (Cartographer/ZED chain), never on
integrated teleop cmd_vel (`navigate.py:972-1009`); staleness checks
(`_localization_fresh`) skip the prompt/measurement rather than record garbage. Settle
times: 15 s goal-confirm, 10 s post-adjustment (`navigate.py:87,109` — goal-confirm was
retuned 25 → 10 → 15 s to guarantee at least one Cartographer optimization epoch).

### 7.5 FLAIR food perception — grounds `bite_ordering` / dipping

- Food classes come from the meal catalog, deterministically singularized
  (`mealtime_context.food_items_for_flair`) so detector prompts match FLAIR's expectations
  ("chicken nuggets" → "chicken nugget"); `_SINGULAR_OVERRIDES` handles irregulars.
- `FLAIR.detect_items` (`flair.py:127-286`): GroundedSAM detection + plate crop, per-item
  masks/portions/bounding boxes, "blue plate" detections removed, labels mapped back to
  catalog categories (unknown labels default to `solid`), per-food-type mask groups and a
  skill per category (Skewer / Scoop / Dip / Twirl).
- `get_autonomous_action` (`inference_class.py:1029-1079`): picks a random instance mask
  per solid, computes skewer/dip keypoints, then asks the **preference planner**
  (`preference_planner.py`, Claude via `GPTInterface`) for the next bite given items,
  rounded portions, efficiencies, the user's `bite_ordering` sentence, dips, and the bite
  history; the response's `Next bite as list:` line is `ast.literal_eval`-ed. With
  `allow_dip=False` any planned dip is stripped deterministically.
- The webapp bite-selection page shows the predicted bite (+ dip options) with the user's
  autocontinue timeout; the user can accept, pick another item, or manually point
  (`acquisition.py:210-331`). If FLAIR finds no actionable bite (nothing detected, only
  dips, or the planner can't match one), detection is silently retried; after 3 consecutive
  failures (`MAX_CONSECUTIVE_FAILED_DETECTIONS`, `acquisition.py:38,213-232`) the page is
  shown in no-detection mode offering **manual skills only** (manual_skewering /
  manual_dipping), with no autocontinue countdown.

### 7.6 Appliance perception (context)

Not a preference consumer, but part of the same perception stack: `AppliancePerception`
detects fridge/microwave handles, hinge sides, sink/table placements (GroundedSAM), and the
microwave start button by querying a **Molmo VLM behind a hardcoded ngrok URL**
(`appliance_perception.py:76`) with an image flipped before upload and coordinates
un-flipped after; `remote_molmo.py` is an alternative SSH/file-polling transport with
hardcoded host paths. Both are deployment-brittle (see C17).

---

## 8. Offline pipeline: data generation & evaluation

- **User encodings** (`data_generation/generate_user_preference_encoding.py`): synthetic
  users = a capability profile + LLM-generated stable tendencies.
- **Datasets** (`generate_deployment_dataset_llm.py`): for each of ~30 days, sample
  context (meal/setting/time/affective state), ask the LLM for the *joint* ground-truth
  bundle with rationales, apply the same hard rules, validate strictly
  (`_validate_joint_output_strict` — every field, `choice` must be a string from the
  option list), and write `{user, physical_profile_label, days:[{day, context,
  preferences:{field:{choice, rationale}}}]}`. **This validator predates the continuous
  dims — see B2.**
- **Evaluation** (`evaluate_prediction_model.py`): replays a dataset day-by-day with a
  simulated oracle user: predict the full bundle; while any unrevealed dim mismatches the
  ground truth, reveal one *random* mismatched dim as a correction (up to
  `--max-corrections 18`) and repredict; then `PredictionModel.update`. Ablations select
  the memory systems: `full` / `ltm_only` / `em_only` / `no_memory`. Metrics per user:
  `accuracy_after_m`, per-day `acc_m0/acc_m1/mismatches/m*`, per-affective-state
  aggregates, per-dimension m0 accuracy, zero-correction meal counts (total and final
  week). Reports (JSON + txt + plots via `metrics.py`) land in `methods/reports/run_<ts>/`.
- The existing `methods/reports/` runs predate the current bundle (their logs contain a
  removed `occlusion_relevance` dim), so treat them as historical.

---

## 9. What the tests pin down

- `tests/test_preference_session.py` (fake model + scripted web interface): full-bundle
  predict at start incl. color seeding from/fallback around the BT YAML; confirm ≠
  correction; correction ⇒ repredict with pinned finalized dims; `wait_seconds` from the
  finalized pref; `record_color` propagation vs confirm; exactly one memory update at
  finalize with the full bundle; settings overlay (finalized-categorical-only view, edit
  validation, `microwave_time` locking, deferred transfer re-init, thread-safety);
  atomic YAML writes under concurrent reads; ask-skips-finalized; capture/resume roundtrip
  preserving pre-crash corrections; `on_change` persistence on every correction and
  failure isolation; repredictions never write memory.
- `tests/test_apply_preferences.py`: the full `_BT_MAPPING` translation table, skip
  semantics ("not applicable", "no microwave", "do not dip"), microwave planner-atom
  behavior, transfer-mode re-init, FLAIR dip/ordering setters, warning paths.
- `tests/test_checkpoint.py`: numbering, feeding recovery names, pref snapshot roundtrip
  and clearing.
- `tests/test_preference_integration.py`: context validation; `PredictionModel` prompt
  content, correction override, malformed-JSON fallback, disk logging (LLM + embeddings
  mocked); bundle config invariants.

**Not covered by tests**: the real LLM prompt/response behavior, the color-picker loop,
nav-offset measurement math, the webapp message contract, all of perception, resume-via-
run.py end-to-end, and the offline data-generation/eval pipeline.

---

## 10. Corner cases and bugs

Verified against the working tree. **B** = bug (behavior is wrong or a pipeline is
broken), **C** = corner case / sharp edge (works as coded, but surprising or fragile).

### Bugs

- **B1 — Embedding cache is never persisted.** `EmbeddingCache.flush()` is only called
  from `EpisodicMemoryModel.reset()` (`episodic_memory.py:120-122`), and nothing —
  deployment (`prediction_model.py`) or eval — ever calls `reset()`/`flush()`. So
  `preference_learning/embeddings.json` is read if present but never written: every day's
  process re-embeds all prior episode texts and every query via the OpenAI API (cost +
  latency + a network dependency at every reprediction; a fresh embedding of an identical
  text is also a silent behavior dependency on embedding determinism).
- **B2 — The dataset generator cannot generate the current bundle.**
  `_validate_joint_output_strict` (`generate_deployment_dataset_llm.py:88-127`) requires
  *every* `PREFERENCE_BUNDLE` field to appear with a non-empty **string** `choice`
  contained in `dim.options`. For the color/nav-offset/text dims `options == []`, so no
  value can ever validate (a dict fails the string check; any string fails membership).
  Generation with the current bundle always raises. The prompt side
  (`prompts/system_description.py`) *was* updated for these kinds — the validator wasn't.
- **B3 — Offline eval can never score continuous dims correct.**
  `_extract_truth_bundle` (`methods/utils.py:78-88`) formats color truth as a compact
  string (`"h=..,s=.."`) and `str()`s everything else (nav offsets would become dict
  reprs, missing fields become `""` / the default color string), while
  `predict_bundle` returns canonical **dicts** for color/nav dims. `pred.get(f) ==
  truth.get(f)` (`evaluate_prediction_model.py:183-188`) is therefore always `False` for
  these dims — each one is a guaranteed "mismatch" consuming a forced oracle correction
  every meal and deflating `acc@m0` by a constant. Together with B2 this means the offline
  pipeline is effectively categorical-only in its current state.
- **B4 — `bite_mask_idx` indexes the wrong food (and off-by-one on the no-web path).** In
  `get_autonomous_action` (`inference_class.py:1036-1042`) `bite_mask_idx` is overwritten
  inside the loop over solids, so it ends up being a random instance index for the *last*
  solid category — not for the item the planner actually chose. On the no-interface path
  `acquisition.py:277` passes it as `skill_params[1]` and line 286 computes
  `item_id = skill_params[1] - 1` (the webapp path is 1-based, this value is 0-based), so
  `bite_mask_idx = 0` selects `masks[-1]`. Both errors only affect *which instance* of a
  food is skewered (or crash with IndexError if the chosen food has fewer instances), and
  only the non-interface/autonomous path for the off-by-one.
- **B5 — FLAIR bite history is polluted.** `update_bite_history` is called **before**
  skill execution (`acquisition.py:288-289`, the `Rajat Imp ToDo` acknowledges it), so
  failed acquisitions count as eaten bites; and `bite_history` is loaded from
  `flair_history.txt` at startup and **never reset per meal** (`flair.py:24-51`; no caller
  clears it), so the preference planner's "bites taken so far" accumulates across meals
  and days. The planner's portion/ordering reasoning (e.g. "all of item A before B") is
  conditioned on a history that includes other meals' bites.
- **B6 — Head-perception thread throttle is broken.** `run_head_perception_thread`
  computes `step_time = t_now - t_init` but never resets `t_init`
  (`perception_interface.py:250-254`), so after the first 20 ms the "50 Hz" gate is always
  true and the loop free-runs (rate-limited only by DECA inference itself). Relatedly,
  `get_head_perception_data` **writes a pickle to disk on every call**
  (`perception_interface.py:324-326`) — gesture detectors poll it at 10 Hz for up to
  600 s.
- **B7 — README / CLI doc drift.** `preference_learning/README.md` references
  `scripts/evaluate_memory_model.py` and `methods/evaluate_memory_model.py`; the actual
  module is `methods/evaluate_prediction_model.py`, and no `scripts/` variant exists.
  `run.py`'s `--pref_meal` help says "Required with --pref_mode=interface"
  (`run.py:1408-1413`), but it is optional (context is collected on the web page when absent).
  `apply_microwave_preference`'s docstring claims the duration wiring "is not yet
  implemented" (`apply_preferences.py:496`), but `_BT_MAPPING` does write
  `MicrowaveDuration`.

### Corner cases / sharp edges

- **C1 — A bad color prediction stalls the pickup for user correction instead of ending
  the meal.** `start()`/repredictions write the LLM's color for still-open dims into the
  pickup YAML (`_write_open_colors_to_bt`). If that color matches ~zero pixels, detection
  returns `None` for all 20 attempts and `perceive_attachment_poses` routes the user
  straight into the live color picker (see §6.1) rather than raising `RuntimeError` →
  `FatalSkillFailure` (`run.py:1088-1106`); the fatal path remains only in terminal mode
  or when the camera never produced a valid frame. The opposite failure is
  silent: a similar-colored larger blob wins DBSCAN and the robot confidently grasps the
  wrong thing (mitigated by the user-facing overlay + confirm gate). Note this risk grew
  with the correlated-preferences prompt revision: the model is now actively encouraged
  to move open colors off their seeds after a this-meal correction, so a wrong inferred
  color reaching a pickup is more likely than under the old keep-the-seed prompt; the
  clipping in `parse_color` and the 20-frame → picker → redo loop are the remaining
  guards.
- **C2 — Predicted (unconfirmed) nav offsets are actuated.** Open nav-offset predictions
  are written to the YAML at `start()` and after every reprediction, so an LLM-invented
  offset physically moves the parking pose by up to ±0.5 m / ±45° with no confirmation
  step other than the post-arrival adjust prompt. Same seed-carry mitigation as C1.
- **C3 — Silent first-option fallback.** For a categorical dim with invalid/missing LLM
  output and no correction, the value silently becomes `options[0]`
  (`prediction_model.py:554,574`). A whole-response JSON parse failure yields a complete
  "first-option bundle" (`no microwave`, `slow`, `parallel…`, `yes`, `outside…`, …) that
  is actuated and shown as the prediction; the only trace is the raw-response dump in
  `prediction_model_llm_calls/`. There is no retry on malformed JSON (only rate-limit
  retries, `methods/utils.py:29-41`).
- **C4 — LTM update failure is silent and sticky.** If the LTM-update response fails
  `json.loads`, the previous summary is kept with only a printed warning
  (`long_term_memory.py:132-137`); the day's `long_term_memory/day_NNNN.json` then stores
  the *previous* day's summary as `ltm_summary_raw`, and the episode is never folded into
  the profile (episodic memory still records it). No retry.
- **C5 — Autocontinue confirms count as real confirms.** A correction page that times out
  echoes the prediction, which `ask()` finalizes as ground truth (`changed=False`). An
  inattentive user "confirms" everything, and those labels feed the memory update with
  the same weight as engaged confirms. (Since the confirmed/corrected prompt split, the
  *predictor* does distinguish confirms from corrections within the meal — but a timeout
  still lands in the CONFIRMED block, and the LTM episode text does not distinguish
  either.)
- **C6 — `wait_before_autocontinue_seconds` does not affect everything its description
  implies.** Its description (shown to the LLM and user) says it covers "the next bite,
  sip, or mouth wiping", but (a) `transfer_wipe.yaml` deliberately has no
  `TimeToWaitBeforeAutocontinue` (wipe never auto-continues — see its description), and
  (b) the `autocontinue` *readiness/completion* options are hardcoded 5 s sleeps
  (`transfer_tool.py:83-86,130-133`), unrelated to this dim. The `10/100/1000 sec` option
  list is also oddly log-scaled — 1000 s ≈ 17 minutes.
- **C7 — Gesture timeouts are ignored.** `mouth_open`/`head_nod` return `False` after
  600 s, but every caller discards the return value (`transfer_tool.py:82,125-129`), so
  after 10 minutes the transfer proceeds exactly as if the gesture had been detected —
  potentially moving a fork toward a user who never signalled readiness. The detectors
  also busy-spin without sleeping while head data is `None`
  (`static_gesture_detectors.py:22-25`).
- **C8 — Synthesized gestures widen BT option spaces beyond the bundle.** A learned
  gesture becomes a valid `InitiateTransferInteraction` enum value in the YAML, but the
  preference bundle only knows `open mouth/button/autocontinue`; if a synthesized gesture
  is the active BT value, `apply_bundle_to_behavior_trees` will happily overwrite it on
  the next `_apply_non_planning`, and the settings overlay can never select it back.
- **C9 — Day re-runs overwrite memory; resume needs matching flags.** A fresh run with an
  existing `--day N` overwrites that day's memory records (warned at finalize,
  `run.py:768-771`). Resuming a personalized meal requires the same
  `--physical_profile_file`/`--day` (mismatches prompt y/n); resuming a prep-phase
  checkpoint whose pref snapshot *and* embedded session are both missing dies on an
  `assert` (`run.py:662`) rather than a clean message. Preference **context** is not
  persisted at all — after a crash the operator must supply it again
  (`ensure_preference_context`, `run.py:532-541`).
- **C10 — Retrieval text spaces are inconsistent.** The episodic query renders corrected
  color/nav values with `str(dict)` (`episodic_memory.py:15`) while stored episodes use
  the compact `format_color`/`format_nav_offset` encoding — cosine similarity compares
  different textual encodings of the same information. Also `retrieve()` returns a joined
  string though it is annotated `List[str]`, and `get_last_retrieved` therefore logs one
  blob rather than a list (`episodic_memory.py:102-118`).
- **C11 — `record_color` only finalizes a location once per meal.** A second pickup at the
  same location in one meal (e.g. after a redo path) skips re-finalization
  (`preference_session.py:769`), unlike nav offsets which deliberately re-finalize.
  A second user correction at the same location in the same meal would update the YAML
  (used next pickup) but not this meal's learned ground truth.
- **C12 — Two YAML writers, one atomic.** Preference applies use the atomic temp-file +
  `os.replace` writer (`apply_preferences._save_yaml`), but the perception/nav write-back
  path uses `save_behavior_tree`'s plain `open("w")` (`base.py:1084`) — a settings-thread
  reader could in principle observe a partially-written pickup/navigate YAML during a
  write-back. Low likelihood (write-backs happen on the executive thread during a skill,
  when the settings overlay stall usually holds), but the asymmetry is real.
- **C13 — the ask-page display window is not locked.** `ask()` joins the background
  worker before showing each step, so the displayed prediction is always settled — but a
  settings edit made *while the page is on screen* can trigger a reprediction that moves
  the still-open dim being displayed before the user answers, so `changed` is computed
  against the (now stale) value that was shown. Benign consequences (worst case: a
  confirmation is recorded as a correction or vice versa for that dim), and inherent to
  showing a live-updating quantity to a human; the alternative — locking open dims while
  a page is up — would defeat edit-driven propagation.
- **C14 — Deployment day-1 first-ask cold start.** The very first `ask()` happens before
  any memory exists; the correction-page autocontinue is 10 s
  (`_DEFAULT_AUTOCONTINUE_SECONDS`) until `wait_before_autocontinue_seconds` itself is
  finalized — i.e., the wait-preference page uses a wait the user hasn't chosen yet.
  Unavoidable bootstrap, but worth knowing when reading logs.
- **C15 — `PYTHONHASHSEED=0` is asserted at import** (`run.py:176`) — plan/grounding
  determinism depends on it; running the module without it fails immediately.
- **C16 — `_apply_hard_rules` only fires for catalog meals** (`known_meal`); a preset
  `--pref_meal` outside the catalog is rejected earlier by context validation, so this is
  consistent — but any future free-form meal support silently loses the no-dip hard rule
  and `food_items_for_flair` raises `KeyError`.
- **C17 — Perception service brittleness.** The microwave-button detector posts to a
  hardcoded ngrok URL (`appliance_perception.py:76`) and `remote_molmo.py` embeds a
  personal SSH host/path; both are outside the preference pipeline but sit on the same
  meal-critical path (microwave routing chosen by `microwave_time`).
- **C18 — Depth validity window.** `pixel2World` rejects depth outside 0.05–1.0 m
  (`attachment_perception.py:552`), so handle detection silently fails if the robot gazes
  from farther than 1 m — appearing as C1's 20-frame failure.
- **C19 — Confirmation-mode params migrate legacy per-user trees in place.** The
  `AskFor*Confirmation` params are upserted by `apply_bundle_to_behavior_trees`
  (`_upsert_confirm_param`): a per-user tree that predates a param gets it APPENDED — safe
  only because the factory YAMLs and the skill signatures also put these params LAST
  (positional binding), and the skills default the argument to None = pre-change behavior.
  The two feeding params' legacy `Enum [0, 1]` spaces are widened to `[0, 1, 2]` on the
  same pass, otherwise mode 2 would fail `space.contains()` at execution. If you ever add
  a confirmation param NON-last, positional binding breaks silently for migrated trees.
- **C20 — Auto-proceed trades explicit consent for flow.** With
  `confirm_manipulation = "yes (with auto-continue countdown)"`, the plate release and detection pages
  proceed on countdown expiry — the robot releases the plate / acts on a detection the
  user never explicitly approved. That is the user's chosen trade (same class as C5's
  autocontinue-confirms-as-ground-truth caveat). A failed/absent detection still takes the
  normal failure path in every mode — "no" never auto-accepts nothing.
- **C21 — The FLAIR next-bite page is NOT governed by a confirmation dim.** A
  `confirm_bite_selection` dim (feeding family: "no" = silent full-auto feeding per the
  learned bite_ordering, "wait for me" = never auto-pick) was considered and deferred;
  the page keeps its existing always-on autocontinue via the wait pref. Door closing
  (`perceive_handle_closing_poses`) shows no page (cached poses), so `close_*` skills are
  intentionally outside `confirm_manipulation`.

---

## 11. Reading order for newcomers

1. `preference_learning/config/preference_bundle.py` — the vocabulary of the whole system.
2. `integration/preference_session.py` — the per-meal state machine (start / ask /
   record_color / record_nav_offset / finalize; the file docstring is accurate).
3. `integration/run.py` (`_start_preference_session`, `_run_meal_preparation`,
   `process_user_command`) — when each stage happens.
4. `methods/prediction_model.py` + the two prompt `.txt` files — what the LLM actually
   sees.
5. `integration/apply_preferences.py` — how a bundle becomes robot behavior.
6. `actions/navigate.py` (`_offer_position_adjustment`) and
   `interfaces/perception_interface.py` (`perceive_attachment_poses`,
   `_interactive_color_correction`) — the two physical correction channels.
