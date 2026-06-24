# Staged Personalization Pipeline — Change Log & Robot-Testing Handoff

This document describes a feature change to the feeding-deployment personalization
system. It is written to be **self-contained** so an engineer (or a fresh AI agent
on the robot, without the original design conversation) can understand the design,
locate every change, and debug issues during on-robot testing.

> **Status:** implemented, off-robot tests green (107 passed). **Not yet verified on
> the robot.** The simulated perception path may be broken, so the robot is the real
> verification. Build confidence off-robot (tests) before relying on a sim run.

---

## 1. What changed and why

### Before
Personalization ran **entirely up front** in `run.py`: collect context → predict the
full preference bundle → show **one** correction page with *all* dimensions at once →
apply everything → one memory update → then the meal began (the first "bite" command
planned the whole fridge→microwave→table→feed sequence in one shot).

Plate-handle **color** for attachment detection was **not** a preference dimension —
it was a continuous HSV value corrected ad-hoc at each pickup and stored in per-object
behavior-tree (BT) YAML.

### After (this change)
A **staged, just-in-time** pipeline that mirrors the physical meal and repeats over
~30 days:

1. User inputs observable **context** (meal / setting / time), then the meal starts.
2. Robot predicts the **full bundle** over all dimensions.
3. Preferences are **asked only when relevant** to the current stage, not all up front.
4. **Every correction repredicts** the still-open dims as a bundle; already-decided
   dims stay **pinned**.
5. **A non-correction is also signal** — it finalizes that dim as ground truth (the
   prediction was right).
6. **Color is now a first-class preference dimension** (three per-location color dims),
   predicted by the LLM, seeded from the per-user BT YAML, learned in LTM/EM like every
   other dimension.

### Stage → dimension map

| Stage (physical) | Where it's driven | Dimensions asked |
|---|---|---|
| **Start** (after context) | `run.py::_start_preference_session` | `robot_speed`, `wait_before_autocontinue_seconds` |
| **Fridge pickup** | color hook in `process_user_command` | `plate_color_fridge` |
| **Plate on holder → before microwave** | `run.py::_run_meal_preparation` | `microwave_time` |
| **Microwave pickup** (if heated) | color hook in `process_user_command` | `plate_color_microwave` |
| **Table, before feeding** | `run.py::_run_meal_preparation` | all feeding/drinking/wiping + transfer dims (see `_TABLE_PREF_DIMS`) |
| **End table pickup** (finish) | color hook in `process_user_command` | `plate_color_table` |
| **Meal end** | `run.py::_finalize_preference_session` | one LTM/EM memory update with the full finalized bundle |

The finalized `wait_before_autocontinue_seconds` drives the **autocontinue timeout of
every subsequent correction page** (replacing a hardcoded 10s).

`occlusion_relevance` was removed in an earlier commit (`09621d48`) and is not part of
this change.

---

## 2. Key design decisions (so you don't "fix" them by accident)

- **Microwave routing is planner-driven, not scripted.** `microwave_time` is a single
  dim asked once when the plate is on the holder. "no microwave" adds the `FoodHeated`
  atom up front; any duration leaves it unset. `PlacePlateOnTableHLA` now requires
  `FoodHeated`, so planning-to-preconditions inserts the microwave detour iff
  `FoodHeated` isn't already true. (`AcquireBiteWithTool` already required `FoodHeated`
  — that was the original microwave trigger; we moved the decision earlier so it's a
  staged question, not a side effect of the first bite.)
- **Plate starts in the fridge** for staged mode (`_pref_mode != "none"`); legacy
  `none` mode keeps the plate on the holder. The meal begins with the robot retrieving
  the plate (and the fridge color correction).
- **Color values never go on the stepwise correction page.** They use the existing
  `color_correction.vue` canvas picker during pickup. The session reads the
  (possibly-corrected) color back from the BT YAML *after* the pickup HLA runs.
- **Color is continuous HSV but predicted by the LLM.** The canonical value is a dict
  `{"h":0-179, "s":0-255, "v":0-255, "range":0.0-1.0}`. Day 1 the seed is the factory
  default `{82,55,84, range 0.1}`; later days it's the last saved value in the
  per-user BT YAML (`log/<user>/behavior_trees/pick_plate_from_*.yaml`), which persists
  across days. On any LLM parse failure, the prediction **falls back to the seed**.
- **Same-plate propagation happens via the LLM**, not a hand rule: the three
  `plate_color_*` dims are described as the same physical handle under different
  lighting, so correcting one is evidence for the others. A color correction repredicts
  the still-open dims.
- **Memory is written exactly once per day**, at `finalize_meal`. Mid-meal repredictions
  only *read* (`predict_bundle`), never write, so LTM/EM aren't polluted with partial
  bundles. Confirmed dims (user accepted the prediction) are in the ground-truth bundle,
  so "no correction is information" is learned.
- **Locking semantics:** both corrected and confirmed dims go into `session.finalized`
  and are passed as the pin/override dict to `predict_bundle`, so a later correction
  never flips a dim the user already saw and we already executed. The
  corrected-vs-confirmed distinction is kept only for the learning signal and logs.

---

## 3. Files changed

### New files
- **`src/feeding_deployment/integration/preference_session.py`** — `PreferenceSession`,
  the per-meal orchestrator. Owns the prediction model, the live `bundle`, the
  `finalized`/`corrected` sets, and the color BT-YAML I/O. Public methods:
  `start()`, `ask(dims)`, `apply_microwave(current_atoms, food_heated_atom)`,
  `record_color(location)`, `finalize_meal(day)`. Degrades gracefully when
  `web_interface`/`scene_description`/`flair` are `None` (used by unit tests).
- **`tests/test_preference_session.py`** — 9 tests; fakes the prediction model and web
  interface, uses a temp BT dir. Covers prediction, confirm-vs-correct + reprediction
  pinning, color seed + fallback, color record (corrected & confirmed), wait→autocontinue,
  one-update-per-day, and "repredictions don't write memory".

### Preference model / config
- **`preference_learning/config/preference_bundle.py`**
  - `PreferenceDim` gained `kind: str = "categorical"`.
  - Added 3 dims: `plate_color_fridge`, `plate_color_microwave`, `plate_color_table`
    (`kind="color"`, empty `options`).
  - Added color helpers: `COLOR_FIELDS`, `DEFAULT_COLOR`, `COLOR_FIELD_BY_LOCATION`,
    `parse_color`, `format_color`, `color_to_bt`, `color_from_bt`. These are the single
    source of truth for the canonical color dict and its BT-YAML / episode-text encodings.
- **`preference_learning/methods/prediction_model.py`**
  - `predict_bundle(context, corrected, color_seeds=None)` — new `color_seeds` arg.
  - Per-kind validation: categorical → must be an allowed option (fallback to
    corrected/default); color → `parse_color` with **seed fallback**.
  - Color corrections are canonicalized; final categorical-only validation.
  - Added `PREF_KIND` map. Options/corrected prompt blocks render color lines with the seed.
- **`preference_learning/methods/utils.py`** — `_episode_text` / `_extract_truth_bundle`
  serialize color values via `format_color` (stable `h=..,s=..,v=..,range=..` string) so
  LTM/EM ingest them. Other fields unchanged.
- **`preference_learning/methods/prompts/bundle_prediction.txt`** — describes color dims,
  the HSV object output shape, seeds, and the same-plate hint; removed the hardcoded
  "exactly 18 keys".
- **`preference_learning/data_generation/prompts/system_description.py`** —
  `render_preference_dimensions` renders color dims with their HSV format instead of an
  empty "Allowed options" line.

### Backend orchestration
- **`integration/run.py`** (largest change) —
  - New constants `_INITIAL_PREF_DIMS`, `_TABLE_PREF_DIMS`.
  - New methods: `_collect_preference_context`, `_start_preference_session`,
    `_run_meal_preparation`, `_finalize_preference_session`.
  - `run()` replaced the up-front predict/correct/apply/learn block with
    `_start_preference_session()` (which predicts, asks initial dims, then runs the
    staged meal prep). The old "Step 1–5" block is gone.
  - `process_user_command` execution loop: **color hook** — after a `pick_plate_from_*`
    HLA runs, calls `session.record_color(location)` for fridge/microwave/table.
  - `finish_feeding` task now calls `_finalize_preference_session()` after the plate is
    placed in the sink (the table color correction happens during that command).
  - Initial state: `PlateAt(fridge)` for staged mode, `PlateAt(holder)` for `none`.
  - `main()`: `--pref_meal` is now **optional** for `interface` mode (web context page
    collects it if not preset).
- **`integration/terminal_preferences.py`** — added `TerminalCorrectionInterface`
  (stdin implementation of the stepwise contract) so `--pref_mode=terminal` reuses the
  same staged session via the console.
- **`actions/place_plate.py`** — `PlacePlateOnTableHLA` gained a `FoodHeated`
  precondition (the planner-driven microwave routing). Imported `FoodHeated` from base.
- **`integration/apply_preferences.py`** — added `gaze_at_table.yaml` to `_ALL_BT_YAMLS`
  so `robot_speed` propagates to the gaze action (was a pre-existing gap; also fixes a
  pre-existing failing test). **Note:** the bundle→BT mapping is field-scoped, so the
  apply helpers are safe to call repeatedly (idempotent).

### Web interface backend
- **`interfaces/web_interface.py`**
  - Replaced the old monolithic `get_preference_corrections` with **stepwise** primitives:
    `start_preference_correction(total, autocontinue_seconds)`,
    `send_preference_step(field, predicted, options, step, total, autocontinue_seconds) -> value`,
    `finish_preference_correction()`. The session drives the loop and repredicts between steps.
  - Added `get_meal_context(meals, settings, times_of_day, defaults)` to drive the
    `preference_context` page (consume the response the page already sent but nobody read).
  - Added `_PREF_LABELS` (field → human label, from the bundle config).

### Frontend (Vue)
- **`webapp/src/views/preference_correction.vue`** — rewritten for the stepwise protocol:
  one dim per backend message, confirm-and-wait-for-next within a single page mount,
  autocontinue read from `autocontinue_seconds` in the message (no hardcoded 10s).
  URL-query fallback supports single-dim dev testing.
- **`webapp/src/views/preference_context.vue`** — added a `ready` handshake on connection
  (mirrors the correction page) so the backend's non-latched options can't race the
  page's subscription.

### Tests
- **`tests/test_preference_integration.py`** — updated `_default_bundle` to emit HSV dicts
  for color dims; added `_assert_valid_value` (per-kind); scoped the "≥2 options" check to
  categorical dims. (These were necessary because color dims have empty option lists.)

---

## 4. Message contracts (frontend ⇄ backend)

All messages are JSON on ROS topics `/robot_to_webapp` (backend→app) and
`/webapp_to_robot` (app→backend), `std_msgs/String`.

### Context page (`preference_context`)
```
BE→app: {"state":"preference_context","status":"jump"}
app→BE: {"state":"preference_context","status":"ready"}            # on (re)connect
BE→app: {"state":"preference_context_data","meals":[...],"settings":[...],
         "time_of_day":[...],"defaults":{...}}
app→BE: {"state":"preference_context_response","meal":..,"setting":..,"time_of_day":..}
```

### Correction page (`preference_correction`) — STEPWISE, one dim at a time
```
BE→app: {"state":"preference_correction","status":"jump","total":M,"autocontinue_seconds":N}
app→BE: {"state":"preference_correction","status":"ready"}         # on (re)connect
# then, per dimension:
BE→app: {"state":"preference_correction_data","field":..,"label":..,"predicted":..,
         "options":[...],"step":i,"total":M,"autocontinue_seconds":N}
app→BE: {"state":"preference_correction_response","field":..,"value":..}
# after the last dim of the stage:
BE→app: {"state":"preference_correction","status":"done"}          # app routes to /robot_executing
```

### Color correction (`color_correction`) — UNCHANGED
The existing detection-confirm / canvas color picker flow is reused as-is. The session
does not send these; the pickup HLA does. The session only reads the resulting color back
from the BT YAML.

---

## 5. Runtime flow (staged mode, `--pref_mode interface`)

```
run()
 └─ _start_preference_session()
     ├─ _collect_preference_context()         # web preference_context page (or --pref_meal preset / terminal)
     ├─ build PredictionModel + PreferenceSession
     ├─ session.start()                        # predict FULL bundle, write predicted colors to BT YAML, apply non-planning dims
     ├─ session.ask(["robot_speed","wait_before_autocontinue_seconds"])
     └─ _run_meal_preparation()
         ├─ process_user_command(PlacePlateOnHolder)   # fridge→holder; fridge color hook fires
         ├─ session.ask(["microwave_time"])
         ├─ session.apply_microwave(current_atoms, FoodHeated)   # sets/leaves FoodHeated
         ├─ process_user_command(PlacePlateOnTable)    # planner routes via FoodHeated; microwave color hook fires if heated
         └─ session.ask(_TABLE_PREF_DIMS)
 └─ ready_for_task_selection()                 # user issues bite/sip/wipe (meal_assistance)
 ...
 finish_feeding:
   process_user_command(PlacePlateInSink)      # picks from table first → table color hook fires
   _finalize_preference_session()              # ONE PredictionModel.update(day, ...)
```

Color hook (in `process_user_command`, after each HLA executes): if the HLA's BT file is
`pick_plate_from_{fridge,microwave,table}`, call `session.record_color(location)`.

---

## 5b. Frontend page sequence (which page shows, and when)

**Routing mechanism (important):** there is **no global navigator in `App.vue`**. Each
currently-displayed page subscribes to `/robot_to_webapp` and, in its own
`handleRosMessage`, navigates via `routeMap[state][status]`. So the page shown *between*
robot stages (`/robot_executing`) is load-bearing: it must forward the next `jump` to the
correct page. Verified that `robot_executing.vue`, `detection_confirm.vue`, and
`color_correction.vue` all do this, and that every `{state,status}` the backend emits is
present in `webapp/src/router/routeMap.js`.

| # | Robot stage | Page shown | Trigger |
|---|---|---|---|
| 1 | Meal start | **preference_context** | `web_interface.get_meal_context()` sends `preference_context/jump`; on submit the page pushes `/robot_executing` |
| 2 | Before moving | **preference_correction** (`robot_speed`, `wait_before_autocontinue_seconds`) | `session.ask(_INITIAL_PREF_DIMS)`; stepwise; `…/done` → page pushes `/robot_executing` |
| 3 | Fetching plate from fridge | **detection_confirm → color_correction** | existing perception flow inside the `PlacePlateOnHolder` fridge pickup |
| 4 | At microwave (plate on holder) | **preference_correction** (`microwave_time`) | `session.ask(["microwave_time"])` |
| 5 | Microwave pickup (only if heated) | **detection_confirm → color_correction** | perception flow inside the microwave pickup |
| 6 | At table, before feeding | **preference_correction** (all feeding/drinking/wiping + transfer dims) | `session.ask(_TABLE_PREF_DIMS)` |
| 7 | Feeding | **task_selection** + bite/drink/wipe confirm pages | existing `meal_assistance` flow |
| 8 | Finish → table pickup | **detection_confirm → color_correction** | perception flow inside `PlacePlateInSink`'s table pickup |

Notes:
- After **every** correction (including each color correction) the still-open dims are
  repredicted; exactly one memory write happens at meal end (step after 8).
- The **color correction pages are unchanged** and are driven by the existing perception
  interface during each pickup, NOT by the session. The session only reads the resulting
  color back from the BT YAML afterward (the color hook in `process_user_command`).
- **"Initiate the meal" = submitting the context page.** There is no separate "start
  meal" screen between context and prediction; confirming context is the initiation,
  after which the robot predicts and begins fetching. (If a distinct confirmation screen
  is wanted there, it's a small addition — not currently implemented.)
- Between stages the iPad sits on `/robot_executing`; the backend's next `…/jump` moves
  it to the next page. If the iPad ever appears "stuck" on `/robot_executing`, the
  backend is still working (or blocked) and hasn't sent the next jump yet.

---

## 6. Known risks / what to watch on the robot

- **Planner reachability of `FoodHeated`.** Verify the planner can achieve `FoodHeated`
  from `PlateAt(holder)`: PickPlateFromHolder → Navigate(microwave) → OpenMicrowave →
  PlacePlateInMicrowave → CloseMicrowave → PressMicrowaveButton (adds FoodHeated) →
  OpenMicrowave → PickPlateFromMicrowave → ... → Navigate(table) → PlacePlateOnTable.
  And for "no microwave": FoodHeated set up front → PickPlateFromHolder → table directly.
  **If `PlacePlateOnTable` planning fails**, suspect the new `FoodHeated` precondition
  (`actions/place_plate.py`) or a missing intermediate operator/atom (e.g. `TableSeen`).
- **Stepwise correction page race.** The backend waits for the page's `ready` before
  sending each stage. If a stage hangs, confirm `preference_correction.vue` emits
  `{state:preference_correction,status:ready}` on connection and that the per-dim
  `field` in the response matches the requested field (the backend filters on it).
- **Context page collection.** `get_meal_context` requires the new `ready` handshake in
  `preference_context.vue`. If it hangs, that's the first thing to check. Fallback:
  pass `--pref_meal/--pref_setting/--pref_time_of_day` to skip the web page.
- **Color round-trip.** The session writes predicted colors into
  `log/<user>/behavior_trees/pick_plate_from_*.yaml` (`HandleColor`=[H,S,V],
  `ColorRange`=float) before each pickup, and reads them back after. If colors don't
  persist across days, check that the per-user BT dir isn't being reset and that the
  pickup HLA writes corrections back (it already did before this change:
  `actions/pick_plate.py`).
- **Autocontinue seconds.** Driven by the finalized `wait_before_autocontinue_seconds`
  ("10 sec"→10, "100 sec"→100, "1000 sec"→1000). Before that dim is finalized, pages use
  10s. 1000s is intentionally long per the preference semantics.
- **`apply_microwave` timing.** It mutates `current_atoms` and **must** run after the
  microwave ask and **before** the `PlacePlateOnTable` command. It's wired that way in
  `_run_meal_preparation`; don't reorder.
- **Memory writes.** There must be exactly **one** `PredictionModel.update` per meal
  (at finish). If you see per-stage memory writes, something is calling `finalize_meal`
  early or `update` directly.

---

## 7. How to verify

### Off-robot (no hardware, no real LLM) — already green
```
PYTHONPATH=src python -m pytest tests/test_preference_session.py tests/test_preference_integration.py tests/test_apply_preferences.py -q
```
Requires `openai` importable (it's a transitive import; only the client is constructed,
never called, in these tests). 107 tests pass.

### Planner routing (offline, needs the PDDL planner, no perception)
Drive the planner directly: with `FoodHeated` unset, `PlacePlateOnTable` should plan
through the microwave; after adding `FoodHeated`, it should plan holder→table directly.

### On-robot (the real check)
One full meal: context → speed/wait asks → fridge retrieval + fridge color → microwave
ask → [microwave route + microwave color | direct] → table → table dims → eat → finish
(table color) → one episode written to `log/<user>/preference_learning/.../day_NNNN.json`.
Then a second "day" (`--pref_day N`) to confirm day-2 predictions reflect day-1
corrections (LTM/EM) and corrected plate colors carry over via the BT YAML.

---

## 8. Notes for a fresh agent debugging on the robot

- The **canonical color dict** and all its encodings live in
  `preference_learning/config/preference_bundle.py` (`parse_color`, `color_to_bt`,
  `color_from_bt`, `format_color`). Use these; don't re-derive HSV handling.
- `PreferenceSession` is deliberately decoupled: it takes the prediction model, the BT
  dir, and optional `web_interface`/`scene_description`/`hla_map`/`flair`. If something
  in apply or correction breaks, you can reproduce most logic with fakes (see
  `tests/test_preference_session.py`).
- The session's private helpers `_loggable_bundle`, `_color_seeds`, `_repredict_open`,
  `_apply_non_planning` are the seams to add logging when debugging.
- This repo's convention (from prior sessions): prefer driving sequencing through the
  **PDDL planner + atom state** rather than hardcoding stage order, and treat the
  **robot as the real verification** (sim perception may be broken).

---

## 9. Files at a glance

```
NEW  src/feeding_deployment/integration/preference_session.py
NEW  tests/test_preference_session.py
MOD  src/feeding_deployment/integration/run.py                      (staged orchestration)
MOD  src/feeding_deployment/integration/terminal_preferences.py     (TerminalCorrectionInterface)
MOD  src/feeding_deployment/interfaces/web_interface.py             (stepwise + context collection)
MOD  src/feeding_deployment/actions/place_plate.py                  (FoodHeated precondition)
MOD  src/feeding_deployment/integration/apply_preferences.py        (gaze_at_table speed)
MOD  src/feeding_deployment/preference_learning/config/preference_bundle.py   (color dims + helpers)
MOD  src/feeding_deployment/preference_learning/methods/prediction_model.py   (color prediction)
MOD  src/feeding_deployment/preference_learning/methods/utils.py              (color serialization)
MOD  src/feeding_deployment/preference_learning/methods/prompts/bundle_prediction.txt
MOD  src/feeding_deployment/preference_learning/data_generation/prompts/system_description.py
MOD  webapp/src/views/preference_correction.vue                     (stepwise UI)
MOD  webapp/src/views/preference_context.vue                        (ready handshake)
MOD  tests/test_preference_integration.py                           (color-aware)
```
