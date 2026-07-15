# Deployment Logging & Metrics Plan

What to log during the month-long deployment, mapped to the paper's claims
("Table for Two", `docs/feeding-deployment-docs`). Status against the current
codebase: **HAVE** (logged today), **PARTIAL** (derivable with effort or
incomplete), **GAP** (not captured — needs a change before day 1).

Guiding rule: **every number, figure, and claim in the paper must be
computable from logs alone.** The single most important item in this plan is
the dry run at the end.

---

## 1. Preference learning efficiency (§Measures; real-world analog of Table II)

| Metric | Source | Status |
|---|---|---|
| Corrections-to-converge per meal (m*) | `events.jsonl`: `preference_asked.corrected` + `preference_finalized.corrected` | **HAVE** |
| First-prediction accuracy per day (per dim + aggregate) | `preference_predicted.predicted_bundle` vs `preference_finalized.ground_truth` | **HAVE** |
| Per-dimension learning curves over days | same | **HAVE** |
| Color / nav-offset channel usage + adjustment magnitudes | `preference_color_recorded`, `preference_nav_offset_recorded` (`changed` flag), `nav_offset_log.jsonl` | **HAVE** |
| **Correction propagation** (the correlated-bundle hypothesis, in the wild): after a correction, which open dims moved and were later accepted | only the initial `preference_predicted` (stage=`start`) is logged; repredictions are not | **GAP** — log every reprediction round: `preference_repredicted {trigger_field, changed_open_dims, new_values}` in `preference_session._repredict_worker` |
| Engaged confirm vs autocontinue timeout | webapp returns identical response either way | **GAP (BLOCKER)** — add `user_action: "tap" \| "autocontinue"` to `preference_correction_response` (and to bite/confirmation pages) in the Vue views + log it |
| Page-shown → response latency | `user_inputs.jsonl` has response time only; sent messages are un-timestamped plaintext | **GAP (BLOCKER)** — make `_send_message` also write a structured `page_shown {state, payload_kind, epoch}` record (upgrade `webapp_sent_messages.txt` → jsonl with epoch) |
| LLM call health: latency, speed served, effort, validation fallbacks | `prediction_model_llm_calls/*.txt` has model/speed headers; **silent first-option fallback (malformed JSON) is not evented** | **PARTIAL** — add `preference_prediction_fallback` event (safety-relevant: a fallback bundle gets physically actuated) + call duration in the txt header |

## 2. Co-adaptation, user side (the title claim)

The key derived measure: split every human input into **forced participation**
(burden: corrections, failure-triggered teleop, redos) vs **chosen
participation** (bite choices, initiation gating, voluntary teleop, settings
exploration). Success = forced ↓ over days while chosen persists.

| Metric | Source | Status |
|---|---|---|
| Teleop sessions: provenance (user-initiated idle / task-menu / mid-skill), duration, per-command traces | `teleop_intervention_log.jsonl` | **HAVE** |
| Teleop: *which HLA was interrupted* + why (skill failure vs proactive) + post-teleop outcome | `session_id` null, no HLA back-reference | **GAP** — pass `interrupted_hla`, `failure_reason` (if any), and log post-teleop `redo/next` outcome linked to the skill event |
| Proactive repositioning signature (takeover *before* skill start vs after failure) | derivable once skill events (§3) + teleop HLA context exist | **PARTIAL** |
| Settings overlay engagement (opens, dismiss-without-edit, edits) | only successful edits logged (`preference_settings_edit`) | **GAP** — log `settings_opened` / `settings_closed {edited: bool}` |
| Autocontinue timeout rate over days (attention/trust) | blocked by the tap-vs-timeout flag | **GAP (BLOCKER #1)** |
| Response latency trends over days | blocked by page-shown timestamps | **GAP (BLOCKER #2)** |
| Trust trajectory via confirmation dims (`confirm_*` relaxing over days) | daily `preference_finalized` bundles | **HAVE** |
| Bite choice behavior (accept autonomous / pick other / manual keypoint; dip overrides) | no per-bite structured event; only cumulative `flair_history.txt` | **GAP** — `bite_event {predicted_item, chosen_item, source: autonomous\|user_selected\|manual, dip_predicted, dip_chosen, image_ref, acquisition_outcome}` |
| Task cadence (bite/sip/wipe rhythm, transparency queries) | `task_command` events | **HAVE** |
| Questions directed at researcher vs system | not capturable in software | **GAP** — researcher diary (§5) |

## 3. Skill success rates & system performance (§Measures)

| Metric | Source | Status |
|---|---|---|
| Per-skill outcome: name, start/end, success/failure, failure reason, retries, recovery path | only implicit (BT plaintext trace, exceptions → teleop log) | **GAP (HIGH)** — add `skill_execute` event in the run.py skill loop: `{hla, plan_step, t_start, t_end, outcome: success\|failure\|teleop_recovered\|aborted, failure_reason, n_retries}` |
| Detection retries / no-detection fallbacks (color picker invocations, bite no-detection page) | counted in code, not persisted | **GAP** — event when the color picker or no-detection bite page is entered |
| Meal-level: total duration, per-phase durations (prep pipeline vs feeding), bites/sips/wipes per meal | `metadata.json` start/end + task_commands; phases derivable once `skill_execute` exists | **PARTIAL** |
| Researcher interventions (hardware/software/skill/safety) | manual | **GAP** — researcher diary (§5) with category enum matching the paper |
| Nav health: goals, residuals, holds, recovery usage, anomaly bags | `nav_diag_logger` (`system_logs/navlog_*`), `goals.csv`, `events.csv` | **HAVE** |
| Sensor/compute health: topic rates, dropouts, GPU/RAM | `zed_health_monitor`, `compute_health_monitor` | **PARTIAL** — compute monitor writes to stdout/syslog only; redirect to a file under `log/` |
| Diag logs ↔ meal linkage | separate log trees, no meal id | **GAP (small)** — write `{user, day}` markers into `system_logs` runs (or log the navlog dir name into `events.jsonl` at meal start) |

## 4. Memory / drift events (lessons learned + longitudinal analyses)

- **Environment drift**: plate changed, furniture moved, lighting changed,
  appliance behavior changed. Automatic signals exist (`changed=True` on
  color/nav records after a stable period) but causes don't — record drift
  events in the researcher diary with a timestamp so staleness incidents can
  be attributed. **GAP (manual)**
- **Memory staleness incidents**: stored color fails detection → picker;
  stored nav offset wrong → large adjustment. Derivable once picker-entry
  events exist (§3) + `nav_offset_log.jsonl` magnitudes. **PARTIAL**
- **Withdrawal probe** (if adopted): mark memory-off meals in
  `metadata.json` (`{"probe": "memory_off"}`) so they're excluded/analyzed
  separately. **GAP (protocol decision)**

## 5. Human-administered instruments (make them digital + consistent)

- **Daily survey** (safety, satisfaction Likert + open-ended adaptation):
  record as `daily_survey.jsonl` per day — not paper notes. Template:
  `{day, epoch, safety: 1-5, satisfaction: 1-5, adaptation_text, notes}`.
- **Recommended addition (1 item, pre-meal)**: a single energy/state check-in
  ("How are you feeling today?" 1–5 or hurried/neutral/tired). This makes the
  latent affective state Y_t from the formulation *partially observed* in
  deployment — it's the only way to later correlate preference/agency shifts
  with state. Requires CR consent; near-zero burden.
- **Researcher diary** `researcher_diary.jsonl`, one entry per event:
  `{epoch, day, category: hardware|software|skill|safety|question|drift|note,
  skill: <hla or null>, text}`. Fill in live during the meal, not from memory
  afterwards. This is the source for the interventions table and the
  questions-to-researcher count.
- **End-of-study instruments** (TAM, control/independence, co-adaptation):
  unchanged from the draft; store as JSON alongside.

## 6. Media & heavyweight data

- **External video**: one fixed-angle recording per meal (day-stamped),
  plus a weekly matched-angle recording of the same subtask (e.g., fridge
  retrieval + one bite) for day-1-vs-day-20 comparisons. Manual checklist item.
- **Images**: verify each perception skill actually calls `log_image()`
  (detection overlays, plate images per bite). Bite images should be
  referenced from the new `bite_event`.
- **Selective rosbags during arm skills** (optional but valuable for
  follow-on learning work): joint states + FT + compressed wrist camera
  during `acquire_bite`/`transfer_*` only. Teleop command traces are already
  captured in `teleop_intervention_log`; this upgrades interventions from
  "events" to "demonstrations". Decide based on disk budget; keep off the
  robot WiFi during runs (teardown-only collection).

## 7. Hygiene

- All timestamps originate robot-side (single clock) — keep it that way; if
  the webapp ever stamps client-side, sync iPad clock via NTP first.
- Nightly teardown checklist: confirm `metadata.json` has `closed: true`,
  rsync the day bundle + `system_logs` off-robot, verify sizes nonzero.
- Git-tag the repo at each deployment day start (`deploy-day-NN`) so code
  state is reconstructable per meal.

---

## Priority fixes before day 1 (small diffs, in order)

1. **`user_action: tap|autocontinue` flag** on every autocontinue-capable page
   (preference correction, confirmations, bite selection) — webapp + logging.
2. **Structured `page_shown` records with epoch** (replace plaintext
   `webapp_sent_messages.txt`).
3. **`skill_execute` outcome events** in the run.py skill loop.
4. **Reprediction logging** (`preference_repredicted` with trigger field).
5. **Teleop context**: interrupted HLA + failure reason + post-teleop outcome.
6. **Per-bite `bite_event`**.
7. **Settings overlay open/close events.**
8. **LLM fallback events** (malformed-JSON → first-option bundle actuated).
9. Diary + daily-survey JSONL templates; diag-log ↔ meal linkage; compute
   monitor → file.
10. Protocol additions needing CR consent: pre-meal state item; withdrawal
    probe; per-meal video.

## The dry run (do this before day 1)

Run one full lab meal, then write the analysis notebook that computes **every
paper number from the logs alone**: corrections-to-converge, first-prediction
accuracy, per-dim curves, propagation events, tap-vs-timeout rates, latency
distributions, forced-vs-chosen participation split, skill success table,
intervention table, phase durations. Any number the notebook can't produce is
a logging gap you found for the price of one lab meal instead of the
deployment. Keep the notebook in the repo; it becomes the paper's analysis
pipeline.
