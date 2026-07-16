"""End-of-meal survey: question wording + orchestration.

After Finish Feeding places the plate in the sink (and the per-day preference
memory is finalized), the executive walks the user through these questions on
the webapp's /survey page, one at a time, then parks the iPad on the terminal
/thank_you page -- the meal deliberately does NOT return to task selection.
Transport lives in WebInterface (start_survey / send_survey_question /
finish_survey); this module owns the wording and the logging.

Note: the survey is skipped when run.py resumes a crashed session with
_resume_phase == "finish" (that path goes straight to task selection).
"""

from typing import Any

from feeding_deployment.interfaces.web_interface import WebInterfaceTakeoverInterrupt

SURVEY_SCALE = {
    "scale_min": 1,
    "scale_max": 7,
    "min_label": "Very Low",
    "max_label": "Very High",
}

# NASA-TLX-style workload items plus trust/safety, then one open-ended
# reflection. kind "likert" renders a 1-7 button row on the page; "text"
# renders a free-text box with voice input.
SURVEY_QUESTIONS: list[dict[str, str]] = [
    {"key": "mental_demand", "title": "Mental Demand", "kind": "likert",
     "question": "How mentally demanding was using the meal-assistance system?"},
    {"key": "physical_demand", "title": "Physical Demand", "kind": "likert",
     "question": "How physically demanding was using the meal-assistance system?"},
    {"key": "temporal_demand", "title": "Temporal Demand", "kind": "likert",
     "question": "How much time pressure did you feel when using the meal-assistance system?"},
    {"key": "performance", "title": "Performance", "kind": "likert",
     "question": "How successful were you in using the meal-assistance system to achieve your goals?"},
    {"key": "effort", "title": "Effort", "kind": "likert",
     "question": "How hard did you have to work to use the meal-assistance system effectively?"},
    {"key": "frustration", "title": "Frustration", "kind": "likert",
     "question": "How frustrated were you while using the meal-assistance system?"},
    {"key": "trust", "title": "Trust", "kind": "likert",
     "question": "How much did you trust the robot to do the right thing during today's meal?"},
    {"key": "safety", "title": "Safety", "kind": "likert",
     "question": "How safe did you feel during today's meal?"},
    {"key": "adaptation", "title": "Adaptation", "kind": "text",
     "question": "What, if anything, did you learn about the robot today, "
                 "or how did your interaction with it change?"},
]


def _coerce_value(kind: str, raw: Any) -> Any:
    """Sanitize a webapp answer: likert -> int within the scale (else None),
    text -> stripped string (may be empty: the question says 'if anything')."""
    if kind == "text":
        return "" if raw is None else str(raw).strip()
    try:
        value = int(raw)
    except (TypeError, ValueError):
        return None
    if not SURVEY_SCALE["scale_min"] <= value <= SURVEY_SCALE["scale_max"]:
        return None
    return value


def run_end_of_meal_survey(web_interface, data_logger) -> dict[str, Any]:
    """Run the survey on the webapp and return {question key: answer}.

    Blocks on each question (mandatory answers, no timeout -- same posture as
    wait_for_start_meal). Two ways out besides answering: a Take-Over press
    (partial result returned; the takeover_event is left set for the idle
    loop's consume_takeover to launch teleop) and the webapp jumping to task
    selection (partial result returned). In both cases the thank_you jump is
    skipped so we don't yank the page the user just escaped to.
    """
    total = len(SURVEY_QUESTIONS)
    responses: dict[str, Any] = {}
    user_actions: dict[str, str] = {}
    aborted = None
    try:
        web_interface.start_survey(total=total)
        for step, q in enumerate(SURVEY_QUESTIONS):
            msg_dict = web_interface.send_survey_question(
                field=q["key"], title=q["title"], question=q["question"],
                kind=q["kind"], step=step, total=total, **SURVEY_SCALE)
            if msg_dict is None:
                # The webapp jumped to task_selection while we were blocked --
                # the user (or operator) force-left the survey.
                aborted = "task_selection_jump"
                data_logger.log_event("survey_response", field=q["key"],
                                      value=None, step=step, total=total,
                                      user_action="jump")
                break
            value = _coerce_value(q["kind"], msg_dict.get("value"))
            action = msg_dict.get("user_action", "tap")
            responses[q["key"]] = value
            user_actions[q["key"]] = action
            data_logger.log_event("survey_response", field=q["key"], value=value,
                                  step=step, total=total, user_action=action)
        if aborted is None:
            web_interface.finish_survey()
    except WebInterfaceTakeoverInterrupt:
        aborted = "takeover"
        data_logger.log_event("survey_aborted", reason="takeover",
                              answered=sorted(responses))
    data_logger.log_event(
        "survey",
        responses=responses,
        user_actions=user_actions,
        completed=aborted is None and len(responses) == total,
        aborted=aborted,
    )
    return responses
