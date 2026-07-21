"""Pre-meal latent questionnaire: question wording + orchestration.

Before the meal starts -- after the meal context is collected, but BEFORE the
robot predicts anything -- the executive asks the user to self-report the four
system-invariant latent traits (pace, trust, proximity, communication) on the
webapp. These answers are HELD OUT: they are written to pre_meal.jsonl only and
are NEVER fed into the preference-prediction LLM or the preference-learning
memory. They exist solely to validate, offline, the model's own inferred
``latent_scores`` against the user's self-report.

The questions are context-free trait questions (illustrative examples inline,
no meal/setting substituted), so the page can run regardless of context.

Transport reuses the generic survey page (WebInterface.start_survey /
send_survey_question) with pre-meal header text and a 3-option "choice" kind for
the signalling-modality question. Unlike the end-of-meal survey, this does NOT
park the iPad on the thank-you page -- it returns control so the meal proceeds
to prediction.
"""

from typing import Any

from feeding_deployment.interfaces.web_interface import WebInterfaceTakeoverInterrupt

PRE_MEAL_SCALE = {"scale_min": 1, "scale_max": 7}
_PRE_MEAL_SUBTITLE = "A few quick questions before we start"
_PRE_MEAL_EYEBROW = "Before We Start"
# Held-out reassurance shown on every question: these answers do not feed the
# robot's predictions (which happen after, from the meal's corrections); they
# are only the yardstick used offline to check those predictions.
_PRE_MEAL_NOTE = ("The robot won't be able to see these answers — they only check "
                  "how well it understood you from your corrections during the meal.")

# The four latent traits, context-free / trait-level. Each is a "likert" item
# scored 1-5 and graded offline against the model's latent_scores. Field keys
# pace/trust/proximity/communication match the model's latent_scores keys so the
# offline join is trivial.
PRE_MEAL_QUESTIONS: list[dict[str, Any]] = [
    {"key": "pace", "title": "Pace", "kind": "likert",
     "min_label": "Slow & unhurried", "max_label": "Quick & efficient",
     "question": "How fast-paced do you want the meal to feel today — quick and "
                 "efficient, or slow and unhurried?"},
    {"key": "trust", "title": "Trust", "kind": "likert",
     "min_label": "Check with me", "max_label": "Trust it",
     "question": "How much do you want the robot to check with you before and "
                 "after it does things, versus trust it to get them right on its own?"},
    {"key": "proximity", "title": "Closeness", "kind": "likert",
     "min_label": "Keep its distance", "max_label": "Close is fine",
     "question": "How okay are you with the robot being close and in your space — "
                 "near your face, or in your line of sight (say, to a TV or the "
                 "people you're with)?"},
    {"key": "communication", "title": "Signalling", "kind": "likert",
     "min_label": "Stay quiet", "max_label": "Signal every time",
     "question": "How much of a heads-up do you want from the robot — should it "
                 "clearly signal when it's about to act or is waiting on you, or "
                 "stay quiet and just get on with it?"},
]


def _coerce_value(raw: Any) -> Any:
    """Sanitize a webapp Likert answer: int within the 1-7 scale (else None)."""
    try:
        value = int(raw)
    except (TypeError, ValueError):
        return None
    if not PRE_MEAL_SCALE["scale_min"] <= value <= PRE_MEAL_SCALE["scale_max"]:
        return None
    return value


def run_pre_meal_survey(web_interface, data_logger) -> dict[str, Any]:
    """Ask the four latent-trait questions on the webapp and log them (held out).

    Returns {question key: answer}. Blocks on each question (mandatory, no
    timeout). Does NOT park the iPad afterward -- the meal proceeds to
    prediction. The call site (run.py) wraps this in try/except so a failure
    never blocks the meal.
    """
    total = len(PRE_MEAL_QUESTIONS)
    responses: dict[str, Any] = {}
    user_actions: dict[str, str] = {}
    aborted = None
    try:
        web_interface.start_survey(
            total=total, subtitle=_PRE_MEAL_SUBTITLE, eyebrow=_PRE_MEAL_EYEBROW,
            note=_PRE_MEAL_NOTE,
        )
        for step, q in enumerate(PRE_MEAL_QUESTIONS):
            msg_dict = web_interface.send_survey_question(
                field=q["key"], title=q["title"], question=q["question"],
                kind=q["kind"], step=step, total=total,
                scale_min=PRE_MEAL_SCALE["scale_min"],
                scale_max=PRE_MEAL_SCALE["scale_max"],
                min_label=q.get("min_label", ""), max_label=q.get("max_label", ""),
                subtitle=_PRE_MEAL_SUBTITLE, eyebrow=_PRE_MEAL_EYEBROW,
                note=_PRE_MEAL_NOTE,
            )
            if msg_dict is None:
                aborted = "task_selection_jump"
                data_logger.log_pre_meal(field=q["key"], value=None, step=step,
                                         total=total, user_action="jump")
                break
            value = _coerce_value(msg_dict.get("value"))
            action = msg_dict.get("user_action", "tap")
            responses[q["key"]] = value
            user_actions[q["key"]] = action
            data_logger.log_pre_meal(field=q["key"], value=value, step=step,
                                     total=total, user_action=action)
    except WebInterfaceTakeoverInterrupt:
        aborted = "takeover"
    data_logger.log_pre_meal(
        record="summary",
        responses=responses,
        user_actions=user_actions,
        completed=aborted is None and len(responses) == total,
        aborted=aborted,
    )
    return responses
