#!/usr/bin/env python3
"""Simulate the robot/backend side of the feeding webapp for an entire workflow.

This stands in for `web_interface.py` so you can exercise the full UI without the
real robot. It plays the "robot" role on ROS:

    data IN  to the webapp  -> published on /robot_to_webapp and /skill_plan
    data OUT of the webapp  -> received on  /webapp_to_robot   (user selections)

Flow it drives:
    home
      -> preference_context        user picks the meal context (made-up options)
      -> robot_executing           8-step fridge-retrieval plan on the skill strip
           (a preference correction is requested *in between* the steps, twice)
      -> feeding loop              task selection + bite/drink/wipe confirmations

Prerequisites (separate terminals):
    1. roscore
    2. rosbridge with TLS (page is served over https):
         roslaunch rosbridge_server rosbridge_websocket.launch \\
           ssl:=true certfile:=/home/isacc/certs/192.168.1.2.pem \\
           keyfile:=/home/isacc/certs/192.168.1.2-key.pem
    3. the webapp open in a browser (any page; it will be jumped to /home's flow)

Run:
    python3 simulate_workflow.py

Ctrl-C to stop. Legend in the console:  ▶ IN = robot→app,  ◀ OUT = app→robot
"""

import json
import queue
import time

import rospy
from std_msgs.msg import String

TO_WEBAPP = "/robot_to_webapp"
FROM_WEBAPP = "/webapp_to_robot"
SKILL_PLAN = "/skill_plan"

# Pacing (seconds). Bump these up to watch the flow more slowly.
STEP_SECONDS = 10          # time "spent" executing each skill
CONTEXT_SUBSCRIBE_WAIT = 2.0  # context page has no ready-handshake; give it time

# ── Made-up meal context shown on the preference_context page ────────────────
CONTEXT_OPTIONS = {
    "meals": ["Scrambled eggs & toast", "Pasta with marinara", "Rice and curry"],
    "settings": ["Kitchen table", "Living-room couch", "Bedside"],
    "time_of_day": ["Breakfast", "Lunch", "Dinner"],
    "defaults": {
        "meal": "Pasta with marinara",
        "setting": "Kitchen table",
        "time_of_day": "Dinner",
    },
}

# ── Made-up predicted preferences asked for *in between* the workflow ─────────
# Each entry: (predicted_bundle, options-per-field). The page pre-selects the
# predicted value and lets the user change it.
PREFERENCE_PROMPTS = [
    (
        {"bite_size": "Medium", "feeding_rate": "Slow"},
        {"bite_size": ["Small", "Medium", "Large"],
         "feeding_rate": ["Slow", "Medium", "Fast"]},
    ),
    (
        {"utensil": "Fork", "distance_to_mouth": "Close"},
        {"utensil": ["Fork", "Spoon"],
         "distance_to_mouth": ["Close", "Medium", "Far"]},
    ),
]

# ── The 8-step fridge-retrieval plan (shown on the robot_executing skill strip).
# (skill_name, category-tag-for-logging, status-text-shown-on-page)
WORKFLOW = [
    ("navigate_to_fridge",          "Navigation",   "Driving to the fridge…"),
    ("reach_handle_detection_pose", "Manipulation", "Positioning arm to find the handle…"),
    ("detect_handle_keypoints",     "Vision",       "Detecting handle keypoints…"),
    ("open_fridge",                 "Manipulation", "Opening the fridge…"),
    ("reach_plate_detection_pose",  "Manipulation", "Positioning arm to find the plate…"),
    ("detect_plate_keypoints",      "Vision",       "Detecting plate grasp points…"),
    ("move_plate_to_holder",        "Manipulation", "Moving the plate to the holder…"),
    ("close_fridge",                "Manipulation", "Closing the fridge…"),
]
WORKFLOW_PLAN = [s[0] for s in WORKFLOW]

# ── End-of-meal survey (mirrors feeding_deployment.integration.survey;
# inlined so this script stays standalone). ──────────────────────────────────
SURVEY_SCALE = {"scale_min": 1, "scale_max": 7,
                "min_label": "Very Low", "max_label": "Very High"}
SURVEY_QUESTIONS = [
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

# Feeding-loop skill plans per action.
FEED_PLANS = {
    "take_bite": (["pick_utensil", "acquire_bite", "transfer_utensil", "stow_utensil"],
                  "bite_confirm_transfer", "after_bite"),
    "take_sip": (["pick_drink", "transfer_drink", "stow_drink"],
                 "drink_confirm_transfer", "after_drink"),
    "mouth_wiping": (["pick_wipe", "transfer_wipe", "stow_wipe"],
                     "wipe_confirm_transfer", "after_bite"),
}


class WorkflowSim:
    def __init__(self):
        rospy.init_node("webapp_workflow_sim", anonymous=True)
        self.to_webapp = rospy.Publisher(TO_WEBAPP, String, queue_size=10)
        # latched, like the real /skill_plan publisher, so a freshly-mounted page
        # picks up the current plan immediately.
        self.skill_pub = rospy.Publisher(SKILL_PLAN, String, queue_size=1, latch=True)
        self.inbox = queue.Queue()
        rospy.Subscriber(FROM_WEBAPP, String, self._on_msg)
        time.sleep(1.0)  # let pub/sub wire up

    # ── ROS plumbing ─────────────────────────────────────────────────────────
    def _on_msg(self, msg):
        try:
            d = json.loads(msg.data)
        except Exception:
            return
        if d.get("state") == "teleop" and d.get("status") == "heartbeat":
            return  # ignore teleop keep-alives
        print(f"  ◀ OUT  {msg.data}")
        self.inbox.put(d)

    def send(self, d):
        self.to_webapp.publish(String(json.dumps(d)))
        print(f"  ▶ IN   {json.dumps(d)}")

    def publish_plan(self, plan, current):
        self.skill_pub.publish(String(json.dumps({"plan": plan, "current": current})))

    def clear_plan(self):
        self.skill_pub.publish(String(json.dumps({"plan": [], "current": -1})))

    def wait_for(self, predicate, what, timeout=None, resend=None):
        # ``resend``: zero-arg callback re-invoked every second while waiting,
        # like WebInterface.get_required_web_interface_message -- lets you test
        # the survey's mid-question browser-reload recovery without the robot.
        print(f"  … waiting for: {what}")
        start = time.time()
        last_resend = time.time()
        while not rospy.is_shutdown():
            try:
                d = self.inbox.get(timeout=0.2)
            except queue.Empty:
                if timeout and (time.time() - start) > timeout:
                    print(f"  ! timeout waiting for {what}")
                    return None
                if resend is not None and time.time() - last_resend >= 1.0:
                    resend()
                    last_resend = time.time()
                continue
            if predicate(d):
                return d
        return None

    def back_to_executing(self):
        self.send({"state": "robot_executing", "status": "jump"})

    # ── Phase 1: meal context ────────────────────────────────────────────────
    def do_context(self):
        print("\n=== 1. MEAL CONTEXT (user inputs context) ===")
        self.send({"state": "preference_context", "status": "jump"})
        # The context page does not announce readiness, so give it a moment to
        # mount + subscribe, then send the (non-latched) options once.
        time.sleep(CONTEXT_SUBSCRIBE_WAIT)
        self.send({"state": "preference_context_data", **CONTEXT_OPTIONS})
        resp = self.wait_for(
            lambda d: d.get("state") == "preference_context_response",
            "user's meal-context choice")
        if resp:
            print(f"    → context: meal={resp.get('meal')!r}, "
                  f"setting={resp.get('setting')!r}, time={resp.get('time_of_day')!r}")

    # ── A preference correction injected mid-workflow ────────────────────────
    def do_correction(self, predicted, options):
        print("\n--- PREFERENCE CORRECTION (asked in between the workflow) ---")
        self.send({"state": "preference_correction", "status": "jump"})
        # Wait for the page's ready-handshake before sending the data (matches the
        # hardened backend in web_interface.get_preference_corrections).
        self.wait_for(
            lambda d: d.get("state") == "preference_correction"
            and d.get("status") == "ready",
            "preference_correction page to report ready")
        self.send({"state": "preference_correction_data",
                   "predicted_bundle": predicted, "options": options})
        resp = self.wait_for(
            lambda d: d.get("state") == "preference_correction_response",
            "user's corrected preferences")
        if resp:
            print(f"    → corrected bundle: {resp.get('bundle')}")
        # The page auto-returns to /robot_executing on submit; re-assert anyway.
        self.back_to_executing()

    # ── Phase 2: fridge-retrieval workflow ───────────────────────────────────
    def run_workflow(self):
        print("\n=== 2. FRIDGE RETRIEVAL (8 steps) ===")
        self.back_to_executing()
        time.sleep(0.5)
        for i, (skill, tag, text) in enumerate(WORKFLOW):
            self.publish_plan(WORKFLOW_PLAN, i)
            self.send({"state": "executing", "status": text})
            print(f"  [{tag}] step {i + 1}/{len(WORKFLOW)}: {skill}")
            time.sleep(STEP_SECONDS)
            # Ask the user for preferences in between specific steps.
            if skill == "open_fridge":
                self.do_correction(*PREFERENCE_PROMPTS[0])
            elif skill == "move_plate_to_holder":
                self.do_correction(*PREFERENCE_PROMPTS[1])
        self.clear_plan()
        print("  fridge retrieval complete.")

    # ── Ready-for-feeding gate (mirrors WebInterface.get_feeding_ready_confirmation:
    # no autocontinue, resend until the user taps) ────────────────────────────
    def do_feeding_ready(self):
        print("\n--- FEEDING READY (waiting for the user to take their seat) ---")
        jump = {"state": "feeding_ready", "status": "jump"}
        self.send(jump)
        self.wait_for(
            lambda d: d.get("state") == "feeding_ready"
            and d.get("status") == "confirm",
            "user confirm on feeding_ready",
            resend=lambda: self.send(jump))
        self.back_to_executing()

    # ── Phase 3: feeding loop ────────────────────────────────────────────────
    def run_feeding_loop(self):
        print("\n=== 3. FEEDING LOOP ===")
        # First choice comes from the task_selection page; afterwards the
        # after_bite / after_drink pages act as the next hub (they also publish
        # state == 'task_selection').
        self.send({"state": "task_selection", "status": "jump"})
        while not rospy.is_shutdown():
            resp = self.wait_for(
                lambda d: d.get("state") == "task_selection",
                "user's task choice (take_bite / take_sip / mouth_wiping / finish_feeding)")
            if resp is None:
                return
            action = resp.get("status")
            print(f"    → task chosen: {action}")

            if action == "finish_feeding":
                self.send({"state": "robot_executing", "status": "jump"})
                self.publish_plan(["place_plate_in_sink"], 0)
                self.send({"state": "executing", "status": "Cleaning up — taking the plate to the sink…"})
                time.sleep(STEP_SECONDS)
                # Ask the user to confirm the plate release before "ungrasping"
                # (mirrors WebInterface.get_plate_release_confirmation).
                self.send({"state": "plate_release_confirm", "status": "sink"})
                self.wait_for(
                    lambda d: d.get("state") == "plate_release_confirm"
                    and d.get("status") == "confirm",
                    "user confirm on plate_release_confirm")
                self.back_to_executing()
                time.sleep(STEP_SECONDS / 2)
                self.clear_plan()
                self.do_survey()
                print("  meal finished (page resting on Thank You).")
                return

            if action not in FEED_PLANS:
                print(f"  (unhandled task '{action}', re-prompting)")
                self.send({"state": "task_selection", "status": "jump"})
                continue

            plan, confirm_state, hub_state = FEED_PLANS[action]
            self.back_to_executing()
            for i, skill in enumerate(plan):
                self.publish_plan(plan, i)
                self.send({"state": "executing", "status": _skill_text(skill)})
                print(f"  [Manipulation] {action} step {i + 1}/{len(plan)}: {skill}")
                time.sleep(STEP_SECONDS)
            self.clear_plan()

            # Ask the user to confirm the transfer.
            self.send({"state": confirm_state, "status": "jump"})
            self.wait_for(
                lambda d: d.get("state") == confirm_state,
                f"user confirm on {confirm_state}")

            # Move to the post-action hub for the next round.
            self.send({"state": hub_state, "status": "jump"})

    # ── Phase 4: end-of-meal survey (mirrors WebInterface.start_survey /
    # send_survey_question / finish_survey, including the 1 s resends) ────────
    def do_survey(self):
        print("\n=== 4. END-OF-MEAL SURVEY ===")
        total = len(SURVEY_QUESTIONS)
        jump = {"state": "survey", "status": "jump", "total": total}
        self.send(jump)
        self.wait_for(
            lambda d: d.get("state") == "survey" and d.get("status") == "ready",
            "survey page to report ready",
            resend=lambda: self.send(jump))
        for step, q in enumerate(SURVEY_QUESTIONS):
            data = {"state": "survey_data", "field": q["key"],
                    "title": q["title"], "question": q["question"],
                    "kind": q["kind"], "step": step, "total": total,
                    **SURVEY_SCALE}
            self.send(data)
            resp = self.wait_for(
                lambda d, f=q["key"]: d.get("state") == "survey_response"
                and d.get("field") == f,
                f"answer to '{q['title']}' ({step + 1}/{total})",
                resend=lambda m=data: self.send(m))
            if resp:
                print(f"    → {q['key']} = {resp.get('value')!r}")
        done = {"state": "thank_you", "status": "jump"}
        self.send(done)
        self.wait_for(
            lambda d: d.get("state") == "thank_you" and d.get("status") == "ready",
            "thank_you page to report ready",
            resend=lambda: self.send(done))

    def run(self):
        print("Legend:  ▶ IN = robot→app   ◀ OUT = app→robot\n")
        self.do_context()
        self.run_workflow()
        self.do_feeding_ready()
        self.run_feeding_loop()
        print("\n=== WORKFLOW COMPLETE ===")


def _skill_text(skill):
    return skill.replace("_", " ").capitalize() + "…"


if __name__ == "__main__":
    try:
        WorkflowSim().run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\nstopped.")
