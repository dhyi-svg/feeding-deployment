"""An interface for perception (robot joints, human head poses, etc.)."""

import threading
import time
from typing import Any
import pickle
import cv2
import argparse

import numpy as np
from pybullet_helpers.geometry import Pose
from pybullet_helpers.joint import JointPositions
from scipy.spatial.transform import Rotation as R
import json
import queue
from pathlib import Path

try:
    import rospy
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String, Empty
    from cv_bridge import CvBridge

except ModuleNotFoundError:
    pass

from feeding_deployment.transparency.continuous_llm import TransparencyContinuous

try:
    from feeding_deployment.preference_learning.config.preference_bundle import (
        PREFERENCE_BUNDLE as _PREF_BUNDLE_DIMS,
    )
    _PREF_LABELS = {dim.field: dim.label for dim in _PREF_BUNDLE_DIMS}
    _PREF_SHORT_DESCRIPTIONS = {
        dim.field: dim.short_description for dim in _PREF_BUNDLE_DIMS
    }
except Exception:
    _PREF_LABELS = {}
    _PREF_SHORT_DESCRIPTIONS = {}

class WebInterfaceTakeoverInterrupt(Exception):
    """Raised out of a blocking web-interface wait when a mid-skill takeover is
    requested, so the HLA layer can run teleop recovery instead of the wait
    silently dropping the takeover message."""


class WebInterface:
    '''
    An interface to interact with the web interface.
    '''
    def __init__(self, task_selection_queue: queue.Queue, data_logger) -> None:

        # Single logs handle: `.state_dir` is the shared user log directory and the
        # logger captures every user input + image shown for the daily release
        # (no-op when its day is disabled).
        self.data_logger = data_logger
        log_dir = data_logger.state_dir

        # Used for generating continuous explanations.
        self.transparency_continuous = TransparencyContinuous(log_dir)
        self.webapp_sent_messages_log = log_dir / "webapp_sent_messages.txt"
        self.webapp_explanation_messages_log = log_dir / "webapp_explanation_messages.txt"
        self.webapp_received_messages_log = log_dir / "webapp_received_messages.txt"

        # Objects of task_selection_queue are dicts and can be of the following types:
        # {'task': 'meal_assistance', 'type': 'bite' / 'sip' / 'wipe'}
        # {'task': 'personalization', 'type': 'transparency' / 'adaptability' / 'gesture'}
        self.task_selection_queue = task_selection_queue
        self.task_selection_jump = False

        # Queue containing all messages from the web interface.
        self.received_web_interface_messages = queue.Queue()
        
        # Create a publisher for communication with the web interface.
        self.web_interface_publisher = rospy.Publisher("/robot_to_webapp", String, queue_size=10)
        # Latched so a page that subscribes mid-skill (e.g. the teleop screens,
        # which only mount after takeover) immediately receives the current plan.
        self.skill_plan_publisher = rospy.Publisher("/skill_plan", String, queue_size=1, latch=True)
        self.web_interface_image_publisher = rospy.Publisher("/camera/image/compressed", CompressedImage, queue_size=10)
        self.image_bridge = CvBridge()
        self.user_preference = None
        self.web_interface_sub = rospy.Subscriber("/webapp_to_robot", String, self._message_callback, queue_size=100)
        self.base_takeover_sub = rospy.Subscriber(
            "/shared_autonomy/takeover",
            Empty,
            self._on_base_takeover,
            queue_size=1,
        )
        self.base_done_sub = rospy.Subscriber(
            "/shared_autonomy/done",
            Empty,
            self._on_base_done,
            queue_size=1,
        )
        self.base_resume_sub = rospy.Subscriber(
            "/shared_autonomy/resume",
            Empty,
            self._on_base_resume,
            queue_size=1,
        )

        # --- Settings overlay (view/edit already-set preferences) ---
        # Dedicated, isolated topic pair so settings traffic NEVER touches the main
        # /webapp_to_robot queue or the takeover/confirmation path. The robot->app
        # topic is latched so an overlay opening mid-meal immediately gets the current
        # prefs without a round-trip race.
        self.settings_publisher = rospy.Publisher(
            "/robot_settings_to_webapp", String, queue_size=1, latch=True
        )
        self.settings_sub = rospy.Subscriber(
            "/webapp_settings_to_robot", String, self._settings_callback, queue_size=20
        )
        # Accessors into the live PreferenceSession, registered by run.py while a meal
        # session exists; None between meals (the overlay then shows the empty state).
        self._prefs_view_fn = None
        self._prefs_edit_fn = None
        # Set == panel closed. The preference-ask and next-HLA stalls wait on this.
        self._settings_panel_closed = threading.Event()
        self._settings_panel_closed.set()
        # Edits are applied on a single worker thread (off the ROS callback) because an
        # edit may trigger a slow LLM re-prediction. A None sentinel stops the worker.
        self._settings_apply_queue: "queue.Queue" = queue.Queue()

        time.sleep(1.0)  # Wait for the subscriber to connect

        self.current_page = "task_selection" # task_selection, transparency, adaptability

        # for setting autocontinue time in task selection page
        self.bite_autocontinue_timeout = 10.0
        self.drink_autocontinue_timeout = 10.0

        # for escaping out of while loops
        self.active = True

        self.explanation_lock = threading.Lock() # Lock for generating continuous explanations

        # Mid-skill manual takeover: set when the user taps the global "Take Over"
        # button. Checked by the executive (idle loop) and by execute_robot_command
        # (mid-skill). _takeover_stop_fn, if registered, is called immediately to
        # best-effort abort whatever move is currently running.
        self.takeover_event = threading.Event()
        self._takeover_stop_fn = None
    
        # Start the thread for generating continuous explanations.
        self.transparency_continuous_thread = threading.Thread(target=self.provide_continuous_explanations)
        self.transparency_continuous_thread.start()

        # Worker that applies settings-overlay edits off the ROS callback thread
        # (an edit may run a slow LLM re-prediction). Always created so the join in
        # stop_all_threads is unconditional.
        self.settings_apply_thread = threading.Thread(target=self._settings_apply_worker)
        self.settings_apply_thread.start()

    def stop_all_threads(self) -> None:
        self.active = False
        try:
            self.transparency_continuous_thread.join()
        except Exception as e:
            print("Error stopping transparency continuous thread: ", e)
        try:
            # Unblock the worker's queue.get() with the shutdown sentinel, then join.
            self._settings_apply_queue.put(None)
            self.settings_apply_thread.join()
        except Exception as e:
            print("Error stopping settings apply thread: ", e)
        try:
            self.gesture_listener_thread.join()
        except Exception as e:
            print("Error stopping gesture listener thread: ", e)

    def switch_to_explanation_page(self) -> None:
        self.current_page = "robot_executing"
        self.web_interface_publisher.publish(String(json.dumps({"state": "robot_executing", "status": "jump"})))

    def publish_skill_plan(self, plan_names: list, current_index: int) -> None:
        """Publish (latched) the ordered skill plan and the index of the skill
        currently executing, so the web interface can show the last / current /
        next skill with the current one highlighted. Latching lets late
        subscribers (e.g. the teleop screens) get the current skill on connect."""
        self.skill_plan_publisher.publish(String(json.dumps({
            "plan": plan_names,
            "current": current_index,
        })))

    def clear_skill_plan(self) -> None:
        """Clear the skill plan (no skill executing, e.g. idle at task selection)."""
        self.skill_plan_publisher.publish(String(json.dumps({"plan": [], "current": -1})))

    def _send_message(self, msg_dict: dict[str, Any], explanation=False) -> None:
        self.web_interface_publisher.publish(String(json.dumps(msg_dict)))
        if explanation and msg_dict["status"] != "":
            with open(self.webapp_explanation_messages_log, "a") as f:
                f.write(json.dumps(msg_dict) + "\n")
        else:
            with open(self.webapp_sent_messages_log, "a") as f:
                f.write(json.dumps(msg_dict) + "\n")

    def _send_image(self, image, flip=True) -> None:
        # Every key image shown to the user (plate image, bite-selection image,
        # detection-confirmation vis, color-correction frame) flows through here.
        #
        # The camera is mounted upside down, so every frame it produces is rotated
        # 180 degrees. Flip it back here -- the single point every webapp image
        # passes through -- so the user always sees a naturally upright image.
        # Callers must NOT pre-rotate the image (that would double-flip it).
        #
        # Pass flip=False when the camera is already upright for this capture -- e.g.
        # the robot physically flips its (upside-down) camera to pick the plate out of
        # the microwave, so that frame is already right-side-up and must NOT be rotated.
        # We (don't) rotate before logging so the data logger records exactly what the
        # user saw.
        if image is not None and flip:
            image = cv2.rotate(image, cv2.ROTATE_180)
        if self.data_logger is not None:
            # "webapp" routes to images/webapp_images/ (monotonic, display order) --
            # the faithful record of every frame shown on the iPad.
            self.data_logger.log_image("webapp", image)
        self.web_interface_image_publisher.publish(self.image_bridge.cv2_to_compressed_imgmsg(image))

    def _message_callback(self, msg: "String") -> None:
        """Callback for the web interface."""
        msg_dict = json.loads(msg.data)
        print("Received message on /webapp_to_robot: ", msg.data)

        # Teleop heartbeats arrive every few seconds; keep them out of the print
        # spam and the verbose received-messages log, but still enqueue them
        # below so the teleop session sees them as keep-alive.
        is_teleop_heartbeat = (
            msg_dict.get("state") == "teleop" and msg_dict.get("status") == "heartbeat"
        )
        if not is_teleop_heartbeat:
            print("Received message on /webapp_to_robot: ", msg.data)
            with open(self.webapp_received_messages_log, "a") as f:
                f.write(msg.data + "\n")
            if self.data_logger is not None:
                self.data_logger.log_user_input("webapp_to_robot", msg_dict)

        # Mid-skill manual takeover request: flag it (and best-effort abort the
        # in-flight move) so the executive hands control to the teleop screen.
        if msg_dict.get("state") == "teleop" and msg_dict.get("status") == "takeover":
            print("Received takeover request from web interface!")
            already_requested = self.takeover_event.is_set()
            self.takeover_event.set()
            if not already_requested:
                if self._takeover_stop_fn is not None:
                    try:
                        self._takeover_stop_fn()
                    except Exception as e:
                        print("Error calling takeover stop function: ", e)
                else:
                    print("No takeover stop function registered.")
            return

        self.task_selection_jump = False

        # Some messages (e.g. *_response payloads) carry no "status"; use .get so
        # they fall through to the queue instead of raising KeyError in this
        # callback (which would silently drop them and hang the waiting caller).
        if msg_dict.get("status") == "finish_feeding":
            task_selected = {
                "task": "finish_feeding",
                "type": "place_plate_in_sink",
            }
            self.task_selection_queue.put(task_selected)
        elif msg_dict["state"] == "task_selection":
            if msg_dict["status"] == "take_bite":
                task_selected = {
                    "task": "meal_assistance",
                    "type": "bite",
                }
            elif msg_dict["status"] == "take_sip":
                task_selected = {
                    "task": "meal_assistance",
                    "type": "sip",
                }
            elif msg_dict["status"] == "mouth_wiping":
                task_selected = {
                    "task": "meal_assistance",
                    "type": "wipe",
                }
            elif msg_dict["status"] == "transparency":
                task_selected = {
                    "task": "personalization",
                    "type": "transparency",
                }
            elif msg_dict["status"] == "adaptability":
                task_selected = {
                    "task": "personalization",
                    "type": "adaptability",
                }
            elif msg_dict["status"] == "gesture":
                task_selected = {
                    "task": "personalization",
                    "type": "gesture",
                }
            elif msg_dict["status"] == "teleop_recovery":
                task_selected = {
                    "task": "teleop",
                    "type": "manual_recovery",
                }
            elif msg_dict["status"] == "jump":
                self.task_selection_jump = True
                return
            else:
                print("Invalid task selection status received from interface: ", msg_dict["status"])
                return
            
            # remove explanation lock (if it exists)
            if self.explanation_lock.locked():
                self.explanation_lock.release()
            
            # set current page to task_selection (effectively reseting transparency and adaptability pages)
            self.current_page = "task_selection"
            
            self.task_selection_queue.put(task_selected)
        else:
            self.received_web_interface_messages.put(msg_dict)

    def _on_base_takeover(self, _msg: "Empty") -> None:
        print("User took over robot base control via web app.")

    def _on_base_done(self, _msg: "Empty") -> None:
        print("User finished robot base control teleoperation.")

    def _on_base_resume(self, _msg: "Empty") -> None:
        print("User resumed autonomous navigation after base teleoperation.")

    def register_takeover_stop(self, fn) -> None:
        """Register a function (e.g. robot_interface.stop_action) called the moment
        a takeover is requested, to best-effort abort the in-flight move."""
        self._takeover_stop_fn = fn

    def consume_takeover(self) -> bool:
        """Return True (and clear) if a manual takeover has been requested."""
        if self.takeover_event.is_set():
            self.takeover_event.clear()
            return True
        return False

    # ---- Settings overlay plumbing (isolated from _message_callback) ----
    def register_preferences_accessor(self, view_fn, edit_fn) -> None:
        """Wire the settings overlay to the live PreferenceSession. ``view_fn()``
        returns the list of editable prefs; ``edit_fn(field, value)`` applies one
        edit. Called by run.py when a meal session is built."""
        self._prefs_view_fn = view_fn
        self._prefs_edit_fn = edit_fn
        self._publish_settings()

    def clear_preferences_accessor(self) -> None:
        """Drop the accessor at meal end and clear the latched prefs so a later-opening
        overlay shows the empty state, not the last meal's stale list."""
        self._prefs_view_fn = None
        self._prefs_edit_fn = None
        self.settings_publisher.publish(String(json.dumps({"preferences": []})))

    def _publish_settings(self) -> None:
        """Publish the current editable prefs on the (latched) settings topic."""
        view_fn = self._prefs_view_fn
        prefs = []
        if view_fn is not None:
            try:
                prefs = view_fn()
            except Exception as e:
                print("Error building settings view: ", e)
                prefs = []
        self.settings_publisher.publish(String(json.dumps({"preferences": prefs})))

    def _settings_callback(self, msg: "String") -> None:
        """Handle settings-overlay messages on the dedicated topic. Runs on its own
        ROS subscriber thread; never touches received_web_interface_messages."""
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        action = data.get("action")
        if action == "open":
            # Panel opened: mark open (so the ask/HLA stalls wait) and send prefs.
            self._settings_panel_closed.clear()
            self._publish_settings()
        elif action == "close":
            self._settings_panel_closed.set()
        elif action == "set":
            # Apply off the callback thread: an edit may run a slow LLM re-prediction.
            self._settings_apply_queue.put((data.get("field"), data.get("value")))

    def _settings_apply_worker(self) -> None:
        """Serialize settings edits off the ROS callback thread."""
        while True:
            item = self._settings_apply_queue.get()
            if item is None:  # shutdown sentinel
                break
            field, value = item
            edit_fn = self._prefs_edit_fn
            if edit_fn is not None and field is not None:
                try:
                    edit_fn(field, value)
                except Exception as e:
                    print("Error applying settings edit: ", e)
            # Re-publish so the overlay reflects the authoritative state.
            self._publish_settings()

    def wait_until_settings_closed(self, status: str, *, raise_on_takeover: bool) -> None:
        """Block the caller while the settings panel is open, so the robot doesn't ask
        for new prefs / start the next HLA mid-edit. Publishes ``status`` once (e.g.
        'robot_waiting' before an ask, 'robot_paused' before an HLA) so the panel can
        show the right banner. Returns immediately if the panel is already closed.

        On a takeover: raise WebInterfaceTakeoverInterrupt if ``raise_on_takeover``
        (the preference-ask path, which already raises from get_required_web_interface_message),
        else just return and leave takeover_event for the existing handler (the
        execute-loop path, which must not get a new exception)."""
        if self._settings_panel_closed.is_set():
            return
        self.settings_publisher.publish(String(json.dumps({"status": status})))
        print(f"Settings panel open; stalling ({status}) until the user closes it ...")
        while self.active and not self._settings_panel_closed.is_set():
            if self.takeover_event.is_set():
                if raise_on_takeover:
                    raise WebInterfaceTakeoverInterrupt()
                return
            time.sleep(0.1)

    def clear_received_messages(self) -> None:
        while not self.received_web_interface_messages.empty():
            self.received_web_interface_messages.get()

    def set_bite_autocontinue_timeout(self, timeout: float) -> None:
        self.bite_autocontinue_timeout = timeout

    def set_drink_autocontinue_timeout(self, timeout: float) -> None:
        self.drink_autocontinue_timeout = timeout

    def ready_for_task_selection(self, last_task_type = None) -> None:
        """Moves the web interface to the task selection page."""

        self.current_page = "task_selection"

        # No skill is executing while idle at task selection.
        self.clear_skill_plan()

        print("Sending message to web interface to move to task selection page with last task type: ", last_task_type)

        # after bite and after sip are special, because they have bite and sip preselected for autocontinue with a timeout
        if last_task_type == "bite":
            self._send_message({"state": "after_bite", "status": "jump"})
            time.sleep(0.5)
            self._send_message({"state": "auto_time", "status": str(self.bite_autocontinue_timeout)})
        elif last_task_type == "sip":
            self._send_message({"state": "after_drink", "status": "jump"})
            time.sleep(0.5)
            self._send_message({"state": "auto_time", "status": str(self.drink_autocontinue_timeout)})
        else:
            self._send_message({"state": "task_selection", "status": "jump"})

    def wait_for_start_meal(self) -> None:
        """Block on the home page until the user presses "Start Meal".

        Home is the webapp's default route (and the page a refresh returns to),
        so gating the meal on a user-initiated press here -- rather than firing a
        one-shot jump the app might miss -- makes startup robust against refreshes
        and slow webapp connections. The home page sends
        {"state":"home","status":"start_meal"} on the press; we drain any stale
        messages first so an old press can't satisfy this immediately.
        """
        self.current_page = "home"
        self.clear_received_messages()
        # Best-effort nudge to pull a connected client sitting on another page
        # (e.g. left on task_selection from a prior run) back to home. Non-latched,
        # so a disconnected/refreshing client won't get it -- the user press on
        # home is still the actual gate, and home is the default/refresh route.
        self._send_message({"state": "home", "status": "jump"})
        print("Waiting for the user to press 'Start Meal' on the home page ...")
        self.get_required_web_interface_message(
            lambda m: m.get("state") == "home" and m.get("status") == "start_meal"
        )

    def get_required_web_interface_message(self, condition, resend=None, resend_interval=1.0) -> dict[str, Any]:
        """Parses through all messages received from the web interface and returns the oldest one satisfying the condition.

        ``resend``: optional zero-arg callback re-invoked every ``resend_interval``
        seconds while waiting. The page-jump topic (/robot_to_webapp) is NOT
        latched and each webapp page opens a fresh rosbridge subscription on
        mount, so a jump published during a page transition can be dropped. For
        handshakes that wait on a page mounting (e.g. preference_correction
        "ready"), pass ``resend`` to re-publish the jump until the page is up.
        """
        print_once = True
        last_resend = time.time()
        while self.active:
            # A mid-skill takeover must break a blocking confirm wait. Checked
            # before draining the queue so it fires even when the queue is full
            # of non-matching messages (which this loop otherwise discards). The
            # HLA layer clears the event via consume_takeover(); we only read it.
            if self.takeover_event.is_set():
                raise WebInterfaceTakeoverInterrupt()
            if self.task_selection_jump:
                return None
            try:
                msg_dict = self.received_web_interface_messages.get_nowait()
                try:
                    if condition(msg_dict):
                        print("Received required message from the web interface")
                        return msg_dict
                except Exception as e:
                    print("Error in condition: ", e)
                    print("continuing to wait for required message from the web interface ...")
            except queue.Empty:
                if print_once:
                    print("Waiting for required message from the web interface ...")
                    print_once = False
                if resend is not None and time.time() - last_resend >= resend_interval:
                    resend()
                    last_resend = time.time()
                time.sleep(0.1)
                continue

    #### Meal Assistance Pages ####

    # NOTE: the old meal_setup page (get_new_meal_input) was removed. Food items
    # now come from the chosen meal's MealContents and the bite-ordering
    # preference is predicted/corrected via the preference session, so FLAIR is
    # already configured before bite acquisition begins.

    def get_next_bite_selection(self, plate_image, n_solid_food_types, bite_data, predicted_bite, n_dip_food_types, dip_data, autocontinue_timeout) -> None:

        self.current_page = "meal_assistance"

        # Drop stale messages so we wait for the ready for THIS navigation.
        self.clear_received_messages()

        # Jump to next bite selection page
        jump_msg = {"state": "bite_selection", "status": "jump"}
        self._send_message(jump_msg)

        # Wait until the page has mounted and reports it's ready for data before
        # sending it. /robot_to_webapp is not latched and the bite_selection page
        # re-subscribes asynchronously on mount, so a jump (and the image/data
        # that follow) published on a fixed timer can be dropped -- leaving the
        # page waiting forever, re-emitting ready_for_initial_data, until it times
        # out and bounces back to task selection (the flicker). Resend the jump
        # until the page reports ready (mirrors the preference_context page).
        self.get_required_web_interface_message(
            lambda m: (
                m.get("state") == "bite_selection"
                and m.get("status") == "ready_for_initial_data"
            ),
            resend=lambda: self._send_message(jump_msg),
        )

        # Send required data for the next bite selection page
        self._send_image(plate_image)
        time.sleep(0.2)
        self._send_message({"n_food_types": n_solid_food_types, "data": bite_data, "current_bite": predicted_bite})
        time.sleep(0.2)
        self._send_message({"n_ordering": n_dip_food_types, "data": dip_data})
        # set autocontinue timeout
        time.sleep(0.2)
        self._send_message({"state": "auto_time", "status": str(autocontinue_timeout)})

        bite_msg_dict, dip_msg_dict = None, None
        # Get the user's next bite selection
        msg_dict_1 = self.get_required_web_interface_message(
            lambda msg_dict: (
                ((msg_dict["status"] == "acquire_food" or msg_dict["status"] == 0 or msg_dict["status"] == 2) or msg_dict["state"] == "dip_selection")
            )
        )
        if msg_dict_1["status"] == "acquire_food" or msg_dict_1["status"] == 0 or msg_dict_1["status"] == 2:
            bite_msg_dict = msg_dict_1
            # But if bite is manual, then we should not wait for dip selection
            if bite_msg_dict["status"] == "acquire_food":
                dip_msg_dict = self.get_required_web_interface_message(
                    lambda msg_dict: (
                        (msg_dict["state"] == "dip_selection")
                    )
                )
                return "autonomous", bite_msg_dict["data"], dip_msg_dict["status"]
            else:
                if bite_msg_dict["status"] == 0:
                    return "manual_skewering", bite_msg_dict["positions"], None
                elif bite_msg_dict["status"] == 2:
                    return "manual_dipping", bite_msg_dict["positions"], None
                else:
                    print("Unsupported message received from the web interface: ", bite_msg_dict)
        else:
            dip_msg_dict = msg_dict_1
            # Dip recieved means bite has to be autonomous
            bite_msg_dict = self.get_required_web_interface_message(
                lambda msg_dict: (
                    (msg_dict["status"] == "acquire_food" or msg_dict["status"] == 0 or msg_dict["status"] == 2)
                )
            )
            return "autonomous", bite_msg_dict["data"], dip_msg_dict["status"]


    def get_successful_food_acquisition_confirmation(self, autocontinue_seconds: float = 0.0) -> None:

        self.current_page = "meal_assistance"

        # Jump to bite confirm transfer page. autocontinue_seconds > 0 makes the
        # page count down and auto-send "confirm" on expiry (confirm_feeding_pickup
        # = "yes (with auto-continue countdown)"); <= 0 means wait for the user. The jump is
        # re-sent until answered: the routing jump is consumed by the PREVIOUS
        # page, so the freshly-mounted page learns its countdown from the first
        # resend it sees (also heals a dropped jump on the non-latched topic).
        jump_msg = {"state": "bite_confirm_transfer", "status": "jump",
                    "autocontinue_seconds": float(autocontinue_seconds)}
        self._send_message(jump_msg)

        # Wait until the user confirms that the food has been acquired
        msg_dict = self.get_required_web_interface_message(
            lambda msg_dict: (
                (msg_dict["state"] == "bite_confirm_transfer")
            ),
            resend=lambda: self._send_message(jump_msg),
            resend_interval=2.0,
        )

        if msg_dict["status"] == "confirm":
            return True
        elif msg_dict["status"] == "cancel":
            return False
        else:
            print("Unsupported message received from the web interface: ", msg_dict)

    def get_drink_transfer_confirmation(self, autocontinue_seconds: float = 0.0) -> None:

        self.current_page = "meal_assistance"

        # Jump to drink confirm transfer page (autocontinue_seconds + resend
        # semantics as in get_successful_food_acquisition_confirmation).
        jump_msg = {"state": "drink_confirm_transfer", "status": "jump",
                    "autocontinue_seconds": float(autocontinue_seconds)}
        self._send_message(jump_msg)

        # Wait until the user confirms that the drink has been transferred
        self.get_required_web_interface_message(
            lambda msg_dict: (
                (msg_dict["state"] == "drink_confirm_transfer" and msg_dict["status"] == "confirm")
            ),
            resend=lambda: self._send_message(jump_msg),
            resend_interval=2.0,
        )

    def get_wipe_transfer_confirmation(self, autocontinue_seconds: float = 0.0) -> None:

        self.current_page = "meal_assistance"

        # Jump to wipe confirm transfer page (autocontinue_seconds + resend
        # semantics as in get_successful_food_acquisition_confirmation).
        print("Jumping to wipe confirm transfer page")
        jump_msg = {"state": "wipe_confirm_transfer", "status": "jump",
                    "autocontinue_seconds": float(autocontinue_seconds)}
        self._send_message(jump_msg)

        # Wait until the user confirms that the wipe has been transferred
        self.get_required_web_interface_message(
            lambda msg_dict: (
                (msg_dict["state"] == "wipe_confirm_transfer" and msg_dict["status"] == "confirm")
            ),
            resend=lambda: self._send_message(jump_msg),
            resend_interval=2.0,
        )

    def get_plate_release_confirmation(self, location: str, autocontinue_seconds: float = 0.0) -> None:
        """Block until the user confirms it is safe to release the plate.

        ``location`` is one of "microwave", "table", "sink"; the frontend
        routeMap turns it into a query param that selects the page copy.
        A takeover while blocked raises WebInterfaceTakeoverInterrupt from
        get_required_web_interface_message (handled by the HLA layer).
        ``autocontinue_seconds`` > 0 makes the page count down and auto-send
        "confirm" on expiry (confirm_manipulation = "yes (with auto-continue countdown)");
        <= 0 keeps today's wait-for-the-user behavior.
        """
        self.current_page = "plate_release_confirm"

        # Drop stale messages so an old confirm (e.g. from the microwave
        # release earlier in the same meal) can't satisfy this wait instantly.
        self.clear_received_messages()

        # The jump doubles as the location payload: the currently-mounted page
        # does the routing, so a fresh page would never see extra keys on the
        # jump message itself. Resend until confirmed -- /robot_to_webapp is
        # not latched and a drop here would deadlock the robot mid-hold. The
        # freshly-routed page picks up autocontinue_seconds from the first
        # resend it sees after mounting (<= 2 s).
        jump_msg = {"state": "plate_release_confirm", "status": location,
                    "autocontinue_seconds": float(autocontinue_seconds)}
        self._send_message(jump_msg)

        msg_dict = self.get_required_web_interface_message(
            lambda m: (
                m.get("state") == "plate_release_confirm"
                and m.get("status") == "confirm"
            ),
            resend=lambda: self._send_message(jump_msg),
            resend_interval=2.0,
        )
        if msg_dict is None:
            # The webapp bounced to task selection. The plate is already
            # resting at its placement pose, so releasing is the safe way out
            # (better than holding forever with the arm inside the microwave).
            print("WARNING: plate release confirmation interrupted by task "
                  "selection jump; releasing the plate anyway.")

        # Move the iPad to the generic 'robot executing' page while the skill
        # finishes its retreat motions (also keeps current_page truthful).
        self.switch_to_explanation_page()

    #### Detection Confirmation Page ####

    def get_detection_confirmation(self, detection_type: str, vis_image=None,
                                   autocontinue_seconds: float = 0.0) -> bool:
        """Show a detection visualization on the web app and ask the user to confirm.

        This drives the generic detection-confirmation page. ``detection_type``
        selects which copy is shown to the user (e.g. ``"handle"``, ``"button"``,
        ``"plate"``) and the visualization image (if any) is sent to the web app.
        Returns True if the user confirms the detection looks correct, and False
        if the perception should be re-run. ``autocontinue_seconds`` > 0 makes
        the page count down and auto-confirm on expiry (confirm_manipulation =
        "yes (with auto-continue countdown)"); <= 0 waits for the user.
        """
        self.current_page = "detection_confirm"

        # Jump to the detection confirmation page.
        self._send_message({"state": "detection_confirm", "status": "jump", "detection_type": detection_type})

        # Wait for the web interface to be ready, then tell the (now-mounted)
        # confirmation page which detection this is and send the visualization
        # image. The "info" status is not in the frontend routeMap, so it only
        # updates the page's copy without triggering a re-navigation.
        time.sleep(0.5)
        self._send_message({"state": "detection_confirm", "status": "info", "detection_type": detection_type,
                            "autocontinue_seconds": float(autocontinue_seconds)})
        if vis_image is not None:
            self._send_image(vis_image)

        # Wait until the user confirms or rejects the detection.
        msg_dict = self.get_required_web_interface_message(
            lambda msg_dict: (msg_dict["state"] == "detection_confirm")
        )

        if msg_dict is None:
            return False
        confirmed = msg_dict["status"] == "confirm"
        if confirmed:
            # Detection accepted: move the iPad off the (now-stale) detection image
            # to the explanation page while the skill continues. On "redo" we leave
            # the page in place so perception can re-run and re-show the image.
            self.switch_to_explanation_page()
        return confirmed

    def get_attachment_detection_action(self, detection_type: str, vis_image=None, flip=True,
                                        autocontinue_seconds: float = 0.0) -> str:
        """Like get_detection_confirmation but also supports 'correct_color' action.

        Returns 'confirm', 'redo', or 'correct_color'. Pass flip=False when the camera is
        already upright for this capture (see _send_image). ``autocontinue_seconds`` > 0
        makes the page auto-confirm on expiry; <= 0 waits for the user.
        """
        self.current_page = "detection_confirm"
        self._send_message({"state": "detection_confirm", "status": "jump", "detection_type": detection_type})
        time.sleep(0.5)
        self._send_message({"state": "detection_confirm", "status": "info", "detection_type": detection_type,
                            "autocontinue_seconds": float(autocontinue_seconds)})
        if vis_image is not None:
            self._send_image(vis_image, flip=flip)

        msg_dict = self.get_required_web_interface_message(
            lambda msg_dict: (msg_dict["state"] == "detection_confirm")
        )
        if msg_dict is None:
            return "redo"
        return msg_dict.get("status", "redo")

    #### Color Correction Page ####

    def start_color_correction(self, raw_bgr_image, initial_vis_image=None, initial_color_range: float = 0.1, flip=True) -> None:
        """Navigate to the color correction page and send images.

        raw_bgr_image: the raw BGR camera frame for pixel color picking.
        initial_vis_image: the detection corners vis to pre-populate the result panel.
        flip: pass False when the camera is already upright for this capture (see _send_image).
        """
        self.current_page = "color_correction"
        self._send_message({"state": "color_correction", "status": "jump"})
        time.sleep(0.5)
        self._send_message({"state": "color_correction", "status": "info",
                            "initial_color_range": initial_color_range})
        if raw_bgr_image is not None:
            self._send_image(raw_bgr_image, flip=flip)
        # Pre-populate the result panel with the initial detection visualization.
        if initial_vis_image is not None:
            time.sleep(0.1)
            self._send_message({"state": "color_correction", "status": "detection_success"})
            time.sleep(0.1)
            self._send_image(initial_vis_image, flip=flip)

    def wait_for_color_correction_message(self) -> dict:
        """Block until the color correction page sends a message."""
        return self.get_required_web_interface_message(
            lambda msg_dict: (msg_dict.get("state") == "color_correction")
        )

    def send_color_correction_result(self, vis_image, success: bool, flip=True) -> None:
        """Send a detection result image (or failure notice) to the color correction page.

        flip: pass False when the camera is already upright for this capture (see _send_image).
        """
        if success:
            self._send_message({"state": "color_correction", "status": "detection_success"})
            time.sleep(0.1)
            if vis_image is not None:
                self._send_image(vis_image, flip=flip)
        else:
            self._send_message({"state": "color_correction", "status": "detection_failed"})

    def update_color_correction_pick_image(self, raw_bgr_image, flip=True) -> None:
        """Replace the pixel-picking image with a newer frame.

        Unlike start_color_correction, this updates the frame used for color
        picking WITHOUT disturbing the result currently shown on the page: the
        "pick_update" status tells the frontend to store the next image as the
        pick image only (used by Reset and the next click) and not redraw.
        flip: pass False when the camera is already upright for this capture (see _send_image).
        """
        if raw_bgr_image is None:
            return
        self._send_message({"state": "color_correction", "status": "pick_update"})
        time.sleep(0.1)
        self._send_image(raw_bgr_image, flip=flip)

    #### Transparency Pages ####

    def get_transparency_request(self) -> None:
        
        if self.current_page != "transparency":
            self.current_page = "transparency"
            # Jump to transparency query page
            self._send_message({"state": "transparency", "status": "jump"})

        msg_dict = self.get_required_web_interface_message(
            lambda msg_dict: (
                (msg_dict["state"] == "transparency_request")
            )
        )

        if msg_dict is None:
            return None
        return msg_dict["status"]
    
    def update_transparency_response(self, response: str) -> None:
        assert self.current_page == "transparency", "Cannot update transparency response when not on the transparency page."
        self._send_message({"state": "transparency_response", "status": response})

    #### Adaptability Pages ####
    
    def get_adaptability_request(self) -> None:
        
        if self.current_page != "adaptability":
            self.current_page = "adaptability"
            # Jump to adaptability query page
            self._send_message({"state": "adaptability", "status": "jump"})

        # Wait until the user provides an adaptability query
        msg_dict = self.get_required_web_interface_message(
            lambda msg_dict: (
                (msg_dict["state"] == "adaptability_request")
            )
        )
        print("Received adaptability request: ", msg_dict)

        # reset the response text
        self._send_message({"state": "adaptability_response", "status": ""})

        if msg_dict is None:
            return None
        return msg_dict["status"]
    
    def update_adaptability_response(self, response: str) -> None:
        assert self.current_page == "adaptability", "Cannot update adaptability response when not on the adaptability page."
        self._send_message({"state": "adaptability_response", "status": response})

    #### Preference Context Page ####

    def get_meal_context(
        self,
        meals: list[str],
        settings: list[str],
        times_of_day: list[str],
        defaults: dict[str, str] | None = None,
    ) -> dict[str, str]:
        """Drive the preference_context page to collect observable meal context.

        Sends the option lists, waits for the user's meal/setting/time_of_day.
        Mirrors the ready-handshake used by the correction page so the
        (non-latched) data can't race the page's subscription.
        """
        self.current_page = "preference_context"
        self.clear_received_messages()
        jump_msg = {"state": "preference_context", "status": "jump"}
        self._send_message(jump_msg)
        # Re-publish the jump while waiting: /robot_to_webapp is not latched and
        # the page re-subscribes asynchronously on mount, so the first jump can be
        # missed. Resend until the page mounts and reports ready (matches the
        # correction page; without it a dropped jump hangs here indefinitely).
        self.get_required_web_interface_message(
            lambda m: (
                m.get("state") == "preference_context"
                and m.get("status") == "ready"
            ),
            resend=lambda: self._send_message(jump_msg),
        )
        self._send_message({
            "state": "preference_context_data",
            "meals": list(meals),
            "settings": list(settings),
            "time_of_day": list(times_of_day),
            "defaults": defaults or {},
        })
        msg = self.get_required_web_interface_message(
            lambda m: m.get("state") == "preference_context_response"
        )
        return {
            "meal": msg["meal"],
            "setting": msg["setting"],
            "time_of_day": msg["time_of_day"],
        }

    #### Navigation Position Adjustment ####

    def get_nav_position_adjust_choice(self, location: str, autocontinue_seconds: float) -> str:
        """Post-arrival prompt: ask whether the robot's parked position at
        ``location`` is OK or the user wants to fine-adjust it via teleop.

        Returns "ok" or "adjust". The page auto-answers "ok" after
        ``autocontinue_seconds`` so an unattended navigation never stalls.
        Mirrors the preference_context ready-handshake (the jump topic is not
        latched, so the jump is re-sent until the page mounts).
        """
        self.current_page = "nav_adjust"
        self.clear_received_messages()
        jump_msg = {
            "state": "nav_adjust",
            "status": "jump",
            "location": str(location),
            "autocontinue_seconds": float(autocontinue_seconds),
        }
        self._send_message(jump_msg)
        self.get_required_web_interface_message(
            lambda m: (
                m.get("state") == "nav_adjust"
                and m.get("status") == "ready"
            ),
            resend=lambda: self._send_message(jump_msg),
        )
        # The page's own subscription may never see the (routing) jump -- the
        # previous page consumed it -- so send the location/countdown data
        # explicitly after the ready handshake (mirrors bite_selection's
        # ready_for_initial_data -> data flow).
        self._send_message({
            "state": "nav_adjust",
            "status": "data",
            "location": str(location),
            "autocontinue_seconds": float(autocontinue_seconds),
        })
        msg = self.get_required_web_interface_message(
            lambda m: (
                m.get("state") == "nav_adjust"
                and m.get("status") in ("ok", "adjust")
            )
        )
        # get_required_web_interface_message can return None on a
        # task-selection jump; treat that as "position OK" (safe no-op).
        return "ok" if msg is None else msg["status"]

    #### Preference Correction Pages ####

    # The preference correction page is driven ONE dimension at a time within a
    # single page mount, so the backend (PreferenceSession) can repredict the
    # still-open dims between steps. Protocol:
    #
    #   start_preference_correction(total, secs):
    #       BE -> app: {"state":"preference_correction","status":"jump",
    #                   "total":M, "autocontinue_seconds":N}
    #       app -> BE: {"state":"preference_correction","status":"ready"}
    #   send_preference_step(...) (called once per dim):
    #       BE -> app: {"state":"preference_correction_data", "field":..,
    #                   "label":.., "predicted":.., "options":[..],
    #                   "step":i, "total":M, "autocontinue_seconds":N}
    #       app -> BE: {"state":"preference_correction_response",
    #                   "field":.., "value":..}
    #   finish_preference_correction():
    #       BE -> app: {"state":"preference_correction","status":"done"}

    def start_preference_correction(self, total: int, autocontinue_seconds: float) -> None:
        """Navigate to the correction page for a stage of ``total`` dims and wait
        until it has mounted/subscribed."""
        # If the user has the settings overlay open, stall until they close it (so
        # they aren't yanked mid-edit and the ask reflects their just-made edits).
        # Mirrors this method's existing get_required_web_interface_message takeover
        # behavior, hence raise_on_takeover=True.
        self.wait_until_settings_closed("robot_waiting", raise_on_takeover=True)
        self.current_page = "preference_correction"
        # Drop stale messages so we wait for the ready for THIS navigation.
        self.clear_received_messages()
        jump_msg = {
            "state": "preference_correction",
            "status": "jump",
            "total": int(total),
            "autocontinue_seconds": float(autocontinue_seconds),
        }
        self._send_message(jump_msg)
        # Re-publish the jump while waiting: /robot_to_webapp is not latched and
        # the target page re-subscribes asynchronously on mount (especially when
        # we just navigated it here from another page, e.g. on resume), so the
        # first jump can be missed. Resend until the page mounts and reports ready.
        self.get_required_web_interface_message(
            lambda msg_dict: (
                msg_dict.get("state") == "preference_correction"
                and msg_dict.get("status") == "ready"
            ),
            resend=lambda: self._send_message(jump_msg),
        )

    def send_preference_step(
        self,
        field: str,
        predicted: str,
        options: list[str],
        step: int,
        total: int,
        autocontinue_seconds: float,
        kind: str = "categorical",
    ) -> str:
        """Show one preference dim (predicted value highlighted) and return the
        user's selection. On autocontinue/no-change the page echoes ``predicted``.

        ``kind`` tells the page how to render the dim: "categorical" shows option
        chips; "text" (e.g. bite_ordering) shows the predicted sentence with an
        "Other..." free text/voice editor."""
        label = _PREF_LABELS.get(field, field.replace("_", " ").title())
        self._send_message({
            "state": "preference_correction_data",
            "field": field,
            "label": label,
            # One plain-language sentence shown as the subtitle under the label
            # (empty string = no subtitle rendered).
            "description": _PREF_SHORT_DESCRIPTIONS.get(field, ""),
            "predicted": predicted,
            "options": list(options),
            "kind": kind,
            "step": int(step),
            "total": int(total),
            "autocontinue_seconds": float(autocontinue_seconds),
        })
        msg_dict = self.get_required_web_interface_message(
            lambda m: (
                m.get("state") == "preference_correction_response"
                and m.get("field") == field
            )
        )
        if msg_dict is None:
            return predicted
        return msg_dict.get("value", predicted)

    def finish_preference_correction(self) -> None:
        """Tell the page the correction stage is done (it returns to its caller)."""
        self._send_message({"state": "preference_correction", "status": "done"})

    #### Gesture Pages ####

    def get_gesture_type(self) -> None:
        """Get whether the user wants to add a gesture or test a gesture."""

        self.current_page = "gesture"

        msg_dict = self.get_required_web_interface_message(
            lambda msg_dict: (
                (msg_dict["state"] == "gesture_menu")
            )
        )

        return msg_dict["status"]
    
    def get_new_gesture_details(self) -> None:
        """Get the gesture label and description from the user."""

        self.current_page = "record_gesture"

        # get first message from web interface (condition is always true)
        msg_dict = self.get_required_web_interface_message(lambda msg_dict: True)

        return msg_dict["state"], msg_dict["status"]
    
    def jump_to_test_gesture_page(self, available_gestures: list[str]) -> None:
        """Send available gestures to the web interface."""
        
        self.current_page = "test_gesture"

        # Jump to gesture test page
        self._send_message({"state": "gesture_test", "status": "jump"})

        # Send available gestures to the web interface
        print("Length of available gestures: ", len(available_gestures))
        print("Available gestures: ", available_gestures)
        time.sleep(0.5)
        self._send_message({"n_ordering": len(available_gestures), "data": available_gestures})
    
    def start_gesture_listener_thread(self, new_gesture_selected_event: threading.Event) -> None:
        """Start the gesture listener thread."""
        assert self.current_page == "test_gesture", "Cannot start gesture listener thread when not on the test gesture page."
        self.new_gesture_selected_event = new_gesture_selected_event
        self.selected_gesture = None
        self.gesture_listener_thread = threading.Thread(target=self.run_gesture_listener_thread)
        self.gesture_listener_thread.start()

    def run_gesture_listener_thread(self) -> None:
        
        while self.active:
            msg_dict = self.get_required_web_interface_message(
                lambda msg_dict: (
                    (msg_dict["state"] == "gesture_test_selection")
                )
            )

            if msg_dict is None or msg_dict["status"] == "back":
                self.selected_gesture = None
                self.new_gesture_selected_event.set()
                break

            self.selected_gesture = msg_dict["status"]
            self.new_gesture_selected_event.set()

    def get_selected_gesture(self) -> None:
        """Get the gesture selected by the user."""
        if self.selected_gesture:
            return self.selected_gesture

    def stop_gesture_listener_thread(self) -> None:
        """Stop the gesture listener thread."""
        self.gesture_listener_thread.join()

    def register_positive_gesture_detection(self) -> None:
        """Register a positive gesture detection."""
        self._send_message({"state": "gesture_response", "status": "Detected the selected gesture."})

    def register_negative_gesture_detection(self) -> None:
        """Register a negative gesture detection."""
        self._send_message({"state": "gesture_response", "status": "Did not detect the selected gesture."})

    def get_gesture_examples(self) -> None:
        """Get gesture examples from the user."""
        
        # Jump to gesture recording page
        self._send_message({"state": "gesture_record", "status": "jump"})

        positive_timestamps = self.record_gesture_examples()
        negative_timestamps = self.record_gesture_examples(positive=False)

        return positive_timestamps, negative_timestamps

    def record_gesture_examples(self, positive=True) -> None:
        """Record gesture examples."""

        if positive:
            trigger_message = "gesture_record_positive"
        else:
            trigger_message = "gesture_record_negative"

        timestamps = []
        start_timestamp, end_timestamp = None, None
        while self.active:
            msg_dict = self.get_required_web_interface_message(
                lambda msg_dict: (
                    (msg_dict["state"] == trigger_message)
                )
            )
            if msg_dict is None:
                break

            if msg_dict["status"] == "start":
                if start_timestamp is not None:
                    self._send_message({"state": "gesture_response", "status": "Invalid example: received start before stop. Please click on stop and then click on start."})
                    print("Invalid example: received start before stop. Please click on stop and then click on start.")
                else:
                    start_timestamp = time.time()
                    # rostopic pub -1 /ServerComm std_msgs/String "{data: '{\"state\": \"gesture_response\", \"status\": \"This is a robot message\"}'}"
                    self._send_message({"state": "gesture_response", "status": f"Started recording gesture example: {len(timestamps) + 1}"})
                    print("Started recording gesture example: ", len(timestamps) + 1)
            elif msg_dict["status"] == "stop":
                if start_timestamp is not None:
                    end_timestamp = time.time()
                    self._send_message({"state": "gesture_response", "status": f"Recorded gesture example: {len(timestamps) + 1}"})
                    print("Recorded gesture example: ", len(timestamps) + 1)
                    timestamps.append((start_timestamp, end_timestamp))
                else:
                    self._send_message({"state": "gesture_response", "status": "Invalid example: received stop before start. Please click on start, then demonstrate the gesture, and then click on stop."})
                    print("Invalid example: received stop before start. Please click on start, then demonstrate the gesture, and then click on stop.")
                start_timestamp, end_timestamp = None, None
            elif msg_dict["status"] == "delete":
                if start_timestamp is None:
                    if len(timestamps) > 0:
                        self._send_message({"state": "gesture_response", "status": f"Deleted gesture example: {len(timestamps)}. {len(timestamps)-1} examples remain in recording history"})
                        print("Deleted gesture example: ", len(timestamps))
                        timestamps.pop()
                        print(f"{len(timestamps)} examples remain in recording history")
                    else:
                        self._send_message({"state": "gesture_response", "status": "No examples to delete"})
                        print("No examples to delete")
                else:
                    self._send_message({"state": "gesture_response", "status": "Cannot delete while recording. Please click on stop and then click on delete."})
                    print("Cannot delete while recording. Please click on stop and then click on delete.")
            elif msg_dict["status"] == "next" or msg_dict["status"] == "back":
                break
            else:
                print("Unsupported message received from the web interface: ", msg_dict)
        return timestamps

    #### Continuous Explanations ####

    def fix_explanation(self, explanation: str) -> None:
        # Wait until the lock becomes available and then fix the explanation
        self.explanation_lock.acquire()  # This will block until the lock is available
        self._send_message({"state": "explanation", "status": explanation}, explanation=True)

    def update_fixed_explanation(self, explanation: str) -> None:
        # lock is already acquired, so just update the explanation
        self._send_message({"state": "explanation", "status": explanation}, explanation=True)

    def clear_explanation(self) -> None:
        # Release the lock to allow continuous explanations to proceed
        if self.explanation_lock.locked():
            self.explanation_lock.release()

    def provide_continuous_explanations(self) -> None:
        current_explanation = ""
        while self.active:
            # Only provide continuous explanations if no one has fixed an explanation
            if not self.explanation_lock.locked():
                try:
                    response = self.transparency_continuous.get_explanation()
                    if response != "No new explanation to provide" and response != current_explanation:
                        current_explanation = response
                    self._send_message({"state": "explanation", "status": current_explanation}, explanation=True)
                except Exception as e:
                    # An LLM/API or file error must not kill the explanation thread;
                    # log it and keep going so explanations resume on the next tick.
                    print("Error generating continuous explanation: ", e)
            time.sleep(1.0)  # Provide explanations at a rate of 1 Hz

if __name__ == "__main__":
    rospy.init_node("web_interface")
    from feeding_deployment.integration.data_logger import DataLogger
    log_dir = Path(__file__).parent.parent / "integration" / "log" / "web_interface_log"
    task_selection_queue = queue.Queue()
    web_interface = WebInterface(task_selection_queue, DataLogger(state_dir=log_dir))
    
    with open(log_dir / "food_detection_data.pkl", "rb") as f:
        food_detection_data = pickle.load(f)

    camera_color_data = food_detection_data["camera_color_data"]
    camera_info_data = food_detection_data["camera_info_data"]
    camera_depth_data = food_detection_data["camera_depth_data"]
    food_items = food_detection_data["food_items"]
    bite_ordering_preference = food_detection_data["bite_ordering_preference"]
    items_detection = food_detection_data["items_detection"]

    # remove next_food_item from data
    food_type_to_data = items_detection['food_type_to_bounding_boxes_plate']

    next_food_item = list(food_type_to_data.keys())[0]

    n_food_types = len(food_type_to_data)
    data = [{k: v} for k, v in food_type_to_data.items() if k != next_food_item]
    predicted_bite = {next_food_item: food_type_to_data[next_food_item]}     

    dip_data = ["Chocolate Sauce", "Ketchup", "No dip"]     
    n_dip_food_types = len(dip_data)
    # load acquisition data
    skill_type, skill_params, dip_type = web_interface.get_next_bite_selection(items_detection['plate_image'], n_food_types, data, predicted_bite, n_dip_food_types, dip_data, autocontinue_timeout=20.0)

    print("Skill type: ", skill_type)
    print("Skill params: ", skill_params)
    print("Dip type: ", dip_type)
