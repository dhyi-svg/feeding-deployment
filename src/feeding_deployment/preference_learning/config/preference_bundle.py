from dataclasses import dataclass
from typing import List, Optional, Tuple

@dataclass(frozen=True)
class PreferenceDim:
    field: str
    label: str
    options: List[str]
    description: str
    # "categorical": value is one of `options` (LLM picks from the list).
    # "color":       value is a continuous HSV color + range (LLM emits
    #                {"h","s","v","range"}); `options` is empty and the dim is
    #                seeded from / validated against the per-user BT YAML color.
    # "nav_offset":  value is a continuous goal-pose correction (LLM emits
    #                {"dx","dy","dyaw"}); `options` is empty and the dim is
    #                seeded from / validated against the ParkingOffset value in
    #                the per-user navigate BT YAML.
    kind: str = "categorical"
    # One plain-language sentence shown as the subtitle under the label on the
    # iPad ask page (the full `description` is written for the LLM and only
    # appears in the settings overlay). Empty = no subtitle.
    short_description: str = ""

PREFERENCE_BUNDLE: List[PreferenceDim] = [
    PreferenceDim(
        field="microwave_time",
        label="Microwave time",
        options=["no microwave", "1 min", "2 min", "3 min"],
        description="How long food should be reheated before being served. Some users may prefer hotter food, while others prefer food closer to room temperature. Many meals begin refrigerated and are intended to be served warm or hot. Fruit and dessert meals with an intended serving temperature of cold are usually eaten without microwaving.",
        short_description="How long to reheat your food before it is served.",
    ),
    PreferenceDim(
        field="robot_speed",
        label="Robot speed",
        options=["slow", "medium", "fast"],
        description="The speed at which the robot moves. Some users may prefer a slower speed to feel more comfortable, or to help social partners feel comfortable, while others may prefer a faster speed to reduce overall meal time or because they are less sensitive to the robot's movements.",
        short_description="How fast the robot moves during the meal.",
    ),
    PreferenceDim(
        field="skewering_axis",
        label="Skewering axis selection",
        options=["parallel to major axis", "perpendicular to major axis"],
        description="The direction in which the robot inserts the fork into food when acquiring a bite. Parallel skewering can produce narrower bites that may be easier to eat for users with limited mouth opening. Perpendicular skewering can increase bite acquisition success.",
        short_description="How the fork is angled when picking up a bite.",
    ),
    PreferenceDim(
        field="confirm_feeding_pickup",
        label="Should the robot check each bite, sip, or wipe pickup before bringing it to you?",
        options=["no", "yes (with auto-continue countdown)", "yes (without any auto-continue)"],
        short_description="",
        description="Whether the robot shows a web-interface confirmation page after picking up the bite, drink, or mouth wipe, before bringing it toward the user. These pages let the user retry a failed pickup (e.g., an empty fork). 'no' skips the pages entirely to reduce interaction time, even if it means the robot might transfer a failed pickup; 'yes (with auto-continue countdown)' shows the page but proceeds automatically after the auto-continue wait (wait_before_autocontinue_feeding_pickup) if the user does not intervene; 'yes (without any auto-continue)' waits indefinitely for the user's explicit confirmation. A user might be comfortable skipping confirmation in some contexts (e.g., eating alone in a personal setting) and prefer it in others (e.g., a social setting where repeated empty-fork transfers would be awkward). Users who trust the robot more tend to relax confirmation across the board."
    ),
    PreferenceDim(
        field="confirm_navigation_arrival",
        label="Should the robot check its parking after driving?",
        options=["no", "yes (with auto-continue countdown)", "yes (without any auto-continue)"],
        short_description="",
        description="Whether the robot shows the position check page after driving to a location (fridge, microwave, table, sink), where the user can approve the parked position or fine-adjust it by teleoperating the base. This page is also how the robot learns the user's preferred parking spots over time. 'no' skips the page and drives on with the learned parking positions (they stop being refined); 'yes (with auto-continue countdown)' shows the page but accepts the position automatically after the auto-continue wait (wait_before_autocontinue_mealprep); 'yes (without any auto-continue)' waits indefinitely. Users typically start with confirmation on and relax it as the robot's parking proves reliable."
    ),
    PreferenceDim(
        field="confirm_manipulation",
        label="During meal prep and cleaning: should the robot check before grabbing, pressing, or placing things?",
        options=["no", "yes (with auto-continue countdown)", "yes (without any auto-continue)"],
        short_description="",
        description="Whether the robot shows web-interface confirmation pages around manipulation: verifying a detection before acting on it (the plate handle before a pickup, the fridge/microwave door handle before opening, the microwave button before pressing, the placement spot before setting the plate down) and confirming it is safe to release the plate at the microwave, table, or sink. The detection pages are also where the user can redo a detection or correct the plate-handle color. 'no' skips these pages (successful detections are accepted automatically and the plate is released without asking; detection colors stop being refined); 'yes (with auto-continue countdown)' shows each page but proceeds automatically after the auto-continue wait (wait_before_autocontinue_mealprep); 'yes (without any auto-continue)' waits indefinitely. Users who trust the robot's perception tend to relax this over time."
    ),
    PreferenceDim(
        field="transfer_mode",
        label="Outside mouth vs inside mouth transfer",
        options=["outside mouth transfer", "inside mouth transfer"],
        description="How food is delivered to the mouth: outside-mouth transfer (the robot stops just outside the mouth, and the user leans forward to take the bite) versus inside-mouth transfer (the robot inserts the food directly into the mouth). Preference may depend on the user's physical capabilities (e.g., whether they can lean forward comfortably), their comfort with the robot moving close to their mouth, or their affective state (e.g., preferring inside-mouth transfer when fatigued). This deployment only performs outside-mouth transfer; predict 'inside mouth transfer' only if the user has explicitly asked for it.",
        short_description="Whether food stops just outside your mouth or is placed inside your mouth.",
    ),
    PreferenceDim(
        field="outside_mouth_distance",
        label="For outside-mouth transfer: distance from the mouth",
        options=["not applicable", "near", "medium", "far"],
        description="This parameter only applies when transfer_mode is outside mouth transfer. If inside mouth transfer is used, the value should be not applicable. When using outside-mouth transfer, this determines how far from the mouth the robot stops before the user takes the bite. Far refers to the farthest distance the user can comfortably reach, and near refers to very close (2-4 cm away) from the mouth. This depends on the user's comfort with the robot and their affective state (e.g., preferring a closer distance when fatigued).",
        short_description="How far from your mouth the robot stops when it feeds outside the mouth.",
    ),
    PreferenceDim(
        field="convey_robot_ready_for_initiating_transfer",
        label="How should the robot signal it's ready to bring a bite, sip, or wipe?",
        options=["speech", "LED", "speech + LED", "no cue"],
        description="How the robot signals to the user that it is ready to initiate transfer of a bite, sip, or mouth wiping.",
        short_description="",
    ),
    PreferenceDim(
        field="detect_user_ready_for_initiating_transfer_feeding",
        label="How do you tell the robot you're ready for a bite?",
        options=["open mouth", "button", "autocontinue"],
        description="How the robot determines that the user is ready to initiate bite transfer. open mouth: readiness is detected from the user opening their mouth (this can be a challenge in social settings when a user tends to open their mouth for talking); button: the user explicitly presses a physical button (this can be cumbersome if the user is fatigued); autocontinue: the robot proceeds automatically after waiting for a certain timeout.",
        short_description="",
    ),
    PreferenceDim(
        field="detect_user_ready_for_initiating_transfer_drinking",
        label="How do you tell the robot you're ready for a sip?",
        options=["open mouth", "button", "autocontinue"],
        description="How the robot determines that the user is ready to initiate sip transfer. open mouth: readiness is detected from the user opening their mouth (this can be a challenge in social settings when a user tends to open their mouth for talking); button: the user explicitly presses a physical button (this can be cumbersome if the user is fatigued); autocontinue: the robot proceeds automatically after waiting for a certain timeout.",
        short_description="",
    ),
    PreferenceDim(
        field="detect_user_ready_for_initiating_transfer_wiping",
        label="How do you tell the robot you're ready for a mouth wipe?",
        options=["open mouth", "button", "autocontinue"],
        description="How the robot determines that the user is ready to initiate transfer of mouth wiper. open mouth: readiness is detected from the user opening their mouth (this can be a challenge in social settings when a user tends to open their mouth for talking); button: the user explicitly presses a physical button (this can be cumbersome if the user is fatigued); autocontinue: the robot proceeds automatically after waiting for a certain timeout.",
        short_description="",
    ),
    PreferenceDim(
        field="convey_robot_ready_for_completing_transfer",
        label="How should the robot signal you can take the bite, sip, or wipe?",
        options=["speech", "LED", "speech + LED", "no cue"],
        description="How the robot signals to the user that the tool has reached the transfer location and the user can complete the transfer by taking a bite, sip, or mouth wiping.",
        short_description="",
    ),
    PreferenceDim(
        field="detect_user_completed_transfer_feeding",
        label="How does the robot know you've finished a bite?",
        options=["perception", "button", "autocontinue"],
        description="How the robot determines that the user has finished taking a bite. perception: the robot detects completion using a force-torque sensor on the fork (very reliable); button: the user explicitly signals completion by physically pressing a button; autocontinue: the robot proceeds automatically after a certain timeout.",
        short_description="",
    ),
    PreferenceDim(
        field="detect_user_completed_transfer_drinking",
        label="How does the robot know you've finished a sip?",
        options=["perception", "button", "autocontinue"],
        description="How the robot determines that the user has finished drinking. perception: the robot detects completion using a head-nod gesture (which may feel unnatural in social settings); button: the user explicitly signals completion by physically pressing a button; autocontinue: the robot proceeds automatically after a certain timeout.",
        short_description="",
    ),
    PreferenceDim(
        field="detect_user_completed_transfer_wiping",
        label="How does the robot know you've finished wiping?",
        options=["perception", "button", "autocontinue"],
        description="How the robot determines that the user has finished mouth wiping. perception: the robot detects completion using a head-nod gesture (which may feel unnatural in social settings); button: the user explicitly signals completion by physically pressing a button; autocontinue: the robot proceeds automatically after a certain timeout.",
        short_description="",
    ),
    PreferenceDim(
        field="retract_between_bites",
        label="Retract between bites",
        options=["yes", "no"],
        description="Whether the robot moves to a retract position between tasks to avoid obstructing the user's view. However, moving to this position makes the meal take longer, so some users may prefer to skip this step to reduce meal time, even if it means the robot might obstruct the user's view for a longer duration during the meal. A user might be more comfortable skipping retracting in certain contexts (e.g., when eating alone in a personal setting) and prefer retracting in other contexts (e.g., when working, watching TV, or eating in a social setting with a partner who might feel uncomfortable if the robot obstructs their view for a long duration).",
        short_description="Whether the robot moves out of your view between bites.",
    ),
    PreferenceDim(
        field="bite_dipping_preference",
        label="Bite dipping preference",
        options=["do not dip", "less", "more"],
        description="How much sauce should be applied when dipping. This depends on the user's personal preference, as well as the context (e.g., some users might prefer more dipping when eating alone in a personal setting and less dipping when eating in a social setting to avoid messiness). Choose do not dip when the user prefers not to dip or when the meal does not have any dippable items or sauces.",
        short_description="How much sauce to apply when dipping your food.",
    ),
    PreferenceDim(
        field="bite_ordering",
        label="Bite ordering",
        options=[],
        kind="text",
        description="The order in which the food items on the plate should be fed, as a short natural-language instruction grounded in THIS meal's actual foods and dips. Capture both (a) how the solid items are ordered relative to each other (e.g., all of one item before the next, or alternating between items) and (b) which solids should be paired with which dips (e.g., dip a particular item in a particular sauce). Base the prediction on the user's known preferences and the meal contents; if there is no evidence of any ordering or dipping-pairing preference, predict 'no particular order'. Keep it to a single concise sentence. This is separate from bite_dipping_preference, which only controls how much sauce is applied.",
        short_description="The order your foods are fed, and which dips go with which items.",
    ),
    PreferenceDim(
        field="wait_before_autocontinue_task_selection",
        label="After a bite or sip: how long before auto-selecting the next task?",
        options=["15 sec", "30 sec", "60 sec", "no autocontinue"],
        short_description="",
        description="How long the robot waits on the next-task page after finishing a bite or a sip before automatically starting another of the same (the page pre-selects 'take a bite' after a bite and 'take a sip' after a sip). 'no autocontinue' means the page never advances on its own — the robot waits for the user to pick the next task, useful for users who chat between bites or dislike being rushed; a shorter wait keeps the meal moving for users who eat steadily. This only governs the between-tasks page; the bite-selection and pickup-check waits are wait_before_autocontinue_feeding_pickup, and the meal-preparation check pages are wait_before_autocontinue_mealprep."
    ),
    PreferenceDim(
        field="wait_before_autocontinue_feeding_pickup",
        label="Bite choice and pickup confirmation checks: how long before auto-continuing?",
        options=["15 sec", "30 sec", "60 sec", "no autocontinue"],
        short_description="",
        description="How long the robot waits on the bite-selection page (where the user can confirm or change the predicted next bite) and on the tool (bite/drink/wipe) pickup-check pages (shown when confirm_feeding_pickup is 'yes (with auto-continue countdown)') before proceeding automatically. 'no autocontinue' means these pages wait indefinitely for the user's answer; on the pickup checks that is equivalent to confirm_feeding_pickup's 'yes (without any auto-continue)'. Users who trust the robot's bite predictions prefer a short wait to reduce meal time; users who like to choose each bite, or are often distracted (e.g., social settings, watching TV), prefer a longer wait or no autocontinue. This is separate from wait_before_autocontinue_task_selection (the between-tasks page) and wait_before_autocontinue_mealprep (the meal-preparation check pages)."
    ),
    PreferenceDim(
        field="wait_before_autocontinue_mealprep",
        label="Meal prep and cleaning check pages: how long before auto-continuing?",
        options=["15 sec", "30 sec", "60 sec"],
        short_description="",
        description="How long the robot waits before automatically continuing on the meal-preparation check pages set to 'yes (with auto-continue countdown)': the post-navigation position check (confirm_navigation_arrival) and the manipulation detection and plate-release checks (confirm_manipulation). The preference question pages themselves also use this wait. There is no 'no autocontinue' option here — a user who wants the robot to wait indefinitely on these pages sets the corresponding confirmation preference to 'yes (without any auto-continue)'. Shorter waits keep meal preparation moving; longer waits give more time to intervene when distracted. The feeding-side waits are wait_before_autocontinue_task_selection and wait_before_autocontinue_feeding_pickup."
    ),
    # --- Color dimensions (kind="color") -------------------------------------
    # The robot picks up the plate by detecting a colored handle. The detection
    # color is an HSV value (+ a tolerance range). These three dims are the
    # SAME physical plate handle seen at three pickup locations under different
    # lighting/backgrounds (fridge, microwave, table), so a correction at one
    # location is strong evidence for the others. Values are continuous HSV, not
    # a fixed option-list: predictions are seeded from the user's current saved
    # color in the dynamic behavior tree (factory default on day 1) and the user
    # corrects them with the on-screen color picker during pickup.
    PreferenceDim(
        field="plate_color_fridge",
        label="Plate handle color (fridge pickup)",
        options=[],
        kind="color",
        description="HSV color of the plate handle used for attachment detection when picking the plate up from inside the fridge. This is the same physical plate handle as plate_color_microwave and plate_color_table, but the fridge interior lighting may shift its apparent color. Predict the handle's HSV color and a tolerance range; the provided seed is the currently saved value from previous meals -- a reasonable default, to be weighed against this meal's corrections and similar prior meals rather than kept unconditionally."
    ),
    PreferenceDim(
        field="plate_color_microwave",
        label="Plate handle color (microwave pickup)",
        options=[],
        kind="color",
        description="HSV color of the plate handle used for attachment detection when picking the plate up from inside the microwave. This is the same physical plate handle as plate_color_fridge and plate_color_table, but microwave interior lighting may shift its apparent color. Predict the handle's HSV color and a tolerance range; the provided seed is the currently saved value from previous meals -- a reasonable default, to be weighed against this meal's corrections and similar prior meals rather than kept unconditionally."
    ),
    PreferenceDim(
        field="plate_color_table",
        label="Plate handle color (table pickup)",
        options=[],
        kind="color",
        description="HSV color of the plate handle used for attachment detection when picking the plate up from the table. This is the same physical plate handle as plate_color_fridge and plate_color_microwave, but table lighting may shift its apparent color. Predict the handle's HSV color and a tolerance range; the provided seed is the currently saved value from previous meals -- a reasonable default, to be weighed against this meal's corrections and similar prior meals rather than kept unconditionally."
    ),
    # --- Navigation-offset dimensions (kind="nav_offset") --------------------
    # Offsets arise between the mapped named locations and where the robot
    # actually parks (and where the user actually wants it). After autonomous
    # navigation to a destination the user may teleoperate the base to
    # fine-adjust its position; the measured adjustment accumulates into a
    # per-location TOTAL offset {"dx","dy","dyaw"} expressed in the stored goal
    # pose's local frame (dx meters forward, dy meters left, dyaw radians
    # counter-clockwise). The next navigation to that location composes the
    # offset onto the nominal goal. Predictions are seeded from the current
    # ParkingOffset saved in the per-user navigate BT YAML (zero on day 1) and
    # the user corrects them physically, not through a form. Unlike the plate
    # colors, the four locations are independent: a correction at one location
    # is only weak evidence for the others.
    PreferenceDim(
        field="nav_offset_table",
        label="Navigation offset (table)",
        options=[],
        kind="nav_offset",
        description="Learned correction to the robot's parking pose when it navigates to the table, expressed in the stored goal pose's local frame: dx (meters, forward), dy (meters, left), dyaw (radians, counter-clockwise). After autonomous navigation the user may teleoperate the base to fine-adjust its position; the measured adjustment accumulates into this total offset, and the next navigation to the table applies it to the goal. Each location has its own independent offset. Predict the current total offset; the provided seed is the accumulated offset saved so far -- a reasonable default, to be weighed against this meal's corrections and similar prior meals rather than kept unconditionally."
    ),
    PreferenceDim(
        field="nav_offset_microwave",
        label="Navigation offset (microwave)",
        options=[],
        kind="nav_offset",
        description="Learned correction to the robot's parking pose when it navigates to the microwave, expressed in the stored goal pose's local frame: dx (meters, forward), dy (meters, left), dyaw (radians, counter-clockwise). After autonomous navigation the user may teleoperate the base to fine-adjust its position; the measured adjustment accumulates into this total offset, and the next navigation to the microwave applies it to the goal. Each location has its own independent offset. Predict the current total offset; the provided seed is the accumulated offset saved so far -- a reasonable default, to be weighed against this meal's corrections and similar prior meals rather than kept unconditionally."
    ),
    PreferenceDim(
        field="nav_offset_sink",
        label="Navigation offset (sink)",
        options=[],
        kind="nav_offset",
        description="Learned correction to the robot's parking pose when it navigates to the sink, expressed in the stored goal pose's local frame: dx (meters, forward), dy (meters, left), dyaw (radians, counter-clockwise). After autonomous navigation the user may teleoperate the base to fine-adjust its position; the measured adjustment accumulates into this total offset, and the next navigation to the sink applies it to the goal. Each location has its own independent offset. Predict the current total offset; the provided seed is the accumulated offset saved so far -- a reasonable default, to be weighed against this meal's corrections and similar prior meals rather than kept unconditionally."
    ),
    PreferenceDim(
        field="nav_offset_fridge",
        label="Navigation offset (fridge)",
        options=[],
        kind="nav_offset",
        description="Learned correction to the robot's parking pose when it navigates to the fridge, expressed in the stored goal pose's local frame: dx (meters, forward), dy (meters, left), dyaw (radians, counter-clockwise). After autonomous navigation the user may teleoperate the base to fine-adjust its position; the measured adjustment accumulates into this total offset, and the next navigation to the fridge applies it to the goal. Each location has its own independent offset. Predict the current total offset; the provided seed is the accumulated offset saved so far -- a reasonable default, to be weighed against this meal's corrections and similar prior meals rather than kept unconditionally."
    ),
]

# ---------------------------------------------------------------------------
# Color-dimension helpers
#
# A color value is the canonical dict {"h": int, "s": int, "v": int,
# "range": float} (HSV, H in [0,179], S/V in [0,255], range in [0,1]). This is
# what the LLM emits for kind="color" dims and what flows through the bundle,
# episode text, and the per-user behavior-tree YAML (PlateHandleColor=[H,S,V],
# PlateHandleColorTolerance=range). The factory default matches
# AttachmentPerception.detect_attachment_color.
# ---------------------------------------------------------------------------

COLOR_FIELDS: List[str] = [dim.field for dim in PREFERENCE_BUNDLE if dim.kind == "color"]

# ---------------------------------------------------------------------------
# Text-dimension helpers
#
# A text value is a free-form natural-language string (the LLM emits it for
# kind="text" dims). It flows through the bundle, episode text, and -- for
# bite_ordering -- into FLAIR's preference planner as the user's bite-ordering
# instruction. Unlike categorical dims there is no option list to validate
# against; an empty/missing prediction falls back to DEFAULT_BITE_ORDERING so
# the value is never None (FLAIR.is_preference_set() keys off non-None).
# ---------------------------------------------------------------------------

TEXT_FIELDS: List[str] = [dim.field for dim in PREFERENCE_BUNDLE if dim.kind == "text"]

DEFAULT_BITE_ORDERING: str = "no particular order"

DEFAULT_COLOR: dict = {"h": 12, "s": 223, "v": 169, "range": 0.1}

# pickup location <-> color field (location names match HLA/perception usage).
COLOR_FIELD_BY_LOCATION: dict = {
    "fridge": "plate_color_fridge",
    "microwave": "plate_color_microwave",
    "table": "plate_color_table",
}


def _clip_int(x, lo: int, hi: int, default: int) -> int:
    try:
        return max(lo, min(hi, int(round(float(x)))))
    except (TypeError, ValueError):
        return default


def _clip_float(x, lo: float, hi: float, default: float) -> float:
    try:
        return max(lo, min(hi, float(x)))
    except (TypeError, ValueError):
        return default


def parse_color(obj, seed: Optional[dict] = None) -> dict:
    """Coerce an LLM/JSON value into a canonical, clipped color dict.

    Accepts a dict with any of h/s/v/range, or a string like
    "h=82,s=55,v=84,range=0.10". Missing/invalid components fall back to
    ``seed`` (or DEFAULT_COLOR). Always returns a fully-populated, clipped dict.
    """
    base = dict(seed) if seed else dict(DEFAULT_COLOR)

    parsed: dict = {}
    if isinstance(obj, dict):
        parsed = obj
    elif isinstance(obj, str):
        for part in obj.replace(";", ",").split(","):
            if "=" in part:
                k, _, v = part.partition("=")
                parsed[k.strip().lower()] = v.strip()

    return {
        "h": _clip_int(parsed.get("h", base["h"]), 0, 179, int(base["h"])),
        "s": _clip_int(parsed.get("s", base["s"]), 0, 255, int(base["s"])),
        "v": _clip_int(parsed.get("v", base["v"]), 0, 255, int(base["v"])),
        "range": _clip_float(parsed.get("range", base["range"]), 0.0, 1.0, float(base["range"])),
    }


def format_color(c: dict) -> str:
    """Stable compact string for episode text / logs: ``h=82,s=55,v=84,range=0.10``."""
    c = parse_color(c)
    return f"h={c['h']},s={c['s']},v={c['v']},range={c['range']:.2f}"


def color_to_bt(c: dict):
    """Canonical color dict -> (PlateHandleColor list [H,S,V], PlateHandleColorTolerance float)."""
    c = parse_color(c)
    return [c["h"], c["s"], c["v"]], c["range"]


def color_from_bt(handle_color, color_range) -> dict:
    """(PlateHandleColor [H,S,V], PlateHandleColorTolerance) from the BT YAML -> canonical color dict."""
    hc = list(handle_color) if handle_color is not None else [DEFAULT_COLOR["h"], DEFAULT_COLOR["s"], DEFAULT_COLOR["v"]]
    if len(hc) < 3:
        hc = (hc + [0, 0, 0])[:3]
    return parse_color({"h": hc[0], "s": hc[1], "v": hc[2], "range": color_range})


# ---------------------------------------------------------------------------
# Navigation-offset-dimension helpers
#
# A nav offset value is the canonical dict {"dx": float, "dy": float,
# "dyaw": float} (meters, meters, radians; expressed in the goal pose's local
# frame, each clipped to +/- NAV_OFFSET_BOUNDS). This is what the LLM emits for
# kind="nav_offset" dims and what flows through the bundle, episode text, and
# the per-user navigate BT YAML (ParkingOffset=[dx, dy, dyaw]). The bounds
# must match the ParkingOffset Box space in the navigate_to_*.yaml behavior
# trees and NavigateHLA's clamp.
# ---------------------------------------------------------------------------

NAV_OFFSET_FIELDS: List[str] = [dim.field for dim in PREFERENCE_BUNDLE if dim.kind == "nav_offset"]

DEFAULT_NAV_OFFSET: dict = {"dx": 0.0, "dy": 0.0, "dyaw": 0.0}

NAV_OFFSET_BOUNDS: dict = {"dx": 0.5, "dy": 0.5, "dyaw": 0.7853981633974483}  # +/- 0.5 m, +/- 45 deg

# navigation location <-> offset field (location names match navigate_to_<loc>.yaml).
OFFSET_FIELD_BY_LOCATION: dict = {
    "table": "nav_offset_table",
    "microwave": "nav_offset_microwave",
    "sink": "nav_offset_sink",
    "fridge": "nav_offset_fridge",
}


def parse_nav_offset(obj, seed: Optional[dict] = None) -> dict:
    """Coerce an LLM/JSON value into a canonical, clipped nav-offset dict.

    Accepts a dict with any of dx/dy/dyaw, or a string like
    "dx=0.050,dy=-0.020,dyaw=0.030". Missing/invalid components fall back to
    ``seed`` (or DEFAULT_NAV_OFFSET). Always returns a fully-populated, clipped
    dict.
    """
    base = dict(seed) if seed else dict(DEFAULT_NAV_OFFSET)

    parsed: dict = {}
    if isinstance(obj, dict):
        parsed = obj
    elif isinstance(obj, str):
        for part in obj.replace(";", ",").split(","):
            if "=" in part:
                k, _, v = part.partition("=")
                parsed[k.strip().lower()] = v.strip()

    return {
        k: _clip_float(parsed.get(k, base[k]), -NAV_OFFSET_BOUNDS[k], NAV_OFFSET_BOUNDS[k], float(base[k]))
        for k in ("dx", "dy", "dyaw")
    }


def format_nav_offset(o: dict) -> str:
    """Stable compact string for episode text / logs: ``dx=0.050,dy=-0.020,dyaw=0.030``."""
    o = parse_nav_offset(o)
    return f"dx={o['dx']:.3f},dy={o['dy']:.3f},dyaw={o['dyaw']:.3f}"


def nav_offset_to_bt(o: dict) -> list:
    """Canonical nav-offset dict -> ParkingOffset list [dx, dy, dyaw]."""
    o = parse_nav_offset(o)
    return [o["dx"], o["dy"], o["dyaw"]]


def nav_offset_from_bt(value) -> dict:
    """ParkingOffset [dx, dy, dyaw] from the BT YAML -> canonical offset dict."""
    v = list(value) if value is not None else []
    v = (v + [0.0, 0.0, 0.0])[:3]
    return parse_nav_offset({"dx": v[0], "dy": v[1], "dyaw": v[2]})


def nav_offsets_equal(a, b, tol: float = 1e-6) -> bool:
    """Float-tolerant equality for change detection. Colors compare with exact
    ``!=`` on ints; offsets are floats round-tripped through YAML, so exact
    comparison would produce spurious 'changed' signals."""
    a, b = parse_nav_offset(a), parse_nav_offset(b)
    return all(abs(a[k] - b[k]) <= tol for k in ("dx", "dy", "dyaw"))