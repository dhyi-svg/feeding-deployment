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
    #                seeded from / validated against the PositionOffset value in
    #                the per-user navigate BT YAML.
    kind: str = "categorical"

PREFERENCE_BUNDLE: List[PreferenceDim] = [
    PreferenceDim(
        field="microwave_time",
        label="Microwave time",
        options=["no microwave", "1 min", "2 min", "3 min"],
        description="How long food should be reheated before being served. Some users may prefer hotter food, while others prefer food closer to room temperature. Many meals begin refrigerated and are intended to be served warm or hot. Fruit and dessert meals with an intended serving temperature of cold are usually eaten without microwaving."
    ),
    PreferenceDim(
        field="robot_speed",
        label="Robot speed",
        options=["slow", "medium", "fast"],
        description="The speed at which the robot moves. Some users may prefer a slower speed to feel more comfortable, or to help social partners feel comfortable, while others may prefer a faster speed to reduce overall meal time or because they are less sensitive to the robot's movements."
    ),
    PreferenceDim(
        field="skewering_axis",
        label="Skewering axis selection",
        options=["parallel to major axis", "perpendicular to major axis"],
        description="The direction in which the robot inserts the fork into food when acquiring a bite. Parallel skewering can produce narrower bites that may be easier to eat for users with limited mouth opening. Perpendicular skewering can increase bite acquisition success."
    ),
    PreferenceDim(
        field="web_interface_confirmation",
        label="Web interface confirmation",
        options=["yes", "no"],
        description="Whether the system requires explicit confirmation from the user, through the web interface, that bite acquisition has succeeded. This page allows the user to retry bite acquisition if it fails, but some users may prefer to skip this step to reduce interaction time, even if it means the robot might attempt to transfer an empty fork or a fork with a failed bite acquisition. A user might be more comfortable skipping confirmation in certain contexts (e.g., when eating alone in a personal setting) and prefer confirmation in other contexts (e.g., when eating in a social setting with a partner who might feel uncomfortable if the robot repeatedly attempts to transfer an empty fork after failed bite acquisitions). Some users may want to have confirmation in all contexts."
    ),
    PreferenceDim(
        field="transfer_mode",
        label="Outside mouth vs inside mouth transfer",
        options=["outside mouth transfer", "inside mouth transfer"],
        description="How food is delivered to the mouth: outside-mouth transfer (the robot stops just outside the mouth, and the user leans forward to take the bite) versus inside-mouth transfer (the robot inserts the food directly into the mouth). Preference may depend on the user's physical capabilities (e.g., whether they can lean forward comfortably), their comfort with the robot moving close to their mouth, or their affective state (e.g., preferring inside-mouth transfer when fatigued)."
    ),
    PreferenceDim(
        field="outside_mouth_distance",
        label="For outside-mouth transfer: distance from the mouth",
        options=["not applicable", "near", "medium", "far"],
        description="This parameter only applies when transfer_mode is outside mouth transfer. If inside mouth transfer is used, the value should be not applicable. When using outside-mouth transfer, this determines how far from the mouth the robot stops before the user takes the bite. Far refers to the farthest distance the user can comfortably reach, and near refers to very close (2-4 cm away) from the mouth. This depends on the user's comfort with the robot and their affective state (e.g., preferring a closer distance when fatigued)."
    ),
    PreferenceDim(
        field="convey_robot_ready_for_initiating_transfer",
        label="Convey robot is ready for initiating transfer",
        options=["speech", "LED", "speech + LED", "no cue"],
        description="How the robot signals to the user that it is ready to initiate transfer of a bite, sip, or mouth wiping."
    ),
    PreferenceDim(
        field="detect_user_ready_for_initiating_transfer_feeding",
        label="Detect User Ready for Initiating Transfer - FEEDING",
        options=["open mouth", "button", "autocontinue"],
        description="How the robot determines that the user is ready to initiate bite transfer. open mouth: readiness is detected from the user opening their mouth (this can be a challenge in social settings when a user tends to open their mouth for talking); button: the user explicitly presses a physical button (this can be cumbersome if the user is fatigued); autocontinue: the robot proceeds automatically after waiting for a certain timeout."
    ),
    PreferenceDim(
        field="detect_user_ready_for_initiating_transfer_drinking",
        label="Detect User Ready for Initiating Transfer - DRINKING",
        options=["open mouth", "button", "autocontinue"],
        description="How the robot determines that the user is ready to initiate sip transfer. open mouth: readiness is detected from the user opening their mouth (this can be a challenge in social settings when a user tends to open their mouth for talking); button: the user explicitly presses a physical button (this can be cumbersome if the user is fatigued); autocontinue: the robot proceeds automatically after waiting for a certain timeout."
    ),
    PreferenceDim(
        field="detect_user_ready_for_initiating_transfer_wiping",
        label="Detect User Ready for Initiating Transfer - MOUTH WIPING",
        options=["open mouth", "button", "autocontinue"],
        description="How the robot determines that the user is ready to initiate transfer of mouth wiper. open mouth: readiness is detected from the user opening their mouth (this can be a challenge in social settings when a user tends to open their mouth for talking); button: the user explicitly presses a physical button (this can be cumbersome if the user is fatigued); autocontinue: the robot proceeds automatically after waiting for a certain timeout."
    ),
    PreferenceDim(
        field="convey_robot_ready_for_completing_transfer",
        label="Convey robot is ready for completing transfer",
        options=["speech", "LED", "speech + LED", "no cue"],
        description="How the robot signals to the user that the tool has reached the transfer location and the user can complete the transfer by taking a bite, sip, or mouth wiping."
    ),
    PreferenceDim(
        field="detect_user_completed_transfer_feeding",
        label="Detect User Completed Transfer - FEEDING",
        options=["perception", "button", "autocontinue"],
        description="How the robot determines that the user has finished taking a bite. perception: the robot detects completion using a force-torque sensor on the fork (very reliable); button: the user explicitly signals completion by physically pressing a button; autocontinue: the robot proceeds automatically after a certain timeout."
    ),
    PreferenceDim(
        field="detect_user_completed_transfer_drinking",
        label="Detect User Completed Transfer - DRINKING",
        options=["perception", "button", "autocontinue"],
        description="How the robot determines that the user has finished drinking. perception: the robot detects completion using a head-nod gesture (which may feel unnatural in social settings); button: the user explicitly signals completion by physically pressing a button; autocontinue: the robot proceeds automatically after a certain timeout."
    ),
    PreferenceDim(
        field="detect_user_completed_transfer_wiping",
        label="Detect User Completed Transfer - MOUTH WIPING",
        options=["perception", "button", "autocontinue"],
        description="How the robot determines that the user has finished mouth wiping. perception: the robot detects completion using a head-nod gesture (which may feel unnatural in social settings); button: the user explicitly signals completion by physically pressing a button; autocontinue: the robot proceeds automatically after a certain timeout."
    ),
    PreferenceDim(
        field="retract_between_bites",
        label="Retract between bites",
        options=["yes", "no"],
        description="Whether the robot moves to a retract position between tasks to avoid obstructing the user's view. However, moving to this position makes the meal take longer, so some users may prefer to skip this step to reduce meal time, even if it means the robot might obstruct the user's view for a longer duration during the meal. A user might be more comfortable skipping retracting in certain contexts (e.g., when eating alone in a personal setting) and prefer retracting in other contexts (e.g., when working, watching TV, or eating in a social setting with a partner who might feel uncomfortable if the robot obstructs their view for a long duration)."
    ),
    PreferenceDim(
        field="bite_dipping_preference",
        label="Bite dipping preference",
        options=["do not dip", "less", "more"],
        description="How much sauce should be applied when dipping. This depends on the user's personal preference, as well as the context (e.g., some users might prefer more dipping when eating alone in a personal setting and less dipping when eating in a social setting to avoid messiness). Choose do not dip when the user prefers not to dip or when the meal does not have any dippable items or sauces."
    ),
    PreferenceDim(
        field="bite_ordering",
        label="Bite ordering",
        options=[],
        kind="text",
        description="The order in which the food items on the plate should be fed, as a short natural-language instruction grounded in THIS meal's actual foods and dips. Capture both (a) how the solid items are ordered relative to each other (e.g., all of one item before the next, or alternating between items) and (b) which solids should be paired with which dips (e.g., dip a particular item in a particular sauce). Base the prediction on the user's known preferences and the meal contents; if there is no evidence of any ordering or dipping-pairing preference, predict 'no particular order'. Keep it to a single concise sentence. This is separate from bite_dipping_preference, which only controls how much sauce is applied."
    ),
    PreferenceDim(
        field="wait_before_autocontinue_seconds",
        label="Time to wait before autocontinue",
        options=["10 sec", "100 sec", "1000 sec"],
        description="How long the robot waits before automatically continuing to the next bite, sip, or mouth wiping if the user does not intervene. Some users may prefer a shorter wait time to reduce meal time, while others may prefer a longer wait time to give themselves more time to intervene if needed, especially in contexts where they might be more distracted (e.g., when eating in a social setting with a partner or when watching TV)."
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
    # PositionOffset saved in the per-user navigate BT YAML (zero on day 1) and
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
# episode text, and the per-user behavior-tree YAML (HandleColor=[H,S,V],
# ColorRange=range). The factory default matches
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

DEFAULT_COLOR: dict = {"h": 82, "s": 55, "v": 84, "range": 0.1}

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
    """Canonical color dict -> (HandleColor list [H,S,V], ColorRange float)."""
    c = parse_color(c)
    return [c["h"], c["s"], c["v"]], c["range"]


def color_from_bt(handle_color, color_range) -> dict:
    """(HandleColor [H,S,V], ColorRange) from the BT YAML -> canonical color dict."""
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
# the per-user navigate BT YAML (PositionOffset=[dx, dy, dyaw]). The bounds
# must match the PositionOffset Box space in the navigate_to_*.yaml behavior
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
    """Canonical nav-offset dict -> PositionOffset list [dx, dy, dyaw]."""
    o = parse_nav_offset(o)
    return [o["dx"], o["dy"], o["dyaw"]]


def nav_offset_from_bt(value) -> dict:
    """PositionOffset [dx, dy, dyaw] from the BT YAML -> canonical offset dict."""
    v = list(value) if value is not None else []
    v = (v + [0.0, 0.0, 0.0])[:3]
    return parse_nav_offset({"dx": v[0], "dy": v[1], "dyaw": v[2]})


def nav_offsets_equal(a, b, tol: float = 1e-6) -> bool:
    """Float-tolerant equality for change detection. Colors compare with exact
    ``!=`` on ints; offsets are floats round-tripped through YAML, so exact
    comparison would produce spurious 'changed' signals."""
    a, b = parse_nav_offset(a), parse_nav_offset(b)
    return all(abs(a[k] - b[k]) <= tol for k in ("dx", "dy", "dyaw"))