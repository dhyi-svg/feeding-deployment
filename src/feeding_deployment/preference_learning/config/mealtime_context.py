from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

SETTINGS = [
    "Personal",
    "Social with person on Left",
    "Social with person in Front",
    "Social with person on Right",
    "Watching TV with TV on Left",
    "Watching TV with TV in Front",
    "Watching TV with TV on Right",
    "Working on laptop with laptop on Left",
    "Working on laptop with laptop in Front",
    "Working on laptop with laptop on Right",
]

TIMES_OF_DAY = [
    "morning", 
    "noon", 
    "evening"
]

@dataclass(frozen=True)
class MealContents:
    label: str
    dippable_items: List[str]
    sauces: List[str]
    storage_condition: str
    intended_serving_temp: str


MEAL_CONTENTS: List[MealContents] = [
    MealContents(
        label="chicken nuggets",
        dippable_items=["chicken nuggets"],
        sauces=[],
        storage_condition="refrigerated_leftover",
        intended_serving_temp="hot",
    ),
    MealContents(
        label="buffalo chicken bites, potato wedges, and ranch dressing",
        dippable_items=["buffalo chicken bites", "potato wedges"],
        sauces=["ranch dressing"],
        storage_condition="refrigerated_leftover",
        intended_serving_temp="hot",
    ),
    MealContents(
        label="strawberries with whipped cream",
        dippable_items=["strawberries"],
        sauces=["whipped cream"],
        storage_condition="refrigerated",
        intended_serving_temp="cold",
    ),
    MealContents(
        label="chicken nuggets, broccoli, and ketchup",
        dippable_items=["chicken nuggets", "broccoli"],
        sauces=["ketchup"],
        storage_condition="refrigerated_leftover",
        intended_serving_temp="hot",
    ),
    MealContents(
        label="general tso's chicken and broccoli",
        dippable_items=["general tso's chicken", "broccoli"],
        sauces=[],
        storage_condition="refrigerated_leftover",
        intended_serving_temp="hot",
    ),
    MealContents(
        label="chicken breast strips and hash brown",
        dippable_items=["chicken breast strips", "hash brown"],
        sauces=[],
        storage_condition="refrigerated_leftover",
        intended_serving_temp="hot",
    ),
    MealContents(
        label="cantaloupes, bananas, watermelon",
        dippable_items=["cantaloupes", "bananas", "watermelon"],
        sauces=[],
        storage_condition="refrigerated",
        intended_serving_temp="cold",
    ),
    MealContents(
        label="bananas, brownies, and chocolate sauce",
        dippable_items=["bananas", "brownies"],
        sauces=["chocolate sauce"],
        storage_condition="refrigerated_leftover",
        intended_serving_temp="warm",
    ),
    MealContents(
        label="bite-sized sandwiches",
        dippable_items=["bite-sized sandwiches"],
        sauces=[],
        storage_condition="refrigerated_leftover",
        intended_serving_temp="warm",
    ),
    MealContents(
        label="breaded fish bites, roasted potatoes, tartar sauce",
        dippable_items=["breaded fish bites", "roasted potatoes"],
        sauces=["tartar sauce"],
        storage_condition="refrigerated_leftover",
        intended_serving_temp="hot",
    ),
    MealContents(
        label="bite-sized pizza and broccoli",
        dippable_items=["bite-sized pizza", "broccoli"],
        sauces=[],
        storage_condition="refrigerated_leftover",
        intended_serving_temp="hot",
    ),
]


MEAL_CONTENTS_BY_LABEL: Dict[str, MealContents] = {
    m.label: m for m in MEAL_CONTENTS
}

MEALS: List[str] = [m.label for m in MEAL_CONTENTS]


# ---------------------------------------------------------------------------
# FLAIR food-item derivation
#
# FLAIR detects/serves food using per-item labels. The old meal_setup flow ran
# user-typed items through an LLM that singularized them ("chicken nuggets" ->
# "chicken nugget", "as you would refer to a single piece"). Now that food items
# come straight from MealContents (which stores plural, human-readable names),
# we reproduce that singularization deterministically (no LLM) so detection
# inputs are unchanged. _singularize_word covers every food word in the catalog
# above; verify with the module's __main__ self-check if the catalog changes.
# ---------------------------------------------------------------------------

# Words the suffix rules below get wrong (singular does not follow the regular
# pattern). Keep this in sync with the catalog if new meals are added.
_SINGULAR_OVERRIDES: Dict[str, str] = {
    "brownies": "brownie",   # not "browny" (-ies->-y is wrong here)
}


def _singularize_word(w: str) -> str:
    lw = w.lower()
    if lw in _SINGULAR_OVERRIDES:
        return _SINGULAR_OVERRIDES[lw]
    if len(w) <= 2 or lw.endswith("ss"):
        return w
    if lw.endswith("ies") and len(w) > 3:
        return w[:-3] + "y"            # strawberries -> strawberry
    for suf in ("ches", "shes", "sses", "xes", "zzes"):
        if lw.endswith(suf):
            return w[:-2]              # sandwiches -> sandwich
    if lw.endswith("oes"):
        return w[:-2]                  # potatoes -> potato
    if lw.endswith("es"):
        return w[:-1]                  # wedges -> wedge, cantaloupes -> cantaloupe
    if lw.endswith("s") and not (lw.endswith("us") or lw.endswith("is")):
        return w[:-1]                  # nuggets -> nugget, bananas -> banana
    return w


def _singularize_phrase(phrase: str) -> str:
    """Singularize the head (last) word of a food phrase, leaving modifiers."""
    parts = phrase.split()
    if not parts:
        return phrase
    parts[-1] = _singularize_word(parts[-1])
    return " ".join(parts)


def food_items_for_flair(meal_label: str) -> Dict[str, List[str]]:
    """FLAIR food items {"solid": [...], "dip": [...]} for a catalog meal.

    Solids are the meal's dippable_items, dips are its sauces, each singularized
    to match the form FLAIR's detector/planner expect. Raises KeyError for a meal
    not in the catalog (callers validate context against MEALS first)."""
    mc = MEAL_CONTENTS_BY_LABEL.get(meal_label)
    if mc is None:
        raise KeyError(
            f"Meal {meal_label!r} is not in the meal catalog; cannot derive food items."
        )
    return {
        "solid": [_singularize_phrase(x) for x in mc.dippable_items],
        "dip": [_singularize_phrase(x) for x in mc.sauces],
    }


if __name__ == "__main__":
    # Self-check: print derived FLAIR food items for every catalog meal.
    for _m in MEALS:
        print(_m, "->", food_items_for_flair(_m))