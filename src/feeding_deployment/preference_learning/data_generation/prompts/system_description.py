from pathlib import Path
from typing import List

from feeding_deployment.preference_learning.config.preference_bundle import PreferenceDim, PREFERENCE_BUNDLE

SYSTEM_DESCRIPTION_PATH = Path(__file__).parent / "system_description.txt"

def render_preference_dimensions(bundle: List[PreferenceDim]) -> str:
    lines = []

    for i, dim in enumerate(bundle, start=1):
        lines.append(f"{i}. {dim.label}")
        lines.append(f"Field name: {dim.field}")
        if getattr(dim, "kind", "categorical") == "color":
            lines.append(
                "Value type: HSV color. Emit an object "
                '{"h": <0-179>, "s": <0-255>, "v": <0-255>, "range": <0.0-1.0>}.'
            )
        else:
            lines.append(f"Allowed options: [{', '.join(dim.options)}]")
        lines.append(dim.description)
        lines.append("")  # blank line between dimensions

    return "\n".join(lines).strip()

def get_system_description_prompt() -> str:
    template = SYSTEM_DESCRIPTION_PATH.read_text(encoding="utf-8")

    preference_dimensions = render_preference_dimensions(PREFERENCE_BUNDLE)

    return template.format(
        preference_dimensions=preference_dimensions
    )
    
if __name__ == "__main__":
    print(get_system_description_prompt())