"""The pure modules here are copies of feeding_deployment/button_press/ with only the import path
renamed. Fails when one side changes without the other. Skipped outside the feeding-deployment
checkout (e.g. once this package lives in its own repo)."""
from pathlib import Path

import pytest

PKG = Path(__file__).resolve().parents[1] / "rammp_button_press"
ORIG = Path(__file__).resolve().parents[2] / "src" / "feeding_deployment" / "button_press"


@pytest.mark.parametrize("name", ["dome_pattern.py", "panel_frame.py", "geometry.py"])
def test_copy_matches_original(name):
    if not (ORIG / name).is_file():
        pytest.skip("not inside the feeding-deployment checkout")
    want = (ORIG / name).read_text().replace("feeding_deployment.button_press", "rammp_button_press")
    assert (PKG / name).read_text() == want, f"{name} drifted from {ORIG / name}"
