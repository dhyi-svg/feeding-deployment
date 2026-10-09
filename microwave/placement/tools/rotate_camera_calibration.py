"""Write the hand-eye calibration with the camera's new roll (it was remounted rotated 90 deg).

    python3 microwave/placement/tools/rotate_camera_calibration.py --roll-deg 90          # writes ..._roll+90.calib
    python3 microwave/placement/tools/rotate_camera_calibration.py --roll-deg -90
    python3 microwave/placement/tools/rotate_camera_calibration.py --roll-deg 90 --pivot 0.0 0.0 -0.02

Which sign is right is decided from data by check_camera_roll.py (run it first with the OLD
calibration published). --pivot: the point (old camera frame, m) the camera actually turned
about, if not its optical centre -- e.g. the mount screw; leaving it 0 puts up to ~2 cm of
translation error in the chain (re-calibrate with easy_handeye2 to remove it for good).
Then start calibration_tf with --calib <the new file> (bringup.sh: CALIB=<file>).
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from camera_roll import rotate_calibration, write_calibration  # noqa: E402

DEFAULT = Path.home() / ".ros2/easy_handeye2/calibrations/wrist_camera_calib.calib"

a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
a.add_argument("--roll-deg", type=float, required=True)
a.add_argument("--pivot", type=float, nargs=3, default=(0.0, 0.0, 0.0))
a.add_argument("--calib", default=str(DEFAULT))
a.add_argument("--out", help="default: <calib>_roll<+deg>.calib next to it")
args = a.parse_args()
src = Path(args.calib).expanduser()
data = yaml.safe_load(src.read_text())
new = rotate_calibration(data, args.roll_deg, args.pivot)
out = Path(args.out) if args.out else src.with_name(f"{src.stem}_roll{args.roll_deg:+.0f}{src.suffix}")
if out.resolve() == src.resolve():
    sys.exit("refusing to overwrite the source calibration")
write_calibration(out, new, f"{src.name} rolled {args.roll_deg:+.1f} deg about the optical axis, pivot "
                            f"{list(args.pivot)} (rotate_camera_calibration.py)")
print(f"wrote {out}\n  translation {np.round([new['transform']['translation'][k] for k in 'xyz'], 4).tolist()}"
      f"\n  rotation    {np.round([new['transform']['rotation'][k] for k in 'xyzw'], 4).tolist()}")
