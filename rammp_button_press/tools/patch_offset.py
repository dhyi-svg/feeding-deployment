# Scratch only: pretend PREPRESS_EE_OFFSET_M was re-measured (fingertip 2 cm out), for the mock dry run.
import os, numpy as np
import rammp_button_press.press_button as pb
if os.environ.get("SCRATCH_OFFSET"):
    pb.PREPRESS_EE_OFFSET_M = np.array([float(v) for v in os.environ["SCRATCH_OFFSET"].split(",")])
