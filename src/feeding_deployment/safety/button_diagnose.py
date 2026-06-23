"""Diagnostic: print live audio amplitude for the e-stop button device.

Run this, then press the button a few times. Watch the 'peak' column:
it shows the largest absolute sample in each ~0.1s chunk. We use this to
pick a detection threshold for button.py (default threshold is 10000).
"""

import argparse
import time

import numpy as np
import pyaudio

p = argparse.ArgumentParser()
p.add_argument("--id", type=int, required=True, help="input device index")
p.add_argument("--seconds", type=float, default=30.0)
args = p.parse_args()

audio = pyaudio.PyAudio()
info = audio.get_device_info_by_index(args.id)
print(f"Listening on device {args.id}: {info['name']}")
print("Press the e-stop button a few times. Watching peak amplitude...\n")

stream = audio.open(
    format=pyaudio.paInt16,
    channels=1,
    rate=48000,
    input=True,
    frames_per_buffer=4800,
    input_device_index=args.id,
)

overall_peak = 0
start = time.time()
try:
    while time.time() - start < args.seconds:
        data = np.frombuffer(stream.read(4800, exception_on_overflow=False), dtype=np.int16)
        peak = int(np.max(np.abs(data.astype(np.int32))))
        overall_peak = max(overall_peak, peak)
        bar = "#" * min(50, peak // 200)
        flag = "  <-- would TRIGGER (>10000)" if peak > 10000 else ""
        print(f"peak={peak:6d}  {bar}{flag}")
except KeyboardInterrupt:
    pass
finally:
    stream.stop_stream()
    stream.close()
    audio.terminate()
    print(f"\nLargest peak seen: {overall_peak}")
    print("If your button presses never exceed 10000, that's why button.py stayed silent.")
