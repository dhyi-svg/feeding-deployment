#!/usr/bin/env python3
"""Startup self-test for the compute-side peripherals.

Runs four quick checks, in order, and prints a PASS/FAIL summary:

  1. Speaker   -- publish to /speak (voiced on the USB speaker AND the iPad).
  2. Transfer  -- exercise the real button path: arm the robot_executing page and
                  wait for the webapp to relay the press on /webapp_to_robot.
  3. LED       -- drive the Feather LED ON for N seconds, then OFF.
  4. Molmo     -- HTTP-probe the molmo /predict endpoint to confirm it is up.

Intended to be run at startup, BEFORE launch_robot.sh / run.py bring up the full
stack (so the LED serial port and the /speak node are not contended). It needs a
ROS master and rosbridge already running for steps 1-2 (the iPad talks over
rosbridge). Run with the workspace sourced:

    python startup_selftest.py
    python startup_selftest.py --button-timeout 60 --led-seconds 3

Exit code is 0 only if every non-skipped check passes.
"""

import argparse
import json
import re
import sys
import time
from pathlib import Path

import serial

import rospy
from std_msgs.msg import String

# The Feather LED is the same device/protocol used by PerceptionInterface
# (see interfaces/perception_interface.py) and misc/control_feather.py.
LED_SERIAL_PORT = "/dev/serial/by-id/usb-UnexpectedMaker_FeatherS2_Neo_84722E753121-if00"
LED_BAUD_RATE = 115200

# The molmo URL is a static ngrok domain hard-coded in appliance_perception.py.
# We read it from there at runtime so this test tracks the real deployment value
# instead of drifting out of date. This literal is only the last-resort fallback.
APPLIANCE_PERCEPTION_PY = (
    Path(__file__).resolve().parent.parent
    / "perception" / "appliance_perception" / "appliance_perception.py"
)
MOLMO_URL_FALLBACK = "https://exponent-sediment-professed.ngrok-free.dev/predict"

WEBAPP_TO_ROBOT_TOPIC = "/webapp_to_robot"   # iPad -> robot
ROBOT_TO_WEBAPP_TOPIC = "/robot_to_webapp"   # robot -> iPad
SPEAK_TOPIC = "/speak"


def _hr():
    print("-" * 68)


def read_molmo_url():
    """Parse `self.molmo_url = "..."` out of appliance_perception.py.

    Returns the fallback if the file or assignment can't be found, so the probe
    still runs against *something* rather than crashing the whole self-test.
    """
    try:
        text = APPLIANCE_PERCEPTION_PY.read_text()
    except OSError as e:
        print(f"[molmo] could not read {APPLIANCE_PERCEPTION_PY}: {e}")
        return MOLMO_URL_FALLBACK
    m = re.search(r'self\.molmo_url\s*=\s*["\']([^"\']+)["\']', text)
    if not m:
        print("[molmo] molmo_url assignment not found; using fallback")
        return MOLMO_URL_FALLBACK
    return m.group(1)


def test_speaker(text):
    """Publish a phrase to /speak. Returns True if the Speak node is listening.

    We can't hear the audio from here, so 'PASS' means the message was published
    to a live subscriber (the speak.py node). Actual audio still needs a human ear.
    """
    _hr()
    print("[1/4] SPEAKER -- publishing to /speak")
    pub = rospy.Publisher(SPEAK_TOPIC, String, queue_size=1)

    # Wait briefly for the Speak node to connect to our publisher.
    deadline = time.time() + 3.0
    while pub.get_num_connections() == 0 and time.time() < deadline:
        time.sleep(0.05)

    subscribers = pub.get_num_connections()
    if subscribers == 0:
        print(f"  [!] no subscribers on {SPEAK_TOPIC} -- is speak.py running?")
        print("      (publishing anyway; the iPad webapp may still pick it up)")

    pub.publish(String(data=text))
    print(f'  published: "{text}"  (subscribers seen: {subscribers})')
    print("  -> listen: audio should play on the USB speaker and the iPad.")
    # Give gTTS + network + playback time to actually produce sound.
    time.sleep(5.0)
    return subscribers > 0


def test_transfer_button(timeout_s):
    """Verify the physical button end-to-end via the real (webapp) path.

    The iPad button never touches /transfer_button. App.vue detects it and the
    robot_executing page relays it to /webapp_to_robot as
    {state:'button_press', status:'pressed'} -- but only while the robot has armed it.
    Here we play the robot: route the iPad to robot_executing and send button_arm:on on
    /robot_to_webapp (re-sent until the press arrives, since the topic isn't latched),
    then wait for the relayed press on /webapp_to_robot.

    Prereq: the webapp must be open/connected on the iPad. For pure button-hardware
    calibration (audio device id + threshold) use the webapp's /mictest screen instead.
    """
    _hr()
    print("[2/4] TRANSFER BUTTON -- exercising the real webapp path")
    print("  Impersonating the robot: routing iPad to robot_executing, arming the button,")
    print("  and showing the 'Waiting for button press' prompt (as the real transfer does).")
    print(f"  >>> The iPad should show 'Waiting for button press'; press the button (timeout {timeout_s:.0f}s) ...")

    # queue_size must cover the jump+arm+expl burst below: at 1, rospy's outbound
    # queue drops the older messages and only 'explanation' reaches subscribers
    # (the arm then only gets through by luck). WebInterface uses 10 for this topic.
    to_robot = rospy.Publisher(ROBOT_TO_WEBAPP_TOPIC, String, queue_size=10)
    pressed = {"ok": False}

    def on_msg(msg):
        try:
            d = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if d.get("state") == "button_press" and d.get("status") == "pressed":
            pressed["ok"] = True

    sub = rospy.Subscriber(WEBAPP_TO_ROBOT_TOPIC, String, on_msg, queue_size=10)
    jump = String(data=json.dumps({"state": "robot_executing", "status": "jump"}))
    arm = String(data=json.dumps({"state": "button_arm", "status": "on"}))
    # Mirror the real transfer flow, which fix_explanation()s this before blocking so
    # robot_executing shows the prompt instead of a blank page while the user presses.
    expl = String(data=json.dumps({"state": "explanation",
                                   "status": "Waiting for button press (startup self-test)"}))
    try:
        time.sleep(0.5)  # let pub/sub connect over rosbridge
        deadline = time.time() + timeout_s
        while not pressed["ok"] and time.time() < deadline and not rospy.is_shutdown():
            # Re-send jump + arm + explanation (~1.5s cadence) so a late-mounting
            # robot_executing page still gets routed, armed, and shows the prompt.
            to_robot.publish(jump)
            to_robot.publish(arm)
            to_robot.publish(expl)
            next_resend = time.time() + 1.5
            while time.time() < next_resend and not pressed["ok"]:
                time.sleep(0.05)
    finally:
        to_robot.publish(String(data=json.dumps({"state": "button_arm", "status": "off"})))
        # Replace the prompt so the page doesn't sit on a stale "waiting" line.
        done_text = "Button press received (self-test)" if pressed["ok"] else "Self-test: button wait ended"
        to_robot.publish(String(data=json.dumps({"state": "explanation", "status": done_text})))
        sub.unregister()

    if pressed["ok"]:
        print("  [ok] press relayed on /webapp_to_robot (button_press:pressed).")
        return True
    print(f"  [x] no button_press received within {timeout_s:.0f}s.")
    print("      Check: webapp open on the iPad? rosbridge_websocket running?")
    print("      button adapter selected/calibrated on the webapp /mictest screen?")
    return False


def test_led(seconds, brightness):
    """Turn the Feather LED ON for `seconds`, then OFF, over one serial session."""
    _hr()
    print(f"[3/4] LED -- ON for {seconds:.0f}s then OFF  ({LED_SERIAL_PORT})")
    try:
        with serial.Serial(LED_SERIAL_PORT, LED_BAUD_RATE, timeout=1) as ser:
            # The FeatherS2 resets when the port opens; give its firmware a moment
            # to come up before the first command (matches control_feather.py).
            time.sleep(2.0)

            def send(cmd):
                ser.reset_input_buffer()
                ser.reset_output_buffer()
                ser.write(f"{cmd}\r\n".encode())

            send("ON")
            send(f"BRIGHTNESS {brightness}")
            print(f"  LED should be ON now (brightness {brightness}).")
            time.sleep(seconds)
            send("OFF")
            time.sleep(0.2)  # let the final write flush before we close the port
            print("  LED commanded OFF.")
        return True
    except (serial.SerialException, OSError) as e:
        print(f"  [x] LED serial failed on {LED_SERIAL_PORT}: {e}")
        print("      Check: Feather plugged in? port busy (is run.py already up)?")
        return False


def test_molmo(timeout_s):
    """Probe the molmo endpoint. Server-alive is PASS even if it 405s a GET."""
    _hr()
    url = read_molmo_url()
    print(f"[4/4] MOLMO -- probing {url}")
    try:
        import requests
    except ImportError:
        print("  [x] python 'requests' not available; cannot probe.")
        return False

    try:
        # ngrok-skip-browser-warning bypasses the ngrok free interstitial page.
        resp = requests.get(
            url, timeout=timeout_s, headers={"ngrok-skip-browser-warning": "true"}
        )
    except requests.exceptions.RequestException as e:
        print(f"  [x] endpoint unreachable ({type(e).__name__}): {e}")
        print("      The ngrok tunnel is down, or the URL rotated in appliance_perception.py.")
        return False

    code = resp.status_code
    # An ngrok error (ERR_NGROK_3200 etc.) means the tunnel itself is offline.
    # ngrok serves these as HTTP 404, so check the header BEFORE trusting the
    # status code -- otherwise a dead tunnel false-passes as "app answered 404".
    ngrok_error = resp.headers.get("ngrok-error-code")
    if ngrok_error:
        print(f"  [x] HTTP {code} with ngrok-error-code={ngrok_error}: tunnel is OFFLINE.")
        print("      Restart ngrok on the molmo machine, or the URL rotated in appliance_perception.py.")
        return False
    # 502/503/504 from ngrok => tunnel is up but the molmo backend behind it isn't.
    if code in (502, 503, 504):
        print(f"  [x] HTTP {code}: tunnel reachable but molmo backend appears DOWN.")
        return False
    # 405 (GET on a POST-only endpoint), 200, 400, 404 all prove the app answered.
    print(f"  [ok] HTTP {code}: molmo server is UP and answering.")
    if code == 405:
        print("       (405 is expected -- /predict only accepts POST.)")
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--speak-text", default="Startup self test. Speaker is working.",
                        help="phrase to send to /speak")
    parser.add_argument("--button-timeout", type=float, default=120.0,
                        help="seconds to wait for the iPad transfer button")
    parser.add_argument("--led-seconds", type=float, default=3.0,
                        help="how long to hold the LED on")
    parser.add_argument("--led-brightness", type=float, default=0.5,
                        help="LED brightness 0.0-1.0 while on")
    parser.add_argument("--molmo-timeout", type=float, default=8.0,
                        help="HTTP timeout for the molmo probe")
    parser.add_argument("--skip", default="",
                        help="comma-separated steps to skip: speaker,button,led,molmo")
    args = parser.parse_args()

    skip = {s.strip().lower() for s in args.skip.split(",") if s.strip()}

    rospy.init_node("startup_selftest", anonymous=True, disable_signals=True)

    results = {}
    if "speaker" not in skip:
        results["speaker"] = test_speaker(args.speak_text)
    if "button" not in skip:
        results["button"] = test_transfer_button(args.button_timeout)
    if "led" not in skip:
        results["led"] = test_led(args.led_seconds, args.led_brightness)
    if "molmo" not in skip:
        results["molmo"] = test_molmo(args.molmo_timeout)

    _hr()
    print("SUMMARY")
    for name in ("speaker", "button", "led", "molmo"):
        if name in results:
            print(f"  {name:8s} : {'PASS' if results[name] else 'FAIL'}")
        elif name in skip:
            print(f"  {name:8s} : skipped")
    _hr()

    return 0 if all(results.values()) else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\ninterrupted.")
        sys.exit(130)
