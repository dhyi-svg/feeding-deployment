"""Create a named REACH_JOINT_ANGLES action on the arm, from a local preset.

The action shows up under **Actions** in the Kinova web app (alongside e.g. RAMMP_HOME),
so a saved pose can be re-played from the arm's own UI with no ROS, no scripts and no
Python session -- handy for getting back to a viewing pose between runs.

    python scripts/session/create_kinova_action.py microwave_approach_start_pos MICROWAVE_HOME
    python scripts/session/create_kinova_action.py --list

Reads joint values from config/local_arm_presets.yaml (the same file goto_preset.py uses),
so the web-app button and the CLI path can never drift apart.

Opens its own short-lived Kortex session. This is a CONFIGURATION write, not motion --
it stores a pose on the arm and moves nothing. It coexists with arm_server.py and with
the web app being open, unlike the motion path where only one process may hold control.

Refuses to overwrite an existing action of the same name; delete it in the web app first.
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import yaml
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.messages import Base_pb2, Session_pb2
from kortex_api.RouterClient import RouterClient
from kortex_api.SessionManager import SessionManager
from kortex_api.TCPTransport import TCPTransport

ARM_IP, ARM_PORT = "192.168.1.10", 10000
PRESETS = Path(__file__).resolve().parents[2] / "config" / "local_arm_presets.yaml"

parser = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("preset", nargs="?", help="key in config/local_arm_presets.yaml")
parser.add_argument("action_name", nargs="?", help="name to show in the web app")
parser.add_argument("--list", action="store_true", help="list existing actions and exit")
args = parser.parse_args()

if not args.list and (not args.preset or not args.action_name):
    parser.error("give a preset and an action name, or use --list")

joint_degrees = None
if not args.list:
    presets = yaml.safe_load(PRESETS.read_text())
    if args.preset not in presets:
        sys.exit(f"No preset '{args.preset}'. Available: {', '.join(presets)}")
    entry = presets[args.preset]
    values = np.asarray(entry["values"], dtype=float)
    degrees = values if entry.get("units", "degrees") == "degrees" else np.degrees(values)
    # The arm reports and displays joint angles in [0, 360); normalising here keeps the
    # stored action readable in the web app and consistent with GetMeasuredJointAngles.
    joint_degrees = np.mod(degrees, 360.0)
    print(f"preset '{args.preset}' -> {[round(float(v), 3) for v in joint_degrees]} deg")

transport = TCPTransport()
router = RouterClient(transport, lambda ex: print("router:", ex))
transport.connect(ARM_IP, ARM_PORT)

info = Session_pb2.CreateSessionInfo()
info.username, info.password = "admin", "admin"
info.session_inactivity_timeout = 60000
info.connection_inactivity_timeout = 2000
session = SessionManager(router)
session.CreateSession(info)
base = BaseClient(router)

try:
    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
    existing = base.ReadAllActions(action_type)
    names = [a.name for a in existing.action_list]

    if args.list:
        print("Existing REACH_JOINT_ANGLES actions on the arm:")
        for n in names:
            print(f"  {n}")
        sys.exit(0)

    if args.action_name in names:
        sys.exit(f"An action named '{args.action_name}' already exists. "
                 f"Delete it in the web app first, then re-run.")

    action = Base_pb2.Action()
    action.name = args.action_name
    action.application_data = ""
    action.handle.action_type = Base_pb2.REACH_JOINT_ANGLES
    for i, value in enumerate(joint_degrees):
        angle = action.reach_joint_angles.joint_angles.joint_angles.add()
        angle.joint_identifier = i
        angle.value = float(value)

    base.CreateAction(action)
    print(f"created action '{args.action_name}'")

    after = [a.name for a in base.ReadAllActions(action_type).action_list]
    if args.action_name in after:
        print("verified: it is now listed on the arm. Refresh the web app's Actions page.")
    else:
        print("WARNING: created without error but not listed on read-back.")
finally:
    session.CloseSession()
    transport.disconnect()
