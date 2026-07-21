# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## ⏱ CURRENT STATE / NEXT STEP (fork, `microwave-task` branch — updated 2026-07-21)

**Goal this week:** autonomous **microwave door opening** on the real arm. Two boxes now: the original single Jetson Orin Nano + Kinova Gen3 rig (no NUC/base/ROS), and a new **Pachirisu (RTX/24.04) + RoboStack** track that gets real `rospy` and the PRPL/py3.10+ deps into one interpreter — impossible on the Jetson. See `LOCAL_DEPLOYMENT.md` for both env/what-works references and `TESTING_LOG.md` for the blow-by-blow hardware log.

**Where we are (Jetson):** the **full autonomous cycle works** — live GroundingDINO detect → handle in arm-base frame (easy_handeye2 calib + live FK, no tf2) → depth/lateral-corrected grasp → open the door ~90° via a joint-space arc → release → retract. It has completed end-to-end.
- **Detect+approach+grip** is one script; corrections baked in: `DEPTH_CORR=0.16` (perception overestimates depth ~16 cm), `LAT_CORR=0.07` (lateral latch bias), `GRIP_EXT=0.065` (grasp EE = handle − 6.5 cm along approach; the 6.5 vs earlier 9 cm is the proven "grip a bit more forward / firmer" fix). Lands within ~1–2 cm, grip holds through the swing.
- **Seeded-IK fix** (prevents the wrist-flip on big moves): before every `p.calculateInverseKinematics`, seed the sim from the arm's *current real joints* (`resetJointState` to `get_state()["position"]`) so IK returns a nearby config. **Caveat:** seeding is too conservative *mid-arc* — it stalled on arc step 2 (sim ik_err 3.5 cm). Use seeding for large jumps (approach, back-off); for the small arc waypoints fall back to unseeded (home-seeded) IK, which reaches the arc exactly.

**Physical state right now:** the arm may be **holding the microwave door grasped** (gripper ~0.99) mid-arc, paused. To reclaim control / reset: restart `arm_server.py` then re-run `bulldog_bypass.py` (Xbox teleop or an e-stop faults the Kortex session; a fresh server re-locks motion so the bypass must be re-run).

**Where we are (Pachirisu):** brand-new track this session, **read-only so far, no motion attempted yet.** RoboStack `ros_env` (conda-forge ROS Noetic on Python 3.11, in the `feed-noetic` container) verified end-to-end against the real arm: network reachable, `kortex_api` installed (pure-Python wheel, pins `protobuf==3.20.0` — benign-but-latent conflict with `google-generativeai`/`anthropic`, flagged not fixed), `ARM_RPC_HOST` override now **committed** (`2c0e3498`), `arm_server.py` runs unbuffered and binds `127.0.0.1:5000`, and a direct `get_state()` read came back sane (7 finite joints, unit EE quaternion, gripper at the documented "open" reading, no fault). Clean start/stop verified twice. Also fixed and verified this session: the `rviz_interface is None` guard in `open_microwave()` (`open_door.py:202`, commit `a16222fb`) — the real `execute_action()` behavior-tree dispatch now runs the whole HLA to completion in replay mode (`NullSimulator`, synthetic `handle_opening_pos.pkl`) with no exception.

**Immediate next step:** on Pachirisu, move past read-only telemetry to the same rung-3/4 motion ladder the Jetson already passed (small joint move, small cartesian move) — still real-hardware-cautious, low speed, hand on e-stop. In parallel, since RoboStack finally gives rospy + PRPL together, evaluate whether the real `bulldog`/watchdog can replace `bulldog_bypass.py` on this box. On the Jetson side: finish the arc open with a solver that tries seeded IK first and **falls back to unseeded when ik_err > 2 cm** (guarding against a big joint-jump = flip), then fold the whole detect→grasp→open flow into the `open_microwave` HLA (rviz guard is now fixed; still needs perception wired in without `PerceptionInterface`'s heavier ROS/camera/model deps — `netft_rdt_driver`/`torch`/`groundingdino`/`supervision` are missing on both boxes).

## What this repo is

The EmPRISE lab's robot-assisted feeding stack. It is both a **catkin (ROS 1 Noetic) package** (`package.xml`, `CMakeLists.txt`, `launch/*.launch`) and a **pip-installable Python package** (`pyproject.toml`, code under `src/feeding_deployment/`). A companion Vue 3 frontend lives in `webapp/`.

This working copy is a personal fork (`origin` = `dhyi-svg`) of `empriselab/feeding-deployment` (`upstream`). **Never push to `upstream`** — it is fetch-only. Work happens on feature branches and pushes go to `origin`.

### Current deployment context

Unlike the lab's reference setup (compute box + Intel NUC on the mobile robot + Cornell cluster running a Molmo VLM, all networked together), this working copy targets a single-machine deployment: one Kinova Gen3 7-DOF arm plugged directly into the local machine, no NUC in the loop. A wheelchair base exists on this platform but is **currently out of scope / parked and unpowered** — treat all in-progress work as arm-only, but don't hardcode assumptions that the base will never exist (avoid baking arm-only shortcuts into shared interfaces that other HLAs/executive code also use). `bulldog` (see Safety below) refuses to start unless *both* the arm and base RPC servers are up; arm-only testing needs a stub base server or a way to bypass bulldog.

## Commands

Install (from repo root):
```bash
pip install -e ".[robot, develop]"   # full install (robot control + dev tooling)
pip install -e .                     # preference-learning-only subset
```

Full CI check (autoformat + mypy + pylint-via-pytest), should finish in 5-10s:
```bash
./run_ci_checks.sh
```

Individual steps (what `run_ci_checks.sh` runs):
```bash
./run_autoformat.sh                                          # black + docformatter + isort, in place
mypy .
pytest . --pylint -m pylint --pylint-rcfile=.pylintrc         # pylint-as-a-test-marker over the whole tree
```

Run the test suite directly (no pylint marker):
```bash
PYTHONPATH=src python -m pytest tests/ -v
PYTHONPATH=src python -m pytest tests/test_checkpoint.py -v   # single file
```
Some tests (e.g. `tests/test_checkpoint.py`) have no heavy deps and can also be run as a plain script: `PYTHONPATH=src python tests/test_checkpoint.py`.

mypy excludes `src/feeding_deployment/robot_controller/*`, `src/feeding_deployment/head_perception/*`, and `integration/perception_interface.py` (hardware-coupled, not meaningfully type-checkable); `.pylintrc` ignores the same paths.

Webapp (Vue frontend, `webapp/`):
```bash
npm run serve   # dev server
npm run build
```

Full demo run instructions (real robot, sim, tmux session layout, navigation/mapping workflow, teleop) are in `README.md` — that file is deployment-procedure documentation (aliases run on specific lab machines), not something to re-derive here.

## Architecture

### High-level actions (HLA) and behavior trees — `src/feeding_deployment/actions/`

`base.py` defines `HighLevelAction`, an abstract base for every robot skill (e.g. `pick_utensil`, `open_door`, `transfer_tool`). Each HLA:
- exposes a PDDL-style `LiftedOperator` (preconditions/effects over `Predicate`s like `Holding`, `DoorOpen`, `PlateAt`) used for task planning,
- maps to a **behavior tree YAML file** under `actions/behavior_trees/` that is loaded via a custom YAML loader (`!hla <method>` and `!scene_description <attr>` tags resolve to bound methods/attributes at load time), parsed into a tree of `SequenceBehaviorTreeNode`/`ParameterizedActionBehaviorTreeNode`, and ticked to execute.
- Behavior tree parameters (speed, duration, etc.) are individually marked `is_user_editable`; the web app / an LLM (`interpret_user_update_request`) can request bounded live edits to a running user's behavior tree (add a `Pause`/`Retract`/`WaitForGesture` node, or change a parameter value), which get validated and persisted back to the YAML.
- `actions/flair/` is a separate LLM-driven "food manipulation skill library" (identifies/plans around food items on a plate) — distinct from the HLA/behavior-tree system above, invoked as a component from within relevant HLAs.

### Executive — `src/feeding_deployment/integration/run.py`

The main entry point. It builds a `PDDLDomain`/`PDDLProblem` from the HLA operators + predicates, runs a planner (`tomsutils.pddl_planning`) to get a plan, converts it to a sequence of `GroundHighLevelAction`s, and executes them one at a time against whichever of sim / real robot / web interface are wired in. It owns the preference-learning session lifecycle (`preference_session.py`), checkpoint/resume (`checkpoint.py`, states like `after_utensil_pickup`), and mid-skill teleop takeover handling (`TeleopTakeoverException` raised out of `actions/base.py`, caught here to redo-or-continue the plan).

`integration/test_navigate_action.py` is the reference pattern for exercising a single HLA standalone outside the full executive loop.

### Control — `src/feeding_deployment/control/`

Split into `robot_controller/` (arm), `base_controller/` (mobile base), `wrist_controller/` (utensil wrist). Each follows a client/server RPC split: a `*_server.py` runs next to the hardware (in the lab's case, on the NUC) and a `*_client.py`/`*_interface.py` is what the executive talks to. `robot_controller/kinova.py` zeroes arm torque offsets and must be run once before inside-mouth transfer; `robot_controller/preset_actions/` (`retract.py`, `transfer.py`, `acquisition.py`) sends the arm to saved joint configs — these are mounting/environment-specific and must be re-recorded per physical setup, never reused blindly across rigs.

### Safety — `src/feeding_deployment/safety/`

`watchdog.py` and `bulldog.py` are the e-stop/liveness layer. `bulldog` requires **both** the arm RPC server and the base RPC server to be reachable before it will start (it stops both on a single e-stop event since, in the lab wiring, the base Arduino shares the NUC's e-stop line with the arm). `estop_udp_bridge.py`, `estop_sender.py`, `estops_publisher.py`, `collision_sensor.py`/`collision_threshold.py`, `net_diag_*` round out fault detection and cross-machine health signaling.

### Interfaces — `src/feeding_deployment/interfaces/`

`web_interface.py` (webapp comms, including `WebInterfaceTakeoverInterrupt` for user-initiated takeover), `perception_interface.py` (camera/detection access — excluded from mypy/pylint), `rviz_interface.py` (visualization).

### Simulation — `src/feeding_deployment/simulation/`

PyBullet-based (`FeedingDeploymentPyBulletSimulator`, `FeedingDeploymentWorldState`). Every HLA method that moves the robot has a `self.robot_interface is None` branch that falls back to sim visualization instead of a real robot command — this is the mechanism by which the same HLA code runs in both sim and real deployment.

**Known wall (this box):** IKFast build hangs on this box (custom Gen3 URDF, aarch64/8GB) — repo's `plan_to_ee_pose` path unavailable locally; sim validation used PyBullet native IK instead (`p.calculateInverseKinematics` + `resetJointState`, e.g. `scripts/sim_gen3_open_microwave.py`).

### Preference learning — `src/feeding_deployment/preference_learning/`

An LLM-driven (Anthropic/OpenAI) system that models per-user food/manipulation preferences across a meal and across days, combining working memory, episodic memory (`methods/episodic_memory.py`), and a long-term summary model (`methods/long_term_memory.py`). Dataset generation and offline evaluation tooling lives in `data_generation/` and `methods/evaluate_*`; see `preference_learning/README.md` for those CLI invocations. **Known bug:** meal finalization can crash in `long_term_memory.py` with an Anthropic `400 invalid_request_error` ("assistant message prefill... conversation must end with a user message") — a pre-existing upstream issue, not something to silently work around by disabling the feature.

### Webapp — `webapp/`

Vue 3 app (`vue-cli-service`) that talks to ROS over `roslib` (rosbridge), used as the iPad/laptop UI during a feeding session (task selection, detection confirmation, teleop/takeover screen).
