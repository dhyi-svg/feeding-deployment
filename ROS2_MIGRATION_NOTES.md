# ROS 1 -> ROS 2 (Jazzy) migration notes

This document tracks the migration of `feeding-deployment` from ROS 1 Noetic (catkin) to
ROS 2 Jazzy (ament). It was written incrementally as the migration proceeded, in the
worktree `.claude/worktrees/agent-aab4331f8f93620f7` (branch
`worktree-agent-aab4331f8f93620f7`), and is meant to be read by whoever reviews/merges
this branch, not just as a historical log.

**Nothing in this migration was validated against real hardware, live ROS topics, or the
RPC servers (`arm_server.py`, `stub_base_server.py`, etc.).** The box this migration was
done on (`Pachirisu`) has a real Kinova Gen3 arm connected and, per the project's own
status notes, possibly mid-task (holding a microwave door) in a separate, non-worktree
checkout at the time of this work. All validation below is static: `ast.parse`, a real
`colcon build` of the message package, live-but-inert `rclpy`/`launch` object
construction (creating nodes/launch descriptions without ever calling `rclpy.spin` or
starting a process), and reading/reasoning about the code. Treat everything in this
migration as "compiles and looks right" at best, not "verified working."

## 1. The core design decision: node ownership (rospy's implicit global node -> rclpy)

rospy has an implicit global node: some script calls `rospy.init_node(name)` once, and
then *any* file, class, or helper function in the process can freely call bare
`rospy.Publisher(...)`, `rospy.get_param(...)`, `rospy.loginfo(...)`, etc., with no node
handle threaded through. rclpy has no implicit global node -- every publisher,
subscriber, timer, parameter, logger, and clock is a method on an explicit `Node`
instance, and `rclpy.init()` must run before any `Node` is constructed.

**Chosen strategy: a lazily-created, per-process singleton `rclpy.Node`.** Implemented in
`src/feeding_deployment/ros2_utils/node_handle.py`:
  - `init_node(name)` -- call once near a script's real entry point (mirrors
    `rospy.init_node(name)`); idempotent (first caller's name wins, matching rospy's own
    "calling it twice just warns" behavior).
  - `get_node()` -- the rclpy analogue of "just reach for `rospy` from anywhere"; lazily
    creates a default-named node if `init_node` was never called first.
  - `is_shutdown()` / `shutdown()`.

`src/feeding_deployment/ros2_utils/rospy_compat.py` layers rospy-shaped convenience
wrappers on top: `loginfo`/`logwarn`/`logerr`/`logdebug`/`logfatal` (+`_throttle`
variants), `sleep`, `now`, `Rate`, `spin`, `wait_for_message` (hand-rolled -- rclpy has no
built-in blocking single-message wait), `wait_for_service`, and `Time`/`Duration`
aliases, plus best-effort `ROSException`/`ROSInterruptException` stand-ins (rclpy has no
single unified "something about ROS communication went wrong" exception the way rospy
does; see that class's docstring for the caveat). **Deliberately NOT wrapped:**
Publisher/Subscriber/Service/Client construction -- these need a real per-call-site
decision (QoS depth, `latch=True` -> `DurabilityPolicy.TRANSIENT_LOCAL`, service handler
signature changes) that a blind wrapper would hide. Every migrated file constructs these
directly via `node_handle.get_node().create_publisher(...)` etc.

### Why a singleton instead of dependency-injecting an explicit `Node`

The more "idiomatic modern rclpy" approach is threading a `Node` (or `self.node`)
through the constructor of every class that touches ROS. That was rejected here because:

1. **Blast radius.** 52 files import `rospy` directly, but many more files construct
   *those* files' classes (HLAs, the executive, test harnesses, preset actions, ...).
   Explicit DI would mean touching call sites far outside the 52-file rospy-import list,
   turning a migration into a much larger refactor of the whole call graph.
2. **Reviewability.** The singleton keeps each of the 52 files' diffs local and
   mechanical (`s/rospy.X/node_handle.get_node().Y/` plus the handful of documented
   signature changes below). A human reviewing this branch file-by-file can verify each
   diff in isolation; a DI refactor would require reviewing the whole call graph at once
   to be confident nothing broke.
3. **Matches the existing style.** This codebase's ROS1 code already leans on rospy's
   implicit-global-node pattern pervasively (see e.g. how liberally `rospy.loginfo` is
   called from deep inside helper functions with no node in scope). The singleton is the
   closest behavioral analogue, minimizing surprise.

**Tradeoffs, explicitly on the record (also documented in the module docstring itself):**
  - Does **not** support multiple distinct nodes coexisting in one process. Nothing in
    this codebase currently needs that -- every entry point (`arm_server.py`, `run.py`,
    each safety daemon) is already its own OS process -- but it's a real constraint if
    that ever changes.
  - Not what the ROS 2 docs recommend for new code. If/when a piece of this codebase gets
    *rewritten* rather than *migrated* (e.g. turned into a proper composable-node
    component), switching that piece to an explicitly-constructed, explicitly-spun `Node`
    in its own `main()` is the natural next step -- `get_node()` calls are easy to grep
    for and replace with `self.node`.
  - Logging (or anything else) called before any `init_node`/`get_node()` call anywhere
    in the process will silently create a node named `"feeding_deployment_node"` rather
    than erroring the way an un-init'd `rospy` call would. Deliberate leniency to avoid
    ordering-sensitive crashes during migration; grep for `init_node(` calls if a node's
    name looks wrong in logs later.
  - `run.py` (the executive) is the natural place for the "real" `init_node(name)` call
    for the main deployment process, since it's the actual entry point per CLAUDE.md's
    architecture description; the integration/ migration batch was instructed to convert
    whatever `rospy.init_node` call already existed there rather than invent a new one.
    See section 4's per-batch report for what was actually found.

## 2. `feeding_deployment_msgs` (custom srv package) -> ROS 2 `ament_cmake` + `rosidl`

`package.xml`: format 2 catkin -> format 3, `buildtool_depend` catkin -> `ament_cmake`,
`message_generation`/`message_runtime` -> `rosidl_default_generators` (build) /
`rosidl_default_runtime` (exec) + `<member_of_group>rosidl_interface_packages</member_of_group>`.

`CMakeLists.txt`: `catkin_package()`/`add_service_files()`/`generate_messages()` ->
`rosidl_generate_interfaces(${PROJECT_NAME} "srv/SetCollisionThreshold.srv" DEPENDENCIES std_msgs)`.

`srv/SetCollisionThreshold.srv` itself is **byte-for-byte unchanged** -- the `.srv` text
format (fields, `---`, response fields) is identical between ROS1 and ROS2's rosidl.

**Verified for real**, not just by inspection: symlinked the package into a scratch
colcon workspace, ran `colcon build --symlink-install` against `/opt/ros/jazzy` --
succeeded cleanly. Then imported the generated
`feeding_deployment_msgs.srv.SetCollisionThreshold` module in Python and instantiated
both `Request` and `Response` -- works, fields present and correctly typed/defaulted
(`threshold=0.0`, `success=False`, `previous_threshold=0.0`).

## 3. Main package: `ament_python`, and a real environment gotcha found (not silently worked around)

### `ament_python` vs `ament_cmake`

Chose **`ament_python`**. Reasoning: there is no first-party C++ source anywhere in this
repo (the ROS1 `package.xml` declared `roscpp` as a dependency, but nothing under `src/`
is C++ -- it was almost certainly a boilerplate leftover from `catkin_create_pkg`'s
default template, never exercised). The repo is a `pyproject.toml`-based Python
package already (`pip install -e ".[robot,develop]"` is the documented install path in
CLAUDE.md). `ament_python` is the ROS2 build type designed exactly for "pure Python
package that also wants to be a ROS2 package" and needs the least new boilerplate.

### package.xml (format 3, `ament_python`)

Dependency mapping (see inline XML comments in `package.xml` for the full reasoning):
  - `roscpp`, `rospy` -> `rclpy` (dropped `roscpp` entirely -- see above).
  - `rviz` -> `rviz2`. **Confirmed available** via `apt-cache search` on this box:
    `ros-jazzy-rviz2`.
  - `rplidar_ros` -> `rplidar_ros`. **Confirmed available AND same package name** on
    Jazzy: `ros-jazzy-rplidar-ros`.
  - Added (grep-confirmed the code actually uses them, even though the ROS1 manifest
    never declared them): `message_filters`, `cv_bridge`, `launch`, `launch_ros`,
    `tf2_geometry_msgs`, `rosbridge_suite` (**confirmed available**:
    `ros-jazzy-rosbridge-suite`; same rosbridge JSON-over-websocket wire protocol as
    ROS1 per the task brief -- only the launch package/executable name changes, and
    that's already reflected in the converted launch files), `feeding_deployment_msgs`.

  This box turned out to have real `apt-cache search` / network access, so most of the
  "package name in ROS2" questions the task anticipated as possibly-unverifiable were
  actually confirmed rather than flagged as guesses. The one still-open case is
  `zed_wrapper` (ZED camera ROS2 driver) -- see section 6.

### setup.py: a thin `ament_python` shim, not a pyproject.toml replacement

`ament_python` packages are conventionally built via `setup.py`. This repo's actual
Python packaging (name/version/dependencies/`[project.optional-dependencies]`) stays in
`pyproject.toml` -- `pip install -e ".[robot,develop]"` must keep working exactly as
before. `setup.py` was added with **only** a `data_files` argument (which PEP 621 /
`pyproject.toml` has no way to express): the ament resource-index marker
(`resource/feeding_deployment`, also added), `package.xml` installed to `share/`, and
`launch/`, `config/`, `rviz/`, `urdf/` mirrored into
`share/feeding_deployment/{launch,config,rviz,urdf}/` (replacing the old catkin
`CMakeLists.txt`'s `install(DIRECTORY launch config rviz urdf ...)`). setuptools merges a
`setup.py` and `pyproject.toml` fine as long as they don't declare the same field twice;
here they don't overlap. The root `CMakeLists.txt` was **deleted** -- `ament_python`
packages don't use CMake at all, and keeping a stray one around risks confusing colcon's
package-type identification (see the environment gotcha below, which is adjacent to but
not caused by this).

### Real environment gotcha found and NOT worked around: colcon-python-setup-py vs modern setuptools

Attempted a real `colcon build --packages-select feeding_deployment` against
`/opt/ros/jazzy` (beyond the task's minimum bar of "at least the message package," which
already passed -- see section 2). It fails, reproducibly, with:

```
File ".../colcon_python_setup_py/package_identification/python_setup_py.py", line 301, in _get_setup_information
    return ast.literal_eval(output)
SyntaxError: invalid syntax (<unknown>, line 1)
```

Root-caused by direct reproduction (not guessed): `colcon-python-setup-py` 0.2.9
extracts a package's metadata by running `distutils.core.run_setup(...)` in a subprocess
and `ast.literal_eval()`-ing a `repr()` of the resulting metadata dict. On this box's
`setuptools` 68.1.2, `dist.metadata.python_requires` is stored as a
`packaging.specifiers.SpecifierSet` **object**, not a plain string -- its `repr()` is
`<SpecifierSet('>=3.10')>`, which is not valid Python literal syntax (angle brackets), so
`ast.literal_eval` throws. This is a real, confirmed (reproduced by hand, isolated to the
exact field) incompatibility between `colcon-python-setup-py`'s metadata-extraction
approach and any modern-enough `setuptools`, for **any** package (ROS2 or not) that sets
`requires-python`/`python_requires` -- not specific to anything unusual about this repo's
`pyproject.toml`.

**Not worked around**, deliberately:
  - Removing `pyproject.toml`'s `requires-python = ">=3.10"` would silently weaken a real
    constraint (several `robot` extras genuinely need py3.10+) just to dodge a tooling
    bug -- exactly the kind of "silent guess to make something pass" the task explicitly
    said to avoid.
  - Upgrading/patching `colcon-python-setup-py` or downgrading `setuptools` would mean
    modifying system-wide, apt-managed Python packages on a shared lab machine
    (`Pachirisu`) outside this worktree's scope -- both a discipline violation (stay
    inside the worktree) and a risk to the other, unrelated live session on this box.
  - This only affects the **main** `feeding_deployment` package's `colcon build`.
    `feeding_deployment_msgs` (section 2) is `ament_cmake`, not `ament_python`, so it
    never goes through this code path -- its `colcon build` genuinely passed.

**Practical effect:** `feeding_deployment` cannot currently be `colcon build`-installed
into a real ROS2 workspace on *this* box without either fixing this tooling
incompatibility (upgrade `colcon-python-setup-py` -- check for a fixed release upstream
-- or pin an older `setuptools` in a venv used only for the ROS2 build) or removing
`requires-python` from `pyproject.toml` (not recommended, see above). **A human needs to
resolve this** before `ros2 launch feeding_deployment ...` will work anywhere. As a
workaround *for validating the launch files only* (not a fix), a fake
`ament_index`/`share/feeding_deployment` symlink tree was used locally to prove the
launch files themselves are structurally sound -- see section 5.

## 4. The 53 rospy-importing Python files

Migrated in 7 parallel batches (by directory), each briefed with the same node-ownership
strategy and conversion cheatsheet (Publisher/Subscriber/Service/Client argument-order and
signature changes, tf2's now-required `node` argument, `message_filters.Subscriber`'s new
required `node` argument, `cv_bridge` being essentially unchanged, actionlib -> 
`rclpy.action` being a real async-architecture change flagged per call site rather than
silently ported, and `rospy.get_param` -> `declare_parameter(...).value` vs
`get_parameter(...).value` depending on once-vs-repeated read semantics). Every batch was
instructed never to execute/import-run any file (syntax-only validation via
`ast.parse`), and the safety/ batch was given extra emphasis on faithful, no-guess
porting given its role as the e-stop/liveness layer.

**Final tally: all 53 files migrated, all committed, zero remaining `import rospy`
anywhere under `src/`** (verified with a repo-wide grep after all 7 batches landed). An
independent final `ast.parse` sweep over every Python file changed across the whole
migration (71 files total, including `ros2_utils/` and the 53 rospy files) found zero
syntax errors.

One additional file outside the 52-file scope was noticed: `tests/test_preference_integration.py`
mentions `rospy` only in a comment (listing heavy optional deps the test intentionally
skips outside the robot extras) -- not real usage, nothing to migrate, confirmed by
reading it.

### 4a. actions/ batch
6 files: `feel_the_bite/base.py`, `feel_the_bite/inside_mouth_transfer.py`,
`feel_the_bite/outside_mouth_transfer.py`, `flair/food_manipulation_skill_library.py`,
`navigate.py` (1880 -> ~1930 lines), `transfer_tool.py`.

`navigate.py` is the significant file here. It uses `actionlib.SimpleActionClient`
throughout a ~250-line watchdog/recovery loop (`_await_nav_result`, `_drive_to_pose`,
`_await_goal_active`) built entirely around actionlib's *blocking* `send_goal()`/
`wait_for_result()`/`get_state()`/`cancel_goal()` API. rclpy's `rclpy.action.ActionClient`
is natively async/goal-handle-based -- there's no mechanical 1:1 translation. Rather than
either (a) blindly renaming methods that don't exist on the new type, producing code that
fails at import/attribute-access time, or (b) silently rewriting the whole control-flow
to be async (a real behavior change to safety-relevant navigation code, guessed at
without review), the batch wrote a new `_BlockingActionClient` shim class: a
synchronous-shaped wrapper around `rclpy.action.ActionClient` that spins the shared node
under the hood to fake the old blocking calls, preserving the existing loop's structure.
This is explicitly flagged as a best-effort structural port, not a verified-equivalent
one -- see its class docstring (TODO block) for the itemized semantic gaps: goal
rejection now raises where actionlib never did; `get_goal_status_text()` has no ROS2
equivalent and always returns `""` (making one existing fallback branch,
`"human teleoperation" in get_goal_status_text()`, permanently dead code -- the primary
signal `_done_pressed_this_leg` is unaffected, but this is a silent capability loss worth
a reviewer's attention); `GoalStatus.ACTIVE`/`SUCCEEDED` -> `STATUS_EXECUTING`/
`STATUS_SUCCEEDED` mappings are unverified against real ROS2 status codes (may also need
`STATUS_ACCEPTED`).

**The single biggest architectural gap found in this entire migration** is also in this
file: `rospy.get_param("move_base/TebLocalPlannerROS/yaw_goal_tolerance")` relied on
ROS1's global parameter server letting any node read *any other node's* parameters.
ROS2 has no global parameter server at all -- parameters are strictly per-node, reachable
only via that node's own parameter *service* (a real RPC call to a specific node, not a
free read). This has no mechanical translation. Implemented as a local
`declare_parameter`/`get_parameter` with a fail-loud `RuntimeError` if unset, clearly
flagged as needing a real fix (a parameter-service client against the actual future Nav2
node) once the ROS2 nav stack (see section 5's `navigation.launch.py` TODO) is finalized
-- this is a downstream consequence of the still-unfinished Nav2 migration, not something
resolvable in isolation here.

Two dead-code cleanups made in passing (unused rospy imports, not functional changes):
`inside_mouth_transfer.py`/`outside_mouth_transfer.py`'s `import rospy` was never
referenced; `food_manipulation_skill_library.py`'s entire `try/except` rospy+tf2_ros+
message-type import block was unused anywhere in the file (removed with an explanatory
comment rather than migrated, since migrating dead code just moves the deadness around).

All 6 files verified via `ast.parse` (both by the migrating agent and independently
re-verified before commit). Committed as `9770a9ed`.

### 4b. control/ batch
15 files (the task briefing undercounted this as "14" but listed 15 paths; all 15 were
migrated): `base_controller/{cmd_vel_bridge_basicmicro,shared_autonomy_manager,
shared_autonomy_teleop,wheel_odom_publisher}.py`, `robot_controller/{arm_client,
joint_states_publisher,kinova}.py`, `robot_controller/preset_actions/{acquisition,
close_gripper,home,open_gripper,retract,transfer}.py`,
`wrist_controller/{horizontal_spoon,wrist_controller}.py`.

  - **`kinova.py` (1425 lines) was deliberately left completely untouched** -- confirmed
    by grep that every `rospy` reference in the file is already inside a comment (a
    commented-out e-stop subscriber/callback); there is no active rospy usage anywhere in
    it. Given this file zeroes arm torque offsets and is used before inside-mouth
    transfer (per CLAUDE.md), "there is nothing to convert" is the correct and safest
    finding here, not an oversight.
  - **`shared_autonomy_manager.py`** gets the same treatment as `navigate.py`'s action
    client (section 4a): `SimpleActionServer`/`SimpleActionClient` calls (`send_goal`,
    `cancel_goal`, `get_state`, `get_result`, `is_preempt_requested`, `set_succeeded`,
    `set_aborted`, `set_preempted`) are left **structurally in place** (same method
    names, same call sites) with a TODO at each, rather than guessed-at rewrites, since
    none of those methods exist on rclpy's async goal-handle-based `ActionClient`/
    `ActionServer`. Also flags (not fixed): `actionlib_msgs.GoalStatus` constants used in
    `_TERMINAL_FAILURE` don't exist in ROS2's `action_msgs/GoalStatus` (different
    `STATUS_*` names/values), and `move_base_msgs` (ROS1-only, no ROS2 port available) is
    imported as-is. **This file cannot function correctly against ROS2 until a human
    makes a real Nav2-action design decision** -- directly downstream of
    `navigation.launch.py`'s TODO from the launch-file conversion (section 5); tracked
    here so the connection between the two is explicit.
  - **Widespread pattern**: `rospy.get_param("~name", ...)` (ROS1's private/relative
    parameter names, resolved via each node's own private namespace) has no rclpy
    equivalent -- ROS2 parameter names are flat per-node with no `~` resolution concept.
    Flagged with a TODO at every call site across `cmd_vel_bridge_basicmicro.py`,
    `shared_autonomy_manager.py`, `shared_autonomy_teleop.py`, `wheel_odom_publisher.py`
    rather than silently stripping the `~` and hoping the flat name doesn't collide with
    anything.
  - `wrist_controller/{horizontal_spoon,wrist_controller}.py`: `ServiceProxy` calls
    against `wrist_driver_interfaces/SetWristMode.srv` needed a guessed request field
    name (`request.mode = ...`) -- that package doesn't exist anywhere in this
    repo/workspace to check the real generated field name against. Flagged explicitly as
    a best-guess placeholder, same category of gap as `watchdog.py`'s `String_cmd.srv`
    guess in the safety/ batch (section 4f).
  - rclpy `Duration` lacking a `.to_sec()` method (unlike rospy's `Duration`) surfaced
    repeatedly (`cmd_vel_bridge_basicmicro.py`, `wheel_odom_publisher.py`) -- converted to
    `.nanoseconds / 1e9` with a TODO at each site rather than assuming `.to_sec()` exists
    on this rclpy version.

All 15 files verified via `ast.parse` (both by the migrating agent and independently
re-verified before commit); 14 have real diffs, `kinova.py` has none. Committed as
`ec20342f`.

### 4c. integration/ batch
6 files: `data_logger.py`, `researcher_timer.py`, `run.py` (the executive, 1607 lines
post-edit), `test_actions.py`, `test_navigate_action.py`, `transfer_calibration.py`.

  - **Canonical `init_node` placement**: `run.py`'s `if __name__ == "__main__":` block
    (was `rospy.init_node("feeding_deployment", anonymous=True)`, gated on
    `args.run_on_robot or args.use_interface`) is now `node_handle.init_node("feeding_deployment")`.
    This runs before `_Runner(...)` is constructed (which builds `ArmInterfaceClient`,
    `WebInterface`, etc. -- everything that will lazily reach for `node_handle.get_node()`),
    so it's the right place for the real deployment process's node to get its actual name.
  - `data_logger.py`: `rospy.Publisher(...)` -> `create_publisher`; also
    `.get_num_connections()` -> `.get_subscription_count()` (rclpy Publisher API rename).
  - `researcher_timer.py` has **no** `rospy` import at all (a standalone Flask app,
    deliberately ROS-independent) -- its only ROS coupling was shelling out to
    `rostopic pub -1 ...`, converted to `ros2 topic pub --once ...` (message type string
    now needs the `/msg/` middle segment, e.g. `std_msgs/msg/String`). TODO(ros2) left
    flagging that `ros2 topic pub` parses its payload as YAML, not JSON -- unverified
    against a real daemon, possible quoting/escaping risk.
  - `test_navigate_action.py` and `run.py` both previously used `anonymous=True`
    (auto-suffixed unique node name); `test_navigate_action.py` also used
    `disable_signals=True` (its own `except KeyboardInterrupt` handler relies on rospy
    NOT installing a SIGINT handler). Neither has a `node_handle`/rclpy equivalent --
    both flagged with TODO(ros2) rather than assumed safe.
  - `test_navigate_action.py`: `rospy.core.is_initialized()` -> `node_handle.has_node()`;
    `rospy.signal_shutdown(...)` -> `node_handle.shutdown()`.
  - `transfer_calibration.py` uses the `rospy_compat as rospy` minimal-diff alias (per
    the cheatsheet) since it calls bare `rospy.is_shutdown()` twice in calibration
    while-loops; only its `Publisher` construction was touched explicitly.
  - Pre-existing bug noted, not fixed: `run.py` imports `from std_msgs.msg import String`
    but never uses it anywhere (dead import, predates this migration).

All 6 files verified via `ast.parse` (both by the migrating agent and independently
re-verified before commit). Committed as `ecc3272b`.

### 4d. interfaces/ batch
4 files: `perception_interface.py` (1660 lines), `realsense_interface.py`,
`rviz_interface.py`, `web_interface.py`.

  - tf2's `TransformListener`/`TransformBroadcaster`/`StaticTransformBroadcaster` all
    updated to take the shared node explicitly (all three, not just `TransformListener`
    as the cheatsheet emphasized -- the batch correctly generalized the pattern).
  - `rospy.Time.now()` used as a `header.stamp` -> `rospy_compat.now().to_msg()` (an
    `rclpy.time.Time` is not directly assignable to a `builtin_interfaces/Time` field,
    needs `.to_msg()`). Similarly `rospy.Duration(0)` -> `rospy_compat.Duration(seconds=0).to_msg()`
    for a `Duration`-typed message field (`marker.lifetime`).
  - `realsense_interface.py`: `message_filters.Subscriber(topic, MsgType, queue_size=...,
    buff_size=...)` -> `message_filters.Subscriber(node, MsgType, topic, qos_profile=queue_size)`
    at 3 call sites; `buff_size` dropped (TODO, no rclpy equivalent). Also fixed
    `rospy.Time(secs=..., nsecs=...)` for ROS2's `builtin_interfaces/Time` field rename
    (`secs/nsecs` -> `sec/nanosec`) -- treated as unambiguous, not a TODO.
  - `perception_interface.py` kept deliberately minimal/mechanical (matches its existing
    mypy/pylint-excluded, "hardware-coupled" status). `ServiceProxy` -> `create_client` +
    `call_async` for the FT-sensor bias command, with 2 TODOs: the `.srv` request field
    name is unverified (`netft_rdt_driver` isn't installed anywhere to check against --
    see section 5's same caveat), and the general async-semantics TODO.
    **Two pre-existing bugs found, not fixed**: `ROSPY_IMPORTED` is set on the failure
    path but never on success and is otherwise unused (dead flag); `self.control_rate` is
    referenced in `getTransformationFromTF` but never assigned anywhere in the file (a
    latent `AttributeError` if that code path is ever hit).
  - `web_interface.py`: 6 publishers/subscribers converted; the 2 `latch=True` publishers
    (`skill_plan_publisher`, `settings_publisher`) now use a shared
    `QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)`.
    **Confirmed the rosbridge-facing wire protocol is unchanged**: no edits to the JSON
    message construction/parsing (`_message_callback`, `_settings_callback`,
    `_send_message`, etc.), topic names, or message types -- only the rospy plumbing
    underneath. This directly satisfies section 6's requirement.

All 4 files verified via `ast.parse` (both by the migrating agent and independently
re-verified before commit). Committed as `270fb0e7`.

### 4e. perception/ + utils/ batch
8 files: `tf_interface.py`, `utils/tf_utils.py`, `appliance_perception.py` (848 lines),
`attachment_perception.py`, `drink_perception.py`, `head_perception/ros_wrapper.py`,
`record_gesture_data.py`, `visualize_gesture_data.py`.

  - **tf2 consistency confirmed across all 3 tf2-using files** in this batch
    (`tf_interface.py`, `tf_utils.py`, `ros_wrapper.py`): identical pattern --
    `tf2_ros.Buffer()` unchanged, `TransformListener(buffer, node_handle.get_node())`,
    `TransformBroadcaster(node_handle.get_node())`. No divergence.
  - `builtin_interfaces/Time` field rename (`secs/nsecs` -> `sec/nanosec`) fixed directly
    in `tf_interface.py` and `tf_utils.py` (unambiguous, same treatment as the
    interfaces/ and misc/ batches gave the identical rename elsewhere in the codebase).
    `tf_interface.py`'s JSON sidecar log format deliberately kept its **output** keys as
    `secs`/`nsecs` for on-disk log-format compatibility -- only the source attribute
    names changed, a good example of the batch distinguishing "wire/API rename" from
    "our own log format," worth double-checking during review.
  - **Proactive, unambiguous correctness fix** (not a guess): `ros_wrapper.py`'s two
    `header.seq = 0` assignments were removed -- `std_msgs/Header` has no `seq` field at
    all in ROS2 (dropped from the message definition), so this line would have raised
    `AttributeError` immediately on any real run. Flagged here for visibility even though
    it wasn't left as a TODO, since it's a behavior change (however clearly correct).
  - `appliance_perception.py` (the active GroundingDINO `detect_items`/
    `detect_handle_and_placement` pipeline central to the current Pachirisu hardware
    work per CLAUDE.md) has only 2 unused `Publisher` calls plus `__main__` boilerplate
    touched by rospy -- the actual detection/DBSCAN/plane-fit logic has zero rospy
    touchpoints and was left completely untouched, minimizing risk to that active work.
  - `record_gesture_data.py`/`visualize_gesture_data.py` both depend on an external
    `rs_ros.RealSenseROS` class outside this 8-file scope; if it uses `rospy` internally,
    it was NOT migrated -- flagged as a likely runtime gap for whoever picks this up.
  - `anonymous=True` dropped in `record_gesture_data.py`, `visualize_gesture_data.py`,
    and `ros_wrapper.py`'s `__main__`, each with its own TODO(ros2), consistent with how
    every other batch handled this same rospy kwarg.
  - Pre-existing bugs noted, not fixed: `ros_wrapper.py`'s `__main__` passes a
    `HeadPerception` instance into `HeadPerceptionROSWrapper`'s constructor positionally,
    but the wrapper's `__init__` silently discards it and builds its own separate
    `HeadPerception` instance internally -- looks like a real bug, unrelated to this
    migration. `appliance_perception.py`'s `handle_points_pub`/`handle_center_pub` and
    the equivalent pair in `attachment_perception.py` are constructed but (at least the
    appliance ones) never published to anywhere in the file -- dead code, pre-existing.

All 8 files verified via `ast.parse` (both by the migrating agent and independently
re-verified before commit). Committed as `c243984c`.

### 4f. safety/ batch (safety-critical -- read this one closely before merging)
8 files: `bulldog.py`, `collision_sensor.py`, `collision_threshold.py`,
`estops_publisher.py`, `estop_udp_bridge.py`, `sensor_diag_logger.py` (864 lines),
`transfer_button_listener.py`, `watchdog.py` (301 lines). Ported by careful reading
only -- **nothing in this batch was ever executed, run, or imported for real**.

19 `TODO(ros2)` comments across this batch (more than any other, correctly -- this is
the e-stop/liveness layer). Every publish/subscribe queue depth, `latch=True` ->
`DurabilityPolicy.TRANSIENT_LOCAL` conversion, and safety-relevant timing constant
(frequency thresholds, debounce windows, UDP timeouts, e.g. `FT_FREQUENCY_THRESHOLD=300`,
`CAMERA_FREQUENCY_THRESHOLD=2`, `EXPERIMENTOR_ESTOP_FREQUENCY_THRESHOLD=30`,
`BULLDOG_STATUS_TIMEOUT_S=0.5`) was preserved byte-for-byte.

**Real bugs caught and fixed** (not just syntax, spot-checked and confirmed correct
before committing):
  - `collision_sensor.py` imported a top-level `SetCollisionThreshold*Response`
    (genpy/ROS1-style flat naming); the rosidl-generated ROS2 module nests these as
    `SetCollisionThreshold.Request`/`.Response` (confirmed against the real `colcon
    build` from section 2) -- fixed to the correct nested attribute access.
  - `sensor_diag_logger.py`: `msg.header.stamp.to_sec()` doesn't exist on ROS2's
    `builtin_interfaces/Time` (only `.sec`/`.nanosec` fields) -- fixed to
    `stamp.sec + stamp.nanosec * 1e-9`. Its self-healing resubscribe logic called
    `sub.unregister()`, which `rclpy.Subscription` has no equivalent for -- fixed to
    `node.destroy_subscription(sub)`.
  - `estop_udp_bridge.py`: `rospy.Duration(5.0)` (positional) -- `rclpy.duration.Duration`
    is keyword-only -- fixed to `Duration(seconds=5.0)`.

**Flagged, deliberately NOT silently fixed -- needs a human reviewer decision**
(spot-checked directly, both are real and well-explained in-code, not just in the
agent's report):
  - `collision_threshold.py`: ROS1's `rospy.wait_for_service(timeout=...)` **raised**
    on timeout, aborting before any threshold mutation. The rclpy port
    (`rospy_compat.wait_for_service`) returns `True`/`False`, and the return value is
    currently **unchecked** at the call site -- a timed-out wait falls through into a
    call that will block forever in `spin_until_future_complete` with no timeout. Left
    as an explicit TODO at the exact line (`collision_threshold.py:43-49` per the
    in-code comment) rather than guessed at, since whether to raise, return early, or
    add a call-level timeout is a real design decision.
  - `collision_sensor.py`: two subscribers had no explicit rospy `queue_size` (rospy's
    unbounded-by-default); depth=10 was picked for both ROS2 conversions and flagged as
    a guess needing review.
  - `watchdog.py`: the FT-sensor-bias `ServiceProxy` call (`netft_rdt_driver`'s
    `String_cmd.srv` -- a package with, per CLAUDE.md, no known ROS2 port at all) had
    its request field name **guessed** as `.cmd`; no build artifact exists anywhere to
    confirm the real field name since the package isn't installed. Explicitly flagged as
    unverified.
  - `anonymous=True` (4 files: bulldog.py, watchdog.py, estop_udp_bridge.py,
    sensor_diag_logger.py) replaced with a pid-suffixed node name -- uniqueness is not
    guaranteed the same way rospy's `anonymous=True` guaranteed it (rospy hashes in a
    random component too). `sensor_diag_logger.py`'s `disable_signals=True` also
    flagged: its own `except KeyboardInterrupt:` (documented as this tool's "stop early
    and still write the summary" path) may depend on rospy's non-default SIGINT
    handling to work the way it currently does.

Pre-existing, cosmetic-only issue noted, not touched: `bulldog.py` and `watchdog.py`
each had `import rospy` duplicated twice at the top of the original file (harmless,
collapsed naturally during the migration edit).

All 8 files verified via `ast.parse` (both by the migrating agent and independently
re-verified before commit, including a direct spot-check of the
`collision_sensor.py`/`collision_threshold.py` fixes above). Committed as `3fa5d509`.

### 4g. misc/ batch
6 files: `check_ft_readings.py`, `drink_manipulation_test.py`,
`grab_head_perception_from_rosbag.py`, `recreate_joint_limit.py`, `speak.py`,
`startup_selftest.py`.

  - `drink_manipulation_test.py`: tf2 listener/broadcaster updated for the required-node
    change; `message_filters.Subscriber` reordered to `(node, MsgType, topic)` at all 4
    call sites; `builtin_interfaces/Time` field rename (`secs/nsecs` -> `sec/nanosec`)
    fixed directly as an unambiguous rename, consistent with how the interfaces/ batch
    handled the same rename in `realsense_interface.py`.
  - `grab_head_perception_from_rosbag.py`: `rosbag.Bag(...)`/`read_messages()` left
    **completely intact** with a TODO flagging `rosbag2_py` as a real, unattempted port
    -- correctly followed the instruction not to guess a bag-format API translation.
  - `startup_selftest.py`: **found and fixed a real correctness gap, not just a syntax
    issue** -- `test_transfer_button`'s polling loop relied on rospy's implicit
    background-thread callback dispatch (a subscription callback updating shared state
    while the main loop just slept); rclpy has no implicit spinning, so that callback
    would simply never have fired. Fixed by replacing the bare `time.sleep(0.05)` with
    `rclpy.spin_once(node_handle.get_node(), timeout_sec=0.05)`. This is exactly the kind
    of subtle behavioral gap the task was concerned about -- worth a human's eyes to
    confirm the fix is right, but it's a real fix, not a guess.
  - `anonymous=True`/`disable_signals=True` dropped across
    `grab_head_perception_from_rosbag.py`, `recreate_joint_limit.py`, `speak.py`,
    `startup_selftest.py` -- each flagged with its own TODO(ros2), consistent with how
    the integration/ batch handled the same rospy kwargs.
  - `speak.py`: `except rospy.ROSInterruptException` -> `except rospy_compat.ROSInterruptException`,
    with a TODO noting this except clause is now effectively dead code (`rclpy.spin()`
    doesn't raise that exception on shutdown, per `rospy_compat`'s own documented caveat).
  - Pre-existing bugs noted, not fixed, in `drink_manipulation_test.py`: `rgbdCallback`
    doesn't `return` after catching `CvBridgeError` (falls through to possibly-undefined
    `rgb_image`/`depth_image`); an ArUco corner-collection loop overwrites the `corners`
    variable each iteration so only the last detected marker's corners ever survive as
    `landmarks` -- looks like a real logic bug predating this migration.

All 6 files verified via `ast.parse` (both by the migrating agent and independently
re-verified before commit). Committed as `93726fbe`.

## 5. Launch files: all 15 converted, XML -> `launch.py`

Converted `arm.launch`, `arm_sensors.launch`, `base_sensors.launch`,
`cartographer_localization.launch`, `cartographer_mapping.launch`, `description.launch`,
`drift_traces.launch`, `lidar_cell_map_debug.launch`, `navigation.launch`,
`rplidar_a1.launch`, `sensors.launch`, `shared_autonomy.launch`, `sim.launch`,
`zed_drift_test.launch`, `zed.launch` to `<name>.launch.py` using `launch`/`launch_ros`
idioms: `Node`, `IncludeLaunchDescription` + `PythonLaunchDescriptionSource`,
`DeclareLaunchArgument` + `LaunchConfiguration`, `GroupAction` + `PushRosNamespace` (for
`<group ns=...>`), `IfCondition` (for `if=`), and `ExecuteProcess` (for the one
`rosbag record` node, since ROS2 has no in-graph "record node," `ros2 bag record` is a
CLI verb). Deleted the old `.launch` files (git history retains them).

**Every piece of the original tribal-knowledge commentary was preserved verbatim** --
these launch files are full of dated, incident-specific tuning notes (USB controller
port assignments, why a specific recovery behavior was disabled, TF-tree ownership
diagrams, exact calibration constants with the date and method they were measured) that
document real operational history, not boilerplate. Losing any of it in the conversion
would have been a real regression independent of the ROS2 migration itself.

### Confirmed-available ROS2 packages (via `apt-cache search` on this box)
`rviz2`, `rplidar_ros` (same name), `rosbridge_suite`, `cartographer_ros`,
`robot_localization`. **Confirmed NOT available:** `move_base` or any ROS2
"move-base"-named package (zero `apt-cache search` hits, vs. real hits for
`ros-jazzy-nav2-*`) -- Nav2 is the only forward path for navigation, and it is a genuinely
different architecture (see below).

### What was NOT mechanically portable -- flagged, not guessed

  - **`navigation.launch.py`**: `move_base` (ROS1's monolithic global-planner +
    local-planner + costmap + recovery-chain node) has no ROS2 package at all. Nav2
    splits this across several lifecycle-managed nodes (`bt_navigator`,
    `controller_server`, `planner_server`, `behavior_server`, ...) orchestrated by a
    behavior-tree XML, with a different costmap-plugin YAML schema. `global_planner` and
    `teb_local_planner` both have Nav2-plugin ports, but wiring them up (BT XML, lifecycle
    bringup, the new costmap YAML schema) is a real navigation-stack migration project,
    not a syntax conversion. Left as an explicit, loud `LogInfo` TODO placeholder with
    every original tuned param preserved in a Python dict
    (`ORIGINAL_ROS1_MOVE_BASE_PARAMS`) so nothing gets silently lost. The file's own
    `cmd_vel_bridge_basicmicro` node (a real, in-scope rospy->rclpy migration target) IS
    fully ported.
  - **`lidar_cell_map_debug.launch.py`**: ROS1's standalone `costmap_2d_node` executable
    (a debug-only bare costmap outside `move_base`) has no ROS2 equivalent -- Nav2's
    `nav2_costmap_2d` is a library used *inside* a lifecycle-managed server, not a
    standalone node. Left as an explicit TODO with the original tuned costmap params
    preserved in a dict (`ORIGINAL_ROS1_COSTMAP_PARAMS`); launches nothing.
  - **`zed_wrapper`** (used by `base_sensors.launch.py`, `sensors.launch.py`,
    `zed.launch.py`): the ROS2 ZED driver (`zed-ros2-wrapper`) was **not verified
    installed** on this box this session -- unlike `rviz2`/`rplidar_ros`/etc., no
    `apt-cache search zed` was run against a confirmed hit. Its ROS2 launch API also
    differs from ROS1's `zedm.launch` (different file name, `zed_camera.launch.py`, and
    likely different argument surface). Left as a try/except
    `get_package_share_directory("zed_wrapper")` guard with a `LogInfo` TODO if it's
    missing, rather than assuming it's there. Additionally: ROS1's pattern of setting
    `/$(arg camera_name)/zed_node/...` params as bare top-level `<param>` tags *after* the
    `<include>` relies on roslaunch's global parameter server pre-seeding values before
    any node starts -- ROS2 has no global parameter server, so this pattern has no direct
    translation at all. All such params are preserved in an
    `ORIGINAL_ROS1_ZED_PARAMS` dict in each affected file for whoever finishes a real
    ROS2 zed_wrapper integration.
  - **`netft_rdt_driver`** (arm_sensors.launch.py, sensors.launch.py): per CLAUDE.md, this
    driver "has no public distribution at all" even for ROS1. Nothing to include; left as
    an explicit `LogInfo` TODO.
  - **`robot_localization`**'s ROS1 executable name `ekf_localization_node` was **renamed**
    to `ekf_node` in its ROS2 port (per the package's own public ROS2 documentation --
    not independently confirmed against an installed binary on this box, since the deb
    isn't installed here, only found via `apt-cache search`). Used `ekf_node` in
    `sensors.launch.py`; verify with `ros2 pkg executables robot_localization` before
    relying on it.
  - **`config/nav/*.yaml` and `config/nav/ekf_zed_wheel.yaml`**: still in **flat ROS1
    rosparam format** (confirmed: `grep -c ros__parameters` returns 0 for every file in
    `config/nav/`). ROS2 node-parameter YAML requires a `<node_name>:\n  ros__parameters:\n
    ...` wrapper per node. These files were passed through **unmodified** and referenced
    as-is from the launch files (`parameters=[path/to/file.yaml]`, which `launch_ros`
    happily accepts as a raw path) rather than reformatted -- reformatting 9 files of
    real, physically-tuned navigation/EKF parameters (costmap footprints, TEB velocity
    limits, EKF sensor fusion config) without the ability to bench-test on hardware risked
    silently corrupting real tuning values. **A human with nav expertise needs to
    reformat these and re-validate on hardware or in simulation** before any Nav2-based
    launch will actually load them correctly.

### Validated (for real, not just by inspection)

  1. `python3 -c "import ast; ast.parse(...)"` -- all 15 files parse cleanly.
  2. **Actually executed each file's `generate_launch_description()`** under a live
     `launch`/`launch_ros` import from `/opt/ros/jazzy` (system Python 3.12) and confirmed
     each returns a real `LaunchDescription` object with the expected number of entities.
     Initially every file that calls `get_package_share_directory("feeding_deployment")`
     failed with `PackageNotFoundError` -- **this is the same root cause as section 3's
     colcon-python-setup-py gotcha** (the package was never successfully registered in the
     ament index because it never finished a real `colcon build`), not a bug in the launch
     files. Confirmed this by registering a **fake** ament-index entry (a scratch
     `AMENT_PREFIX_PATH` with `share/ament_index/resource_index/packages/feeding_deployment`
     and `share/feeding_deployment/{launch,config,rviz,urdf,package.xml}` symlinked
     straight at this worktree) and re-running -- all 15 files then load successfully.
     This proves the launch files are structurally sound; it does **not** prove any node
     they reference actually starts (most reference nodes that are themselves
     rospy->rclpy migration targets covered in section 4, and hardware-adjacent ones were
     never executed here per the safety constraint).

## 6. Webapp / rosbridge

Per the task brief's framing, this needed only confirming the wire protocol is unchanged
-- it is: `rosbridge_suite` exists for ROS2 Jazzy (**confirmed available**:
`ros-jazzy-rosbridge-suite`) and speaks the same JSON-over-websocket rosbridge protocol
`roslib` (the webapp's frontend library) already expects. Only the launch side changes
(package name `rosbridge_server`, executable `rosbridge_websocket` -- already reflected in
the converted `arm.launch.py`/`sim.launch.py`/`sensors.launch.py`). `web_interface.py`
(the server-side ROS integration for the webapp) is covered by section 4d's interfaces/
batch for its own internal rospy->rclpy port; nothing about its rosbridge-facing behavior
was changed.

## 7. Explicitly NOT attempted (do not read silence as "verified")

  - Running `arm_server.py`, `bulldog.py`, `watchdog.py`, or anything that opens a
    serial/USB/network connection to real hardware, an RPC port, or a live ROS graph.
  - "Testing" `safety/` code by executing it -- ported by careful reading only.
  - Any real Nav2 migration (BT XML, lifecycle bringup, costmap YAML reformatting) --
    `navigation.launch.py` and `lidar_cell_map_debug.launch.py` are explicit placeholders.
  - Reformatting `config/nav/*.yaml` / `ekf_zed_wheel.yaml` into ROS2's
    `ros__parameters:` schema.
  - Fixing the colcon-python-setup-py/setuptools incompatibility (section 3) -- needs a
    human decision (tooling upgrade vs. accepting the constraint) outside this worktree's
    scope.
  - Verifying `zed_wrapper` (ROS2 ZED driver) is actually installed/has the assumed
    launch API anywhere.
  - **`README.md` was NOT updated** -- it is entirely ROS1 procedure documentation
    (`roslaunch feeding_deployment sensors.launch`, `source devel/setup.bash`,
    `catkin build`, etc.), and per CLAUDE.md's own instruction that file is
    "deployment-procedure documentation ... not something to re-derive here." Rewriting
    it for ROS2 (`ros2 launch feeding_deployment sensors.launch.py`, `source
    install/setup.bash`, `colcon build`) is a real, separate documentation task, not
    attempted. **Also worth flagging independent of the migration**: several
    `roslaunch` invocations in `README.md` already reference launch file names that
    don't exist in this repo's `launch/` directory at all (e.g. `vention_description.launch`,
    `vention_rplidar_a1.launch`, `vention_odm_d435.launch`, `vention_cartographer_lidar.launch`
    -- the real files are `description.launch`(`.py`), `rplidar_a1.launch`(`.py`), no
    `odm_d435`/`cartographer_lidar` equivalent found at all) -- this doc was already stale
    against ROS1 reality before this migration touched anything.
  - `scripts/*.py` (18 additional files there also `grep`-match `rospy`, e.g.
    `scan_gate.py`, `drift_lock.py`, `zed_pose_to_odom_feedback.py`, referenced by several
    converted launch files as `feeding_deployment` executables) -- **out of scope**: the
    task's own confirmed-via-grep scope was explicitly "52 Python files under `src/` and
    `integration/`," and `scripts/` is neither. These are real gaps for anyone trying to
    actually run the converted launch files (several `Node` actions reference these
    scripts by name), tracked here so it isn't a silent surprise.
  - Any actual `ros2 run`/`ros2 launch` execution, anywhere, at any point.

## 8. Prioritized human-review punch list before this goes anywhere near the real arm

1. **`safety/` batch** (bulldog.py, watchdog.py, collision_sensor.py,
   collision_threshold.py, estop_udp_bridge.py, estops_publisher.py,
   sensor_diag_logger.py, transfer_button_listener.py) -- e-stop/liveness layer, ported by
   reading only, never executed. Check every `# TODO(ros2)` in this batch first (see
   section 4f once filled in).
2. **`control/robot_controller/kinova.py`** and `preset_actions/*` -- zeroes arm torque
   offsets, sends the arm to saved joint configs; mounting/environment-specific and
   safety-adjacent. Check section 4b's report closely.
3. **`navigation.launch.py`** / Nav2 migration -- currently a placeholder; the base won't
   navigate autonomously at all until a human does the real Nav2 port.
4. **`config/nav/*.yaml` + `ekf_zed_wheel.yaml`** reformatting to ROS2's parameter schema
   -- currently unmodified/incompatible with any real ROS2 nav node.
5. **The colcon-python-setup-py/setuptools incompatibility** (section 3) -- blocks
   actually installing/running `feeding_deployment` as a ROS2 package at all until
   resolved.
6. **tf2 call sites** across `actions/navigate.py`, `interfaces/perception_interface.py`,
   `interfaces/realsense_interface.py`, `interfaces/rviz_interface.py`,
   `perception/tf_interface.py`, `perception/head_perception/ros_wrapper.py`,
   `utils/tf_utils.py`, `misc/drink_manipulation_test.py`,
   `misc/grab_head_perception_from_rosbag.py` -- the ROS2 API differences here
   (`TransformListener`/`TransformBroadcaster` now requiring an explicit `node` arg) are
   easy to get subtly wrong; check section 4's per-file notes for consistency across all
   of these.
7. **`actions/navigate.py` and `control/base_controller/shared_autonomy_manager.py`**'s
   actionlib -> `rclpy.action` ports -- a real async-architecture change (blocking
   `wait_for_result()` -> futures/callbacks), flagged with TODOs rather than silently
   ported; needs real testing (in sim first, not on the real base) before trusting
   takeover/goal-cancellation behavior.
8. **`scripts/*.py`** referenced by the converted launch files (`scan_gate.py`,
   `drift_lock.py`, `zed_pose_to_odom_feedback.py`, `wheel_odom_publisher.py` is actually
   in-scope and covered, `gyro_bias_estimator.py`, `zed_svo_recorder.py`) -- these are
   NOT migrated (out of the confirmed 52-file scope) but several launch files still
   reference them by name; anyone trying to actually run `sensors.launch.py` etc. will
   hit missing/un-ported scripts.
