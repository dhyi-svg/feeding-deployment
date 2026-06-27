## Requirements

- Python 3.10+
- Tested on Ubuntu 20.04

## Pre-Installation

1. Install ROS and rospy.
2. Install [pyaudio](https://pypi.org/project/PyAudio/).


## Installation

1. Recommended: create and source a virtualenv or a conda environment
2. `pip install -e ".[robot, develop]"` for full install or `pip install -e .` for only preference learning setup

## Run Feeding Demo on Real Robot
1. Run the arm controller server on the NUC:
   - ssh to the NUC: `sshnuc` with lab password
   - [only for inside-mouth bite transfer] zero the arm torque offsets:
        - Alias `set_zeros` on NUC
        - Otherwise, run the following commands:
             - `conda activate controller`
             - `cd ~/feeding-deployment/src/feeding_deployment/robot_controller`
             - `python kinova.py`
   - run the controller server:
        - Alias `launch_arm` on NUC
        - Otherwise, run the following commands:
             - `conda activate controller`
             - `cd feeding-deployment/src/feeding_deployment/robot_controller`
             - `python arm_server.py`
1b. Run the base controller server on the NUC:
   - The base Arduino is plugged into the **NUC** (not the compute box), so the
     Bulldog e-stop also stops the base. The cmd_vel bridge and teleop scripts on
     the compute box drive the base over RPC.
   - ssh to the NUC: `sshnuc` with lab password
   - run the base server:
        - Alias `launch_base` on NUC
        - Otherwise, run the following commands:
             - `conda activate controller`
             - `cd feeding-deployment/src/feeding_deployment/control/base_controller`
             - `python base_server.py`
   - _Note:_ bulldog now **requires** both `arm_server.py` and `base_server.py`;
     it refuses to start if either RPC server is down.
2. Run bulldog on the NUC:
   - ssh to the NUC: `sshnuc` with lab password
   - run bulldog with alias `launch_bulldog`
3. Run a roscore on the compute system: `roscore`
4. Launch the roslaunch on compute system for sensors / visualizations:
   - Alias `launch_sensors` on compute system
   - Otherwise,run the following commands from the root of your ROS workspace:
        - `conda activate feed`
        - `source devel/setup.bash`
        - `roslaunch feeding_deployment sensors.launch`
5. Launch the watchdog on compute system:
   - Alias `launch_watchdog` on compute system
   - Otherwise,run the following commands from the root of your ROS workspace:
        - `conda activate feed`
        - `source devel/setup.bash`
        - `cd ~/deployment_ws/src/feeding-deployment/src/feeding_deployment/integration`
        - `chmod +x launch_robot.sh`
        - `./launch_robot.sh`
4. Start feeding utensil:
   - Alias `launch_utensil` on compute system
   - Otherwise, run the following commands from the root of your ROS workspace:
        - `conda activate feed`
        - `source devel/setup.bash`
        - `rosrun wrist_driver_ros wrist_driver`  
   - _Important Note:_ To shutdown this node, press Ctrl + / (Signal handling is setup to shutdown cleanly)
5. Start the web application:
   - Make sure that the feeding laptop's WiFi is off (so that the webapp only launches on the router IP)
   - Alias `launch_app` on compute system
   - Otherwise, run the following commands from the root of your ROS workspace:
        - `conda activate feed`
        - `source devel/setup.bash`
        - `cd ~/deployment_ws/src/feedingpage/vue-ros-demo`
        - `npm run serve`
   - On a browser connected to FeedingDeployment-5G (on the laptop or the iPad), open the following webpage: `http://192.168.1.2:8080/#/task_selection`  
6. Start the cluster:
   - If not on cornell network, make sure that CISCO VPN is on.
   - ssh to the cluster: `sshcluster` or (ssh rj277@unicorn-login-01.coecis.cornell.edu)
   - launch the molmo server: `launch_molmo`
6. Run the feeding demo:
   - Make sure that the feeding laptop's WiFi is on and connected to the internet so that ChatGPT API works 
   - Alias `run_demo` on compute system
   - Otherwise,run the following commands from the root of your ROS workspace:
        - `conda activate feed`
        - `source devel/setup.bash`
        - `cd src/feeding-deployment/src/feeding_deployment/integration`
        - `python run.py --user feeding_deployment --run_on_robot --use_interface --no_waits`
   - _Important Note:_ If you want to resume from some state (state names: after_utensil_pickup, after_bite_pickup, last_state), use: `python run.py --user tests --run_on_robot --use_interface --no_waits --resume_from_state after_utensil_pickup` (replace after_utensil_pickup with appropriate state name).

### Moving the robot to preset configurations

You can move the robot to preset configurations by running:
- Alias `cd_actions` on compute system
- `python retract.py` (you can also send it to transfer.py and acquisition.py) 

### Calibrate tool offset for inside-mouth transfer

1. Grasp the tool and move to before bite transfer position.
2. Calibrate tool:
   - Alias `cd_demo` on compute system
   - Otherwise, run the following commands from the root of your ROS workspace:
        - `conda activate feed`
        - `source devel/setup.bash`
        - `cd src/feeding-deployment/src/feeding_deployment/integration`
   - `python transfer_calibration.py --tool <tool_name>` where <tool_name> is one of "fork", "drink" and "wipe"
3. Manually (using buttons on the robot) move the robot to the intended inside-mouth transfer config, and press [ENTER] in the script above to record it. 
4. To test the tool calibration:
   - Alias `cd_demo` on compute system
   - Otherwise, run the following commands from the root of your ROS workspace:
        - `conda activate feed`
        - `source devel/setup.bash`
        - `cd src/feeding-deployment/src/feeding_deployment/integration` 
   - `python transfer_calibration.py --tool <tool_name> --test` where <tool_name> is one of "fork", "drink" and "wipe"
  
## Running the demo with tmux (compute + NUC)

Helper scripts under `scripts/` build labeled tmux sessions and add a one-key
restart (`prefix + r`). Each machine runs its own local session, so the session
(and its processes) survive your SSH client disconnecting.

### Compute: `scripts/feeding-compute.sh`
Builds session `feeding` as an 8-pane 2x4 grid (run on the compute box):

```
1 roscore          2 launch_sensors   3 launch_app       4 launch_utensil
5 launch_watchdog  6 cartographer_localization  7 shared_autonomy  8 run.py
```

- Each command is **pre-typed but not executed** — fire them in order. Pane 8
  (`run.py`) is pre-typed in the integration dir so you can edit it before Enter.
- `prefix + r` restarts the **bottom row only** (5-8), leaving roscore/sensors/
  app/utensil (1-4) untouched: Ctrl+C 5-8 -> watchdog -> 10s -> cartographer ->
  5s -> shared_autonomy -> pre-type run.py. Timings are tunable via the
  `RESTART_GRACE` / `POST_WATCHDOG_DELAY` / `INTER_DELAY` constants at the top.
- Run: `./scripts/feeding-compute.sh`

### NUC: `scripts/feeding-nuc.sh`
Builds session `robot` as 3 stacked panes (run on the NUC):
`launch_arm` / `launch_base` / `launch_remote_bulldog`.

- `prefix + r` restarts all three after an e-stop: Ctrl+C all -> relaunch
  arm + base -> bulldog ~3s later (bulldog needs both RPC servers up first).
- Run: `./scripts/feeding-nuc.sh`

### Permanence
Each script installs `prefix + r` at build time (lasts for the tmux server's
life). To persist it across a full tmux-server restart, add the matching
`bind r ...` line to that machine's `~/.tmux.conf` (see `scripts/nuc.tmux.conf`
for the NUC template). Panes are resolved by **geometry**, not title, because
programs like roscore/htop overwrite pane titles.

## Run Feeding Demo in Simulation
1. Launch the roslaunch for visualization / publish tfs:
   - Navigate to the launch files: `cd launch`
   - Launch: `roslaunch sim.launch`
2. Run the feeding demo:
   - Navigate to integration scripts: `cd src/feeding_deployment/integration`
   - Run demo: `python demo.py`

## Random

- To check FT readings: `rostopic echo /forque/forqueSensor`
- IP for robot: 192.168..10
- IP for webapp: `http://192.168.1.2:8080/#/task_selection`
- To check if wrist controller is working: `rostopic pub -1 /cmd_wrist_joint_angles wrist_driver_interfaces/SimpleJointAngleCommand '{q0: 0.0, q1: 0.0}'`

## Build navigation map + save named base locations (feeding_deployment)

`roslaunch feeding_deployment vention_navigation.launch` does **not** load map files by itself.
It starts `move_base`, which uses whatever `/map` and `map -> odom` are currently published.

The workflow below uses Cartographer-native saved state (`.pbstream`).

### Part 1: First-time mapping + save map state + save named locations

From `/home/isacc/deployment_ws`, source your workspace in each terminal: `source devel/setup.bash`

1. Start core and robot sources:
   - `roscore`
   - `roslaunch feeding_deployment vention_description.launch`
   - `roslaunch feeding_deployment vention_rplidar_a1.launch`
   - `roslaunch feeding_deployment vention_odm_d435.launch`
   - `roslaunch feeding_deployment vention_cartographer_lidar.launch`
2. Build map and save Cartographer state (`.pbstream`):
   - `cd src/feeding-deployment`
   - `python src/feeding_deployment/integration/build_map_interactive.py --pbstream-file /home/isacc/deployment_ws/src/feeding-deployment/config/maps/vention_map.pbstream`
   - Optional: also export YAML/PGM snapshot: add `--save-occupancy-snapshot`
   - By default this script does **not** call `/finish_trajectory`, so Cartographer can keep publishing `map -> odom` for follow-up steps like named location capture.
   - Optional: if you explicitly want to finish the trajectory during save, add `--finish-trajectory-before-save`
3. Save named navigation locations:
   - `python src/feeding_deployment/integration/capture_named_locations.py --locations-file /home/isacc/deployment_ws/src/feeding-deployment/config/nav_named_locations.yaml`
   - This captures in order: `fridge`, `microwave`, `table`, `sink`.

### Part 2: Actual deployment (reuse saved map)

From `/home/isacc/deployment_ws`, source your workspace in each terminal: `source devel/setup.bash`

1. Start core and robot sources:
   - `roscore`
   - `roslaunch feeding_deployment vention_description.launch`
   - `roslaunch feeding_deployment vention_rplidar_a1.launch`
   - `roslaunch feeding_deployment vention_odm_d435.launch`
2. Start Cartographer localization from saved state:
   - `roslaunch feeding_deployment vention_cartographer_localization.launch load_state_filename:=/home/isacc/deployment_ws/src/feeding-deployment/config/maps/vention_map.pbstream`
3. Start navigation:
   - `roslaunch feeding_deployment vention_navigation.launch`

In deployment mode, Cartographer publishes `/map` and `map -> odom` from the saved `.pbstream`, and `move_base` consumes that.

By default, named locations are written to `config/nav_named_locations.yaml`.
`NavigateHLA` reads this file automatically. To use a different file, set:
`export FEEDING_NAV_LOCATIONS_FILE=/absolute/path/to/your_locations.yaml`

## Check Installation

Run `./run_ci_checks.sh`. It should complete with all green successes in 5-10 seconds.


# Setting up Vention Navigation Stack

# Navigation Dependencies
Create a ROS workspace in your home directory:
```
mkdir vention_dependencies_ws
```
We use Cartographer for multi-lidar SLAM. Follow their instructions at https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html.
Be sure to set this up in vention_dependencies_ws.

Source vention_dependencies_ws before continuing.

# feeding_deployment

Download the Vention ROS package, and put it into catkin_ws.

Download the URDF at
```
https://drive.google.com/file/d/1OZAdcuAua0Nr7p6ITxTQDwUMFzZjeR8F/view?usp=sharing
```
Put the URDF into feeding_deployment/urdf/meshes.

build the workspace with

```
catkin build
```


# Teleoperation with Xbox
```
python src/feeding_deployment/src/controllers/basicmicro_arduino/vention_controller.py
```

# Navigation 

## Load Vention RobotModel
```
roslaunch feeding_deployment description.launch
```

## Start Lidar
You may need to change the usb id/path of lidars in the launch file.
```
roslaunch feeding_deployment vention_rplidar_a1.launch
```

## Start ZED Camera
We are using the ZED built-in VIO.
We use the IMU for odom -> vention_base_link.
```
roslaunch feeding_deployment vention_zed_pose.launch
```

## Start Cartographer For SLAM

We use cartographer for map -> odom

For building map:
```
roslaunch feeding_deployment vention_cartographer_lidar.launch
```
Save map using 
```
python src/feeding_deployment/scripts/build_map_interactive.py --pbstream-file /home/isacc/deployment_ws/src/feeding_deployment/maps/emprise_572_map.pbstream
```

For using against existing map:
```
python src/feeding_deployment/scripts/build_map_interactive.py --pbstream-file /home/isacc/deployment_ws/src/feeding_deployment/maps/emprise_572_map.pbstream
```

## Start move_base For Navigation

First make sure we are publishing the odom link required:

```
python src/feeding_deployment/scripts/zed_pose_to_odom_feedback.py
```

You may need to change the Arduino usb id cmd_vel_bridge_basicmicro.py.
```
roslaunch feeding_deployment vention_navigation.launch
```

## Rviz

Open RViz and use the config in rviz/vention.rviz

You can move the base by giving it a 2D nav goal in RViz.

## Capture Named Locations

```
python src/feeding_deployment/scripts/capture_named_locations.py --locations sink_easy --locations-file /home/isacc/deployment_ws/src/feeding-deployment/config/nav_named_locations.yaml
```

## [Feeding-Deployment] Test Navigation

Check that you are in feed conda env
```
python /home/isacc/deployment_ws/src/feeding-deployment/src/feeding_deployment/integration/test_navigate_action.py
```


1. ```roslaunch feeding_deployment sensors.launch```
2. navigation.launch + 
 - cartographer_localization.launch
 - cartographer_mapping.launch (map for the first time)

python src/feeding_deployment/scripts/build_map_interactive.py \
    --pbstream-file /home/isacc/deployment_ws/src/feeding_deployment/maps/4-28.pbstream

Teleoperate with a controller: ```python src/feeding_deployment/src/controllers/basicmicro_arduino/vention_controller.py```


ERRORS:

Saved system state -> last_state.p, 21_stow_utensil.p
Refining PickPlateFromTable(plate, table)
Executing parameterized policy PickPlateFromTable with bindings:
  Speed = medium
  HandleColor = [85, 83, 132]
  ColorRange = 0.1
Picking plate from table ...
Got images
Found 224 pixels in mask
DBSCAN found no clusters.
Got images
Found 159 pixels in mask
DBSCAN found no clusters.
Got images
Found 141 pixels in mask
No valid 3D points from mask.
Got images
Found 126 pixels in mask
Waiting for required message from the web interface ...
Received message on /webapp_to_robot:  {"state":"detection_confirm","status":"redo","detection_type":"attachment"}
Received message on /webapp_to_robot:  {"state":"detection_confirm","status":"redo","detection_type":"attachment"}
Received required message from the web interface
Attachment detection rejected by user. Re-running attachment perception ...
Got images
Found 231 pixels in mask
No valid 3D points from mask.
Got images
Found 279 pixels in mask
DBSCAN found no clusters.
Got images
Found 200 pixels in mask
No valid 3D points from mask.
Got images
Found 176 pixels in mask
DBSCAN found no clusters.
Got images
Found 294 pixels in mask
No valid 3D points from mask.
Got images
Found 285 pixels in mask
No valid 3D points from mask.
Got images
Found 309 pixels in mask
No valid 3D points from mask.
Got images
Found 374 pixels in mask
No valid 3D points from mask.
Got images
Found 249 pixels in mask
No valid 3D points from mask.
Got images
Found 527 pixels in mask
No valid 3D points from mask.
Got images
Found 413 pixels in mask
No valid 3D points from mask.
Got images
Found 272 pixels in mask
No valid 3D points from mask.
Got images
Found 252 pixels in mask
No valid 3D points from mask.
Got images
Found 357 pixels in mask
DBSCAN found no clusters.
Got images
Found 226 pixels in mask
No valid 3D points from mask.
Got images
Found 221 pixels in mask
No valid 3D points from mask.
Got images
Found 306 pixels in mask
No valid 3D points from mask.
Got images
Found 375 pixels in mask
No valid 3D points from mask.
Got images
Found 399 pixels in mask
No valid 3D points from mask.
Got images
Found 434 pixels in mask
No valid 3D points from mask.
HLA execution failed: Could not detect attachment pose
Aborting task and returning to task selection page.
Sending message to web interface to move to task selection page with last task type:  None
[learn] Updating memory models (day 1) ...
  [long_term_memory_model] Updating summary (day 1) ...
Traceback (most recent call last):
  File "/home/isacc/deployment_ws/src/feeding-deployment/src/feeding_deployment/integration/run.py", line 1459, in <module>
    runner.run()
  File "/home/isacc/deployment_ws/src/feeding-deployment/src/feeding_deployment/integration/run.py", line 831, in run
    self._finalize_preference_session()
  File "/home/isacc/deployment_ws/src/feeding-deployment/src/feeding_deployment/integration/run.py", line 761, in _finalize_preference_session
    self._pref_session.finalize_meal(day)
  File "/home/isacc/deployment_ws/src/feeding-deployment/src/feeding_deployment/integration/preference_session.py", line 334, in finalize_meal
    self._model.update(
  File "/home/isacc/deployment_ws/src/feeding-deployment/src/feeding_deployment/preference_learning/methods/prediction_model.py", line 269, in update
    self.long_term_memory_model.add_episode(ep_txt)
  File "/home/isacc/deployment_ws/src/feeding-deployment/src/feeding_deployment/preference_learning/methods/long_term_memory.py", line 121, in add_episode
    resp = self._retry(_call)
  File "/home/isacc/deployment_ws/src/feeding-deployment/src/feeding_deployment/preference_learning/methods/utils.py", line 27, in _retry_on_rate_limit
    return fn()
  File "/home/isacc/deployment_ws/src/feeding-deployment/src/feeding_deployment/preference_learning/methods/long_term_memory.py", line 109, in _call
    return self.client.messages.create(
  File "/home/isacc/miniconda3/envs/feed/lib/python3.10/site-packages/anthropic/_utils/_utils.py", line 294, in wrapper
    return func(*args, **kwargs)
  File "/home/isacc/miniconda3/envs/feed/lib/python3.10/site-packages/anthropic/resources/messages/messages.py", line 1032, in create
    return self._post(
  File "/home/isacc/miniconda3/envs/feed/lib/python3.10/site-packages/anthropic/_base_client.py", line 1536, in post
    return cast(ResponseT, self.request(cast_to, opts, stream=stream, stream_cls=stream_cls))
  File "/home/isacc/miniconda3/envs/feed/lib/python3.10/site-packages/anthropic/_base_client.py", line 1195, in request
    raise self._make_status_error_from_response(response) from None
anthropic.BadRequestError: Error code: 400 - {'type': 'error', 'error': {'type': 'invalid_request_error', 'message': 'This model does not support assistant message prefill. The conversation must end with a user message.'}, 'request_id': 'req_011CcUHj4Bb7FMqDPNrY6TKr'}
^C


^\Quit (core dumped)
