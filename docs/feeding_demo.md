# Run the Feeding Demo on the Real Robot

> The entire bring-up below is scripted in [`launch_demo.sh`](../launch_demo.sh), which
> opens a tmux session and fires each step in order. Run the steps manually the first
> time so you understand them; use the script once it's familiar.

Most steps have a shell **alias** (defined on the relevant machine). The manual
commands are shown as a fallback.

## On the NUC (robot)

SSH in first: `sshnuc` (lab password). The NUC owns the arm, the base Arduino, and the
e-stop.

### 1. Arm controller server

- Alias: `launch_arm`
- Manual:
  ```bash
  conda activate controller
  cd feeding-deployment/src/feeding_deployment/robot_controller
  python arm_server.py
  ```
- **Inside-mouth bite transfer only** — first zero the arm torque offsets:
  - Alias: `set_zeros`, or manually:
    ```bash
    conda activate controller
    cd ~/feeding-deployment/src/feeding_deployment/robot_controller
    python kinova.py
    ```

### 2. Base controller server

The base Arduino is plugged into the **NUC** (not the compute box), so the Bulldog
e-stop also stops the base. The compute box drives the base over RPC via the cmd_vel
bridge / teleop.

- Alias: `launch_base`
- Manual:
  ```bash
  conda activate controller
  cd feeding-deployment/src/feeding_deployment/control/base_controller
  python base_server.py
  ```

### 3. Bulldog

- Alias: `launch_bulldog`
- **Note:** bulldog now **requires both** `arm_server.py` and `base_server.py` — it
  refuses to start if either RPC server is down.

## On the compute box (laptop)

### 4. roscore

```bash
roscore
```

### 5. Sensors / visualization

- Alias: `launch_sensors`
- Manual (from the workspace root):
  ```bash
  conda activate feed
  source devel/setup.bash
  roslaunch feeding_deployment sensors.launch
  ```

### 6. Robot watchdog

- Alias: `launch_watchdog`
- Manual:
  ```bash
  conda activate feed
  source devel/setup.bash
  cd ~/deployment_ws/src/feeding-deployment/src/feeding_deployment/integration
  chmod +x launch_robot.sh
  ./launch_robot.sh
  ```

### 7. (Recommended) Compute health monitor

A **system** watchdog (distinct from the ROS robot watchdog) that guards against the
RAM-exhaustion freeze. See [troubleshooting.md](troubleshooting.md#compute-health-monitor)
for details and flags. Run it in its own terminal:

```bash
cd ~/deployment_ws/src/feeding-deployment/src/feeding_deployment/integration
python compute_health_monitor.py
```

### 8. Feeding utensil driver

- Alias: `launch_utensil`
- Manual:
  ```bash
  conda activate feed
  source devel/setup.bash
  rosrun wrist_driver_ros wrist_driver
  ```
- **Important:** to shut this node down, press **Ctrl + /** (clean shutdown is wired to
  that signal).

### 9. Web application

- Make sure the feeding laptop's **WiFi is off** so the webapp binds to the router IP.
- Alias: `launch_app`
- Manual:
  ```bash
  conda activate feed
  source devel/setup.bash
  cd ~/deployment_ws/src/feedingpage/vue-ros-demo
  npm run serve
  ```
- Open on a browser connected to `FeedingDeployment-5G` (laptop or iPad):
  `http://192.168.1.2:8080/#/task_selection`

## On the cluster

### 10. molmo VLM server

- If off the Cornell network, enable the CISCO VPN first.
- SSH: `sshcluster` (or `ssh rj277@unicorn-login-01.coecis.cornell.edu`)
- Launch: `launch_molmo`

## Run the demo

- Make sure the laptop's **WiFi is on** and internet-connected (so the ChatGPT API works).
- Alias: `run_demo`
- Manual (from the workspace root):
  ```bash
  conda activate feed
  source devel/setup.bash
  cd src/feeding-deployment/src/feeding_deployment/integration
  python run.py --user feeding_deployment --run_on_robot --use_interface --no_waits
  ```
- **Resume from a state** (`after_utensil_pickup`, `after_bite_pickup`, `last_state`):
  ```bash
  python run.py --user tests --run_on_robot --use_interface --no_waits \
      --resume_from_state after_utensil_pickup
  ```

---

## Move the robot to preset configurations

- Alias: `cd_actions`
- Then: `python retract.py` (also `transfer.py`, `acquisition.py`)

## Calibrate tool offset for inside-mouth transfer

1. Grasp the tool and move to the pre-transfer position.
2. Calibrate:
   - Alias: `cd_demo`, or:
     ```bash
     conda activate feed
     source devel/setup.bash
     cd src/feeding-deployment/src/feeding_deployment/integration
     ```
   - `python transfer_calibration.py --tool <tool_name>` — `<tool_name>` ∈ {`fork`, `drink`, `wipe`}
3. Manually move the robot (using the buttons on the robot) to the intended inside-mouth
   transfer config, then press **ENTER** in the script to record it.
4. Test the calibration:
   ```bash
   python transfer_calibration.py --tool <tool_name> --test
   ```
