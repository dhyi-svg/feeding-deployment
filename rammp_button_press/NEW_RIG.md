# Button press on a new rig (different Jetson + arm) — checklist

Written 2026-10-12 for the first session on a new rig with a **cream** Comfee microwave. On the
original rig (this repo's Jetson, red microwave) everything below is already done.

**What moves over with `git clone`:** all the code, the sheppy manifest, the tools.
**What does NOT, and must be redone on the new rig:**

1. **Hand-eye calibration** — where the camera sits on the wrist. Every button position comes
   from it; ours is for our camera mount only.
2. **`PREPRESS_EE_OFFSET_M`** (`rammp_button_press/press_button.py`) — it absorbs the rig's
   calibration error, so it is wrong on a new rig until re-measured. (Moving it from rchi-cpu-5
   put the fingertip 7 cm into the panel; `check_fingertip` refuses that, but only that.)
3. **The detector** — the installed one finds the panel by its RED colour. On cream it finds
   nothing (it refuses; it will not press the wrong thing). The colour-free version is work in
   progress: `tools/dome_pattern_shape_wip.py`.

Safety, every session: someone at the e-stop for every `--execute`; never touch the arm while
the `arm` driver is up (`WRONG_SERVOING_MODE` crash) — switch to `camera-only` first; check the
gripper and arm state at the start (press_button's preflight does).

---

## A. Check what is on the new Jetson (~10 min, nothing moves)

```bash
ls ~/.local/bin/sheppy ~/rammp-deployments ~/ros2_ws/src 2>&1     # RAMMP's tooling there?
docker images | grep -E "kinova|curobo|realsense"                 # RAMMP's images pulled?
ping -c1 192.168.1.10                                             # the arm (IP may differ)
lsusb | grep -i -E "realsense|8086"                               # which camera: D435i or D405?
ls ~/.ros2/easy_handeye2/calibrations/ 2>&1                       # an existing hand-eye calibration?
nproc; cat /proc/cmdline | tr ' ' '\n' | grep -E "isolcpus|nohz"  # which core is isolated for the driver
```

Ask whoever runs that rig: **is there already a wrist-camera hand-eye calibration, which camera
is on the wrist, and is sheppy with RAMMP's images set up?**

Images our manifest uses (pull if missing; cuRobo is ~19.5 GB):

```bash
docker pull ghcr.io/rammp-org/kinova-gen3-ros2:1.1.0
docker pull ghcr.io/rammp-org/rammp-curobo:1.1.0
docker pull ghcr.io/rammp-org/rammp-suite-software/realsense:main
```

## B. Install our code (~30–60 min if ROS 2 Humble + Docker + sheppy are there)

```bash
git clone -b rammp-integration https://github.com/dhyi-svg/feeding-deployment.git ~/feeding-deployment

# venv that also sees /opt/ros (rclpy, cv_bridge). Versions that work on the original Jetson:
python3 -m venv --system-site-packages ~/feeding-deployment/.venv
~/feeding-deployment/.venv/bin/pip install "numpy<2" scipy opencv-python pybullet pyyaml
#   (there: numpy 1.26.4, scipy 1.14.0, opencv 4.11.0, pybullet 3.2.7, pyyaml 6.0.3)

# RAMMP's repos at the tags the 1.1.0 images use -- clone, never modify
git clone -b v1.1.0 https://github.com/rammp-org/rammp-interfaces-ros2.git ~/rammp-interfaces-ros2
git clone -b v1.1.0 https://github.com/rammp-org/RAMMP-CuRobo.git ~/RAMMP-CuRobo
git clone https://github.com/rammp-org/kinova-gen3-driver.git ~/kinova-gen3-driver   # only for its URDF (arm_tf)

mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
ln -s ~/rammp-interfaces-ros2 rammp-interfaces-ros2
ln -s ~/RAMMP-CuRobo/rammp_curobo_interfaces rammp_curobo_interfaces
ln -s ~/feeding-deployment/rammp_button_press_interfaces rammp_button_press_interfaces
ln -s ~/feeding-deployment/rammp_button_press rammp_button_press
cd ~/ros2_ws && source /opt/ros/humble/setup.bash
colcon build      # NOT --symlink-install (it would give the scripts /usr/bin/python3, no pybullet)
```

If any of those already exist on the rig (RAMMP's machine may have them), reuse them — just check
the tags (`git -C <repo> describe --tags` → v1.1.0).

**Check `rammp_button_press/sheppy/compose.yaml` before bringing anything up:** it assumes the arm
at `192.168.1.10` (`KINOVA_ARM_IP`) and the driver pinned to CPU core `11` (`KINOVA_RT_CORE`) —
both this Jetson's settings. Export the new rig's values, or edit the defaults.

If the wrist camera is a **D405** (no IMU, different profiles), the `wrist_camera` block in
`sheppy-manifest.yaml` (640x480x15, align_depth on, IMU off) may need adjusting; topics must stay
`/wrist_camera/color/image_raw`, `/wrist_camera/color/camera_info`,
`/wrist_camera/aligned_depth_to_color/image_raw` (or pass `--camera-ns`).

Every terminal from here on:

```bash
cd ~/feeding-deployment/rammp_button_press/sheppy && source env.sh
```

## C. First bring-up, read-only

```bash
sheppy up --manifest sheppy-manifest.yaml camera-only     # camera + cuRobo planner, arm free
sheppy status
ros2 topic hz /wrist_camera/aligned_depth_to_color/image_raw   # ~15 Hz
```

Then the arm (holds where it is — hands off it):

```bash
sheppy up --manifest sheppy-manifest.yaml arm-camera
ros2 topic echo --once --qos-durability transient_local --qos-reliability reliable /control_status   # estopped false
ros2 topic echo --once --qos-reliability best_effort /gripper_state                                   # closed ~1.0
```

`press_button`'s preflight also checks that `/ee_state` matches our arm model ("tool frame ...
0.0 mm"). If that fails on the new arm, stop: the URDF or the driver's frame differs.

## D. Hand-eye calibration (the big one)

- **The rig has one** (easy_handeye2 `.calib`, eye-in-hand, `end_effector_link →
  camera_color_optical_frame`): `export BUTTON_CALIB=/path/to/it.calib` before `sheppy up`
  (env.sh has the line commented; process nodes inherit sheppyd's environment, so if sheppyd is
  already running, `sheppy down` first), then check `ros2 run tf2_ros tf2_echo base_link
  camera_color_optical_frame` gives a sensible camera pose. Verify by touch: detect, hand-guide
  the fingertip onto a button, compare (that is what step F does anyway).
- **It has none:** it needs a hand-eye calibration (a calibration board, 10–20 arm poses,
  easy_handeye2 or OpenCV `calibrateHandEye` with the PARK method — TSAI was unstable on
  Pachirisu, see TESTING_LOG.md). Plan an hour or more. **Nothing can be pressed without it.**

## E. Cream detector (nothing moves; can be done without the calibration)

Record real frames — hold the camera still at each view (hand-guide under `camera-only`):

```bash
python3 ~/feeding-deployment/rammp_button_press/tools/record_frames.py cream_front   # ~30 cm, straight on
python3 ~/feeding-deployment/rammp_button_press/tools/record_frames.py cream_close   # ~25 cm
python3 ~/feeding-deployment/rammp_button_press/tools/record_frames.py cream_far     # ~50 cm
python3 ~/feeding-deployment/rammp_button_press/tools/record_frames.py cream_left    # from a side
python3 ~/feeding-deployment/rammp_button_press/tools/record_frames.py cream_high    # from above
```

Frames land in `tools/frames/rec/`. Then, with Claude: "tune the shape detector on the cream
frames" — tune `tools/dome_pattern_shape_wip.py` on them, check it live
(`tools/live_compare.py`, `view_detection`), and only then install it as
`rammp_button_press/dome_pattern.py` (and the `button-task` copy). Commit the frames.

Microwave placement: start_30s about **0.75–0.85 m** from the arm base (reach limit 0.91 m),
roughly facing the arm, tool height at the button above 0.12 m.

## F. Re-measure the pre-press offset (attended)

The same as on the original rig (`README.md`, "Re-measuring the pre-press spot"):

1. Camera ~25–35 cm from the panel, all 5 domes in view, `arm-camera` up.
2. `ros2 run rammp_button_press measure_prepress_offset --target start_30s` — detects, then waits.
3. `sheppy up ... camera-only`, hand-guide the LEFT fingertip to just in front of START (where
   a press should start), let go, `sheppy up ... arm-camera`, press Enter. It prints the offset.
4. Confirm it clicks: `python3 tools/nudge.py 0.01` (dry), then with `--execute` (e-stop manned).
5. Put the printed `PREPRESS_EE_OFFSET_M` / `PREPRESS_EE_QUAT_PANEL` in `press_button.py`.
   **Important:** the press plans from the CLOSE re-detect at the staging pose, so express the
   offset in that frame — on the original rig, the first value (from the far view) put the run's
   spot 1.4 cm low. Easiest: after step 3, run a dry `press_button` from the same start and
   recompute the offset against its staging re-detect (ask Claude; it did exactly this on 10-12).
6. Rebuild: `cd ~/ros2_ws && colcon build --packages-select rammp_button_press`.

## G. Runs (someone at the e-stop)

```bash
ros2 run rammp_button_press press_button --target start_30s                                   # dry run
ros2 run rammp_button_press press_button --target start_30s --execute --presses 0 --no-return # stop in front of START
ros2 run rammp_button_press press_button --target start_30s --execute                         # press
sheppy down                                                                                    # end of session
```

Watch: the far detection, `to staging ... cuRobo`, the re-detect, `tool xyz goal`, the press.
Paste the output to Claude if anything stops — every refusal says why.

## If time is short

A–C and E need neither the calibration nor any motion, so recording cream frames is possible
either way. Pressing needs D and F done.
