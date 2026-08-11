# Microwave door opening over ROS 2 — live run checklist

Pre-created **before** the run. Check rungs off live; do not reconstruct after.

Every rung is either **READ-ONLY** (safe) or **MOTION** (hand on the physical
e-stop). Do not skip ahead: each rung exists because a previous session lost time
to exactly the thing it checks.

Companions: `JETSON_SETUP.md` (the ordered bring-up commands this checklist
assumes), `LOCAL_DEPLOYMENT.md` (env/what-works), `TESTING_LOG.md` (history),
`CLAUDE.md` (current state).

---

## Rung 0 — Hardware present  ☐  READ-ONLY

The two most common blockers, both invisible from software until you look:

```bash
# Arm ethernet link must have carrier (the arm is 192.168.1.10)
cat /sys/class/net/enP8p1s0/carrier        # want: 1
ping -c1 192.168.1.10                      # want: reply

# RealSense must enumerate
lsusb | grep -i intel                      # want: an Intel device
```

**As of 2026-07-29 both were DOWN** — `enP8p1s0` had `carrier=0` (no ethernet to
the arm) and no Intel device was on USB. Nothing below can run until both pass.

Also confirm no other project holds the Kortex session (only one process can —
see the bottle-picking stack): `ps aux | grep -E "[h]ome_launch|[a]rm_driver"`
(the brackets stop grep from matching its own command line and looking like a hit).

---

## Rung 1 — Arm servers, fresh  ☐  READ-ONLY

A long-lived `arm_server.py` can wedge: on 2026-07-29 one had been spinning at
**99.7% CPU since Jul 14**. Restart rather than reuse.

```bash
pkill -f arm_server.py ; pkill -f bulldog_bypass.py ; pkill -f stub_base_server.py
rm -f /tmp/kinova.lock          # only if no live process holds the arm

E="PYTHONPATH=$HOME/.local/lib/python3.10/site-packages ARM_RPC_HOST=127.0.0.1"
PY=$HOME/feeding-deployment/.venv/bin/python

$PY src/feeding_deployment/control/robot_controller/arm_server.py   # 1st
$PY scripts/stub_base_server.py                                     # 2nd
$PY scripts/bulldog_bypass.py                                       # 3rd
```

Order matters. **The bypass has no software e-stop — physical only.** If the
bypass dies the arm e-stops in ~1 s (the one retained safety property).

Verify read-only: `get_state()` returns, mode `SERVOING`.

> **Is the arm still gripping the door?** CLAUDE.md records it may have been left
> holding the door mid-arc (gripper ~0.99). Check `gripper_pos` before anything
> moves, and clear it by hand if so.

---

## Rung 2 — ROS 2 TF chain  ☐  READ-ONLY

```bash
ros2 launch launch/ros2/microwave_bringup.launch.py
```

Run `ros2 launch` with **system python3** (it needs `lark`, absent from the
venv); the launch starts repo modules with the venv itself.

**This chain has never been brought up on hardware. This rung is the whole point
of the ROS 2 port — do not proceed past a failure here.**

```bash
ros2 topic hz /joint_states                                    # ~50 Hz
ros2 run tf2_ros tf2_echo arm_base_link camera_color_optical_frame
ros2 topic hz /camera/aligned_depth_to_color/image_raw          # depth alive
```

The tf2_echo translation should look like a wrist-mounted camera (~5 cm scale).
Wildly larger means the calibration or the joint bridge is wrong — stop.

---

## Rung 3 — Detection only, no motion  ☐  READ-ONLY

```bash
CUDA_VISIBLE_DEVICES="" PYTHONPATH=src:$PYTHONPATH \
  $PY -m feeding_deployment.perception.appliance_perception.appliance_perception \
      --handle_type "microwave handle"
```

GroundingDINO runs on **CPU** here (~34 s/frame; the Tegra iGPU hits an NVML
assert). Expect the handle at ~0.55–0.68 confidence.

**Compare the reported handle pose against teleop ground truth**
`[0.713, -0.099, 0.465]`. The known bias is **+16 cm in depth**, ~5 mm lateral,
~4 cm height.

Two things are new here and must be checked, not assumed:

1. **The transform path changed.** The proven numbers came from the calib chained
   with live FK. This now goes through **tf2**, using the same calibration — so
   the bias *should* match, but measure it.
2. **The hinge is now perceived, not assumed.** The standalone script hardcoded
   the hinge one door-width (`+0.32 m`) to +y. The repo instead fits the door
   plane and takes its far edge. Compare the two. **If they disagree
   substantially, trust the tuned `+0.32 m` — it is what actually opened the
   door.**

---

## Rung 4 — Enable the empirical corrections  ☐  READ-ONLY

Corrections are **off by default**. Turn them on only once rung 3 confirms the
bias is still what it was:

```bash
export HANDLE_DEPTH_CORR=0.094    # measured 2026-07-29 on the tf2 path
export HANDLE_LAT_CORR=0.0        # lateral residual is <1cm on this path
```

> **Not 0.16.** That value belongs to the old calib+FK path and undershoots by
> ~6 cm here. 0.094 was measured against a fresh touch ground truth at two
> distances (offset moved 5 mm over 13.7 cm, so it is a fixed offset, not a
> scale error). Re-measure after any recalibration, and re-touch the handle
> first -- the microwave moved 4.7 cm between 07-14 and 07-29.

Re-run rung 3 and confirm the corrected handle lands within ~1–2 cm of truth.

> Uncorrected, the grasp overshoots and **pushes the microwave**, and reachability
> checks spuriously abort (0.98 m read where 0.86 m was real). Do not run rung 5
> until this is verified.

---

## Rung 5 — Full open  ☐  **MOTION — hand on the e-stop**

Only after rungs 0–4 pass.

```bash
$PY scripts/validate_open_microwave_hla_sim.py --log_dir <dir>   # sim first, if IK allows
```

Sim motion validation is currently **unavailable on this box** — `plan_to_ee_pose`
fails even for a trivial 2 cm move (the IKFast wall). So the first real motion is
unrehearsed: go slow.

- `set_speed("low")`. Hand on the physical e-stop the entire time.
- Sweep **forward / away from the base** — a previous session e-stopped when the
  arc swept inward.
- Keep z raised if the other project's camera rig is under the wrist.
- Expect to pause at the grip check before the swing.

**Known gap:** joint-space staging configs (`left_back_retract_pos`,
`fridge_door_staging_pos`, `left_retract_pos`) are **lab values** and are
mounting-specific. Verify each is sane on this rig before trusting it.

---

## After the run

Record in `TESTING_LOG.md` while it is fresh: what each rung did, the measured
handle pose vs ground truth, perceived hinge vs the `+0.32 m` assumption, and
anything that aborted.
