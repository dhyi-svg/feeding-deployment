# tools/ -- hardware helpers and work in progress (not installed, not part of the package)

Run with the session env: `source ../sheppy/env.sh && python3 <tool>.py`. Everything that can move
the arm is a DRY RUN unless given `--execute`, and only ever with someone at the e-stop.

| tool | what |
|---|---|
| `goto_start.py [--execute]` | back out 1 cm along the last measured panel normal, then a straight sim-gated line to the start pose saved in `~/press_logs/start_joints.json` |
| `curobo_to_front.py [--execute]` | cuRobo-planned, gated move to the "front" start (camera ~34 cm from the panel) |
| `nudge.py DIST [--execute]` | move the tool DIST m (<= 2 cm) along the last measured panel normal (+ = toward the panel) |
| `imp_hold.py [--execute]` | open an `ee_pose_impedance` session, hold the current pose ~0.9 s, close. **2026-10-12: the arm moved 1.9 cm in that session -- don't use impedance on this Jetson yet** |
| `mock_rig.py CAM_D X,Y,Z` | offline driver + camera + tf stand-in (ROS_DOMAIN_ID=77 only) for dry runs |
| `mock_stream_driver.py`, `run_imp_mock.py` | offline streaming-tier mock for the impedance press (domain 77) |
| `dome_pattern_shape_wip.py` | **work in progress**: colour-free dome detector (grey rings + depth + the 3+2 formation, 5 same-size spots) for the cream microwave. Correct and steady on live red frames; NOT reliable on the saved/recoloured frames yet -- tune on real cream frames before installing it |
| `eval_frames.py WIP_MODULE red\|cream` | run a detector module over `frames/` (red, or recoloured cream); `frames/base_red.json` = the colour detector's answers |
| `live_compare.py` | colour detector vs the WIP shape one on live frames, plus the same frames recoloured cream (read-only) |
