"""Simulate the microwave open + close end to end in PyBullet and render it (sim only -- no robot, no ROS).

    python3 -u microwave/tools/viz_close_plans.py             # from the repo root, sheppy env sourced
    python3 -u microwave/tools/viz_close_plans.py --only P3   # one placement

1. A microwave MODEL is fitted from every saved detection (~/microwave_detections/*_result.json): hinge edge ->
   handle, handle offset off the face, handle height, face width/height, facing. Body depth (0.40) and the door
   past the handle (DOOR_PAST_HANDLE) are the code's own assumptions; the handle bar length is assumed.
2. The model is placed at PLACEMENTS (handle distance from the arm base x door height). For each placement the
   detector's output is synthesised from the model, and the REAL open runs (`open_task` + `push_open`) on a
   simulated arm (`SimArm`: executes motions the way SheppyArm does, via its own `cartesian_knots`), from a
   view pose relative to the microwave (36 cm in front of the handle, closer if needed to keep J4 within 135).
3. Where the open finished, every close variant then runs (execute=True, on the simulated arm), continuing from
   the open: same microwave, the door where the push-open left it, the door file the open wrote; the hand where
   the open left it, or moved 4 cm (back / up / right, as when the container is gripped by teleop).
   When a planner refuses, an in-memory instrumented copy of it (same source, every `return None` also records
   the planner's locals) gives what it had planned up to there and the pose it refused -- the planners
   themselves are not changed.

Output: microwave/close_viz/ (git-excluded) -- index.html, README.md, model/, and per placement
`P<n>_.../0_open/` + `<start>/<variant>/` with trajectory.mp4, keyframes.png, end.png, log.txt, result.json.

The door in the renders moves with the hand while the handle is held, else is pushed by whatever touches it
(a rough sweep, not physics); there is no force sim, so the close's torque-stopped push runs to its planned max.
"""
import argparse, contextlib, inspect, io, json, multiprocessing as mp, re, shutil, sys, textwrap, time, types
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
OUT = REPO / "microwave" / "close_viz"
DET = Path.home() / "microwave_detections"
HANDLE_LEN = 0.18                  # not in the detections: assumed
BODY_DEPTH = 0.40                  # = microwave_common.BODY_DEPTH (the code's assumption)
CONTAINER_LEN, CONTAINER_W, CONTAINER_H = 0.18, 0.11, 0.07   # tupperware in the fingers (approximate)
CONTAINER_ARGS = ["--route-over", "--keep-orientation", "--door-deg", "90", "--hold-offset-x", "-0.152",
                  "--push-radius", "0.17", "--push-radius-end", "0.15", "--over-above", "0.18", "--push-z", "0.22",
                  "--push-contact-nm", "6"]
HANDLE_XS = (0.65, 0.72, 0.79, 0.86)          # handle distance in front of the arm base (m)
BOTTOM_ZS = (0.05, 0.15)                       # door bottom above the arm base (10-21/22 ~0.05, 10-22 10:44 ~0.15)
VIEW_REL = (-0.36, 0.08, 0.10)                 # view-pose hand - handle on 10-21/22 (all detected from there)
VIEW_QUAT = (0.5607, 0.5611, 0.4299, 0.4313)   # camera forward, tilted 15 deg down
VIEW_SEED_DEG = (43.5, 21.1, -167.5, -141.2, 60.2, 69.0, 71.5)
VIEW_J4_MAX_DEG = 135.0                        # keep the view pose's J4 within this (hand closer to the microwave if not)
VIEW_MIN_X = 0.22                              # ... but the hand no closer than this in front of the handle
VIEW_W, VIEW_H, CAPTION_H = 360, 270, 66       # 336 px tall: a multiple of 8 for the encoder
MAX_FRAMES = 70
FPS = 8

# Every close continues from the simulated open at the same placement (same microwave, the door where the push-open
# left it, the door file the open wrote). Starts: the hand where the open left it, and moved a little, as when the
# container is gripped by teleop afterwards (10-21) -- within the old close's 6 cm start tolerance.
STARTS = {
    "1": ("1_hand-where-the-open-left-it", (0.0, 0.0, 0.0), "the arm exactly where the simulated open + push-open ended"),
    "2": ("2_hand-4cm-back", (-0.04, 0.0, 0.0), "then the hand moved 4 cm back (toward the arm), same orientation"),
    "3": ("3_hand-4cm-up", (0.0, 0.0, 0.04), "then the hand moved 4 cm up, same orientation"),
    "4": ("4_hand-4cm-right", (0.0, -0.04, 0.0), "then the hand moved 4 cm right (away from the hinge), same orientation"),
}
VARIANTS = {
    "old_wrist-left": ("old", None, False, "door_task.py close (default)"),
    "old_wrist-left_shifted-10cm": ("old", None, True, "door_task.py close --push-offset"),
    "old_wrist-right": ("old", "right", False, "door_task.py close --close-wrist-side right"),
    "old_wrist-right_shifted-10cm": ("old", "right", True, "door_task.py close --close-wrist-side right --push-offset"),
    "new_push-close_container": ("new", None, False,
                                 "real_gen3_ros2_close_microwave.py --phase push-close " + " ".join(CONTAINER_ARGS)),
}

G = types.SimpleNamespace()


# ============================================================================ setup (per process)
def _stub_detector():
    m = types.ModuleType("handle_detect")      # the planners don't detect; keep YOLO/torch out
    m.HandleDetector = None
    sys.modules["handle_detect"] = m


def _init():
    import os
    sys.path.insert(0, str(REPO / "microwave"))
    os.chdir(REPO)
    _stub_detector()
    import pybullet as p
    import microwave_common as mc, demo_replay, door_push, door_task as dt, sheppy_arm
    import real_gen3_ros2_grasp_and_swing_microwave as mw
    G.p, G.mc, G.dr, G.dp, G.dt, G.sa, G.mw = p, mc, demo_replay, door_push, dt, sheppy_arm, mw
    G.rec = []

    def fail(loc, fn):
        G.rec.append({"kind": "fail", "fn": fn, "locals": dict(loc)})
        return None

    def instrument(mod, name, users):
        fn = getattr(mod, name)
        src = re.sub(r"\breturn None\b", f"return __viz_fail(locals(), {name!r})", textwrap.dedent(inspect.getsource(fn)))
        mod.__dict__["__viz_fail"] = fail
        exec(compile(src, inspect.getsourcefile(fn), "exec"), mod.__dict__)
        for u in users:
            if getattr(u, name, None) is fn:
                setattr(u, name, mod.__dict__[name])

    def record_ok(mod, name, users, label_arg=None):
        inner = getattr(mod, name)

        def w(*a, **k):
            r = inner(*a, **k)
            ok = r is not None and not (isinstance(r, tuple) and r[0] is None)
            G.rec.append({"kind": "ok" if ok else "none", "fn": name, "result": r,
                          "label": a[label_arg] if label_arg is not None and len(a) > label_arg else None})
            return r
        for u in [mod] + users:
            if getattr(u, name, None) is inner:
                setattr(u, name, w)

    users = [demo_replay, door_push, dt, mw]
    instrument(mc, "plan_cartesian", users)
    instrument(mc, "plan_side_path", users)
    instrument(demo_replay, "plan", [dt])
    instrument(door_push, "_chain", [])
    instrument(door_push, "_plan_close", [])
    record_ok(mc, "plan_cartesian", users, label_arg=7)
    record_ok(demo_replay, "plan", [dt])
    record_ok(door_push, "_plan_close", [])

    # the open's swing check: every waypoint's IK result, per call (the last one is the refused step)
    src = textwrap.dedent(inspect.getsource(mw._check_arc))
    src = src.replace("        nq = np.array(sol[:7])\n", "        nq = np.array(sol[:7])\n        __viz_arc_step(nq)\n")
    mw.__dict__["__viz_arc_step"] = lambda nq: G.rec[-1]["qs"].append(np.asarray(nq, float))
    exec(compile(src, inspect.getsourcefile(mw._check_arc), "exec"), mw.__dict__)
    arc_inner = mw._check_arc

    def arc(scene, rb, wps, q0):
        G.rec.append({"kind": "arc", "fn": "_check_arc", "qs": [], "q0": np.asarray(q0, float)})
        r = arc_inner(scene, rb, wps, q0)
        G.rec[-1].update(ok=bool(r[0]), msg=r[2])
        return r
    mw._check_arc = arc

    # push-close: the swing-start search calls door_push.solve_ik / clearance (nothing else in door_push before it)
    ik_inner, cl_inner = door_push.solve_ik, door_push.clearance

    def ik_rec(scene, rb, pos, quat, seed):
        q, err = ik_inner(scene, rb, pos, quat, seed)
        G.rec.append({"kind": "dp_ik", "fn": "solve_ik", "pos": np.asarray(pos, float), "q": q, "err": err})
        return q, err

    def cl_rec(rb, q, bodies, closed=False):
        c = cl_inner(rb, q, bodies, closed)
        G.rec.append({"kind": "dp_clear", "fn": "clearance", "c": c})
        return c
    door_push.solve_ik, door_push.clearance = ik_rec, cl_rec
    G.scene, G.rb = mc.make_sim()               # render + FK + SimArm IK sim: the arm alone on the floor
    c = G.rb.physics_client_id
    for i in range(p.getNumBodies(physicsClientId=c)):
        bid = p.getBodyUniqueId(i, physicsClientId=c)
        if bid not in (G.rb.robot_id, 0):
            p.resetBasePositionAndOrientation(bid, [0, 0, -50], [0, 0, 0, 1], physicsClientId=c)
    G.base = np.asarray(G.scene.robot_base_pose.position)


def set_door_file(path):
    for m in (G.mc, G.dr, G.dp, G.dt, G.mw):
        if hasattr(m, "DOOR_FILE"):
            m.DOOR_FILE = Path(path)


def set_q(q, fingers_closed=False):
    p, rb, mc = G.p, G.rb, G.mc
    for j, jj in enumerate(mc.ARM):
        p.resetJointState(rb.robot_id, jj, float(q[j]), physicsClientId=rb.physics_client_id)
    for jj in mc.FINGER_JOINTS:
        p.resetJointState(rb.robot_id, jj, mc.FINGER_CLOSE_SIGN[jj] * mc.FINGER_CLOSED_RAD if fingers_closed else 0.0,
                          physicsClientId=rb.physics_client_id)


def fk(q):
    set_q(q)
    pe = G.rb.get_end_effector_pose()
    return np.r_[np.asarray(pe.position) - G.base, pe.orientation]


def ik(pos, quat, seed):
    q, err = G.mc.solve_ik(G.scene, G.rb, pos, quat, seed)
    for _ in range(5):
        q, err = G.mc.solve_ik(G.scene, G.rb, pos, quat, q)
    return G.mc.wrap_joints(q), err


# ============================================================================ microwave model
def fit_model():
    """Medians over every saved detection, plus the code's own assumptions."""
    rows = []
    for f in sorted(DET.glob("*_result.json")):
        d = json.loads(f.read_text())
        if not all(d.get(k) is not None for k in ("handle", "hinge", "normal", "door_z", "door_mid")):
            continue
        h, g, n, z, m = (np.asarray(d[k], float) for k in ("handle", "hinge", "normal", "door_z", "door_mid"))
        n = n / np.linalg.norm(n[:2])
        t = np.array([n[1], -n[0], 0.0])                      # along the face, toward the hinge side
        if (g - h)[:2] @ t[:2] < 0:
            t = -t
        rows.append({"r": float((g - h)[:2] @ t[:2]), "proud": float((h - m)[:2] @ n[:2]),
                     "face_w": 2 * float((g - m)[:2] @ t[:2]), "handle_dz": float(h[2] - z[0]),
                     "face_h": float(z[1] - z[0]), "yaw_deg": float(np.degrees(np.arctan2(-n[1], -n[0]))),
                     "handle_y": float(h[1])})
    med = {k: float(np.median([r[k] for r in rows])) for k in rows[0]}
    med.update(n_detections=len(rows), handle_len_assumed=HANDLE_LEN, body_depth_assumed=BODY_DEPTH,
               door_past_handle_code=G.mc.DOOR_PAST_HANDLE, door_t_code=G.mc.DOOR_T)
    return med


def placement_geometry(model, handle_x, bottom_z):
    """Model placed with its handle `handle_x` in front of the arm base, door bottom at `bottom_z`."""
    yaw = np.radians(model["yaw_deg"])
    n = -np.array([np.cos(yaw), np.sin(yaw), 0.0])          # face normal, toward the arm
    u = np.array([n[1], -n[0], 0.0])                         # along the face: hinge -> handle
    if u[1] > 0:                                             # hinge on the +y (left) side, as on the rig
        u = -u
    handle = np.array([handle_x, model["handle_y"], bottom_z + model["handle_dz"]])
    face_at_handle = handle - n * model["proud"]
    hinge = face_at_handle - u * model["r"]
    door_mid = hinge + u * model["face_w"] / 2
    door_mid[2] = bottom_z + model["face_h"] / 2
    sign = float(np.sign(u[0] * n[1] - u[1] * n[0]))       # opening sense (door_push.DoorFrame)
    return {"n": n, "u": u, "handle": handle, "hinge": hinge, "door_mid": door_mid,
            "door_z": np.array([bottom_z, bottom_z + model["face_h"]]), "open_sign": sign, "model": model}


def synth_detection(geo):
    """What handle_detect + detect_handle return for this placement (two agreeing looks)."""
    return (geo["handle"].copy(), np.array([-0.5, 0.5, 0.5, -0.5]), geo["n"].copy(), geo["door_z"].copy(),
            geo["door_mid"].copy(), geo["hinge"].copy())


# ============================================================================ simulated arm
class SimArm:
    """Stands in for SheppyArm: same calls, motion executed in sim the way SheppyArm turns it into joints."""

    def __init__(self, q, gripper=0.0, phase=""):
        self.q = G.mc.wrap_joints(q)
        self.ee, self.g, self.phase = fk(self.q), float(gripper), phase
        self.moves, self.events, self.last_path, self.monitor_stopped = [], [], [], False

    def get_state(self):
        return {"position": self.q.copy(), "velocity": np.zeros(7), "effort": np.zeros(7),
                "ee_pos": self.ee.copy(), "gripper_pos": self.g}

    def _label(self):
        for fr in inspect.stack(0)[2:14]:
            fn, loc = fr.function, fr.frame.f_locals
            if fn == "execute_joint_plan":
                return str(loc.get("label"))
            if fn == "_push_continuous":
                return "push straight in (no force sim: runs to its planned max)"
            if fn == "_run_sweep":
                return "swing the door shut"
            if fn == "retract":
                return "retract"
            if fn == "open_task":
                return "grasp approach" if self.g < 0.5 else "swing the door open (holding the handle)"
            if fn == "run" and "enc_path" in loc:
                return f"replay {Path(str(loc['enc_path'])).name.split('.')[0]}"
        return "move"

    def execute_command(self, cmd):
        from feeding_deployment.control.robot_controller.command_interface import (
            CartesianTrajectoryCommand, CloseGripperCommand, JointCommand, OpenGripperCommand)
        if isinstance(cmd, JointCommand):
            return self.move_joints([np.asarray(cmd.pos, float)])
        if isinstance(cmd, CartesianTrajectoryCommand):
            return self.move_cartesian(cmd.traj)
        if isinstance(cmd, OpenGripperCommand):
            return self.gripper(0.0)
        if isinstance(cmd, CloseGripperCommand):
            return self.gripper(1.0)
        raise TypeError(type(cmd).__name__)

    def move_joints(self, knots, cart_lengths=None, speed_scale=1.0, impedance=None, monitor=None):
        path = G.sa._unwrap_from(self.q, knots)
        self.monitor_stopped = False
        done = []
        for q in path:
            done.append(np.asarray(q, float))
            self.q = G.mc.wrap_joints(q)
            self.ee = fk(self.q)
            self.last_path.append((len(self.last_path) * 0.02, *self.ee[:3]))
            if monitor is not None and monitor():
                self.monitor_stopped = True
                break
        self.moves.append({"phase": self.phase, "label": self._label(), "qs": done, "grip": self.g})
        return True

    def move_cartesian(self, traj, impedance=None):
        knots, _ = G.sa.cartesian_knots((G.scene, G.rb), self.ee, self.q, traj)
        if knots is None:
            self.events.append("a Cartesian move was refused by SheppyArm's own IK (cartesian_knots)")
            print("  SimArm: cartesian_knots refused this move (IK / jump / J4 / J6 on the way)")
            return False
        return self.move_joints(knots)

    def gripper(self, position, timeout_s=4.0, force=None):
        self.g = 0.8 if position > 0.5 else 0.0          # closing on the handle/container reads ~0.8 (not empty)
        self.moves.append({"phase": self.phase, "label": "gripper " + ("closes" if position > 0.5 else "opens"),
                           "qs": [self.q.copy()], "grip": self.g})
        return True

    def close(self):
        pass


# ============================================================================ planner bookkeeping (failures)
def capture(fn):
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        try:
            r = fn()
        except SystemExit as e:
            print(f"REFUSED (sys.exit): {e}")
            r = False
    return r, "\n".join(l for l in buf.getvalue().splitlines() if "running IKFast with a custom URDF" not in l)


def recs(kind=None, fn=None):
    return [r for r in G.rec if (kind is None or r["kind"] == kind) and (fn is None or r["fn"] == fn)]


def why_line(log):
    lines = [l.strip() for l in log.splitlines() if l.strip()]
    generic = re.compile(r"no plan passes every gate|a leg fails a gate -- refusing")
    hits = [i for i, l in enumerate(lines)
            if re.search(r"fails a gate|refusing|REFUSED|out of reach|no joints with|refused", l) and not generic.search(l)]
    if not hits:
        return lines[-1] if lines else ""
    i = hits[-1]
    if i > 0 and re.search(r"step \d+ fails a gate|not at the demo's start", lines[i]):
        return f"{lines[i - 1]} | {lines[i]}"
    return lines[i]


def gates(L, pc=True):
    mc = G.mc
    nq = L.get("nq")
    if nq is None:
        return ""
    out = []
    err, jump = L.get("err", 0.0), L.get("jump", 0.0)
    if err > mc.MAX_IK_ERR:
        out.append(f"IK can't reach the target ({err * 100:.1f} cm off)")
    if jump > mc.MAX_STEP_JUMP_DEG:
        out.append(f"a joint jumps {jump:.0f} deg in one step")
    if abs(np.degrees(nq[3])) > mc.J4_GUARD_DEG:
        out.append(f"J4 {np.degrees(nq[3]):.2f} deg past its {mc.J4_GUARD_DEG:.0f} guard")
    if abs(np.degrees(nq[5])) > mc.J6_GUARD_DEG:
        out.append(f"J6 {np.degrees(nq[5]):.2f} deg past its {mc.J6_GUARD_DEG:.0f} guard")
    q = L.get("q") if pc else L.get("qq")
    if q is not None and not mc.continuous_ok(q, nq):
        out.append("a free joint would wrap past +-180")
    w = L.get("worst") if pc else L.get("c_body")
    lim = (L.get("min_clear") or mc.MIN_CLEAR) if pc else mc.MIN_CLEAR
    if w is not None and w[0] < lim:
        out.append(f"clearance {w[0] * 100:.1f} cm < {lim * 100:.0f} ({w[1]})")
    return "; ".join(out)


def stuck_from_pc():
    pc = recs("fail", "plan_cartesian")
    if not pc:
        return [], None
    L = pc[-1]["locals"]
    seg = [(f"PLANNED, not run: {L.get('label', 'leg')} (up to the refusal)", list(L.get("plan", [])))] if L.get("plan") else []
    return seg, {"q": L.get("nq"), "target": L.get("tgt"),
                 "reason": f"{L.get('label', '')}: step {L.get('k')}/{L.get('n')} -- {gates(L) or 'a gate'}"}


def side_partial(L):
    if "path" in L:
        return [list(l) for l in L["legs"]], L.get("q"), None, f"clearance {L['w'][0] * 100:.1f} cm between steps ({L['w'][1]})"
    layers, steps, k = L["layers"], L["steps"], L["k"]
    path, j = [], 0
    for li in range(len(layers) - 1, 0, -1):
        q, _, jp = layers[li][j]
        path.append(q)
        j = jp
    path.reverse()
    legs = [[] for _ in range(max(s[2] for s in steps) + 1)]
    for (pos, quat, leg), q in zip(steps, path):
        legs[leg].append(q)
    return [l for l in legs if l], None, steps[k], ""


def planned_partial(kind, side, labels, log):
    """After a refusal: (segments planned but not run, stuck dict) from the instrumented planners."""
    segs, stuck = [], None
    if kind == "old":
        if "not at the demo's start" in log or "out of reach by more than" in log:
            return [], {"q": None, "target": G.cur_kps[0][0] if "not at the demo" in log else G.cur_kps[-1][0],
                        "reason": why_line(log)}
        if not recs("fail", "plan") and not recs("fail", "plan_side_path"):
            s, st = stuck_from_pc()                          # retract
            return s, st or {"q": None, "target": None, "reason": why_line(log)}
        outer = recs("fail", "plan")
        for i, leg in enumerate(outer[-1]["locals"].get("legs", []) if outer else []):
            segs.append((f"PLANNED, not run: close leg {i + 1} ('{labels[i + 1][:24]}')", list(leg)))
        if side:
            legs, q_bad, tgt, why = side_partial(recs("fail", "plan_side_path")[-1]["locals"])
            for i, leg in enumerate(legs):
                segs.append((f"PLANNED, not run: close leg {i + 1} ('{labels[i + 1][:24]}')", leg))
            stuck = {"q": q_bad, "target": None, "reason": why or why_line(log)}
            if tgt is not None:
                last = legs[-1][-1] if legs else G.cur_q0
                gq, gerr = ik(tgt[0], tgt[1], last)
                off = G.mc.wrist_offset(G.scene, G.rb, gq)
                stuck.update(q=gq, target=tgt[0], ghost=True,
                             reason=stuck["reason"] + f" -- shown: nearest plain-IK pose (wrist {off * 100:+.0f} cm right "
                                                      f"of the elbow, J4 {np.degrees(gq[3]):.0f}, J6 {np.degrees(gq[5]):.0f})")
            return segs, stuck
        s, stuck = stuck_from_pc()
        return segs + s, stuck
    fails = recs("fail", "_plan_close")
    if not fails or "route" not in fails[-1]["locals"]:
        return [], {"q": None, "target": None, "reason": why_line(log)}
    L = fails[-1]["locals"]
    if L["route"] is None:
        ch = recs("fail", "_chain")
        for lab, leg in (ch[-1]["locals"].get("legs", []) if ch else []):
            segs.append((f"PLANNED, not run: {lab}", list(leg)))
        s, stuck = stuck_from_pc()
        return segs + s, stuck
    for lab, leg in L["route"]:
        segs.append((f"PLANNED, not run: {lab}", list(leg)))
    if L.get("plan"):
        segs.append(("PLANNED, not run: swing the door shut", list(L["plan"])))
    if "back" in L:
        segs.append(("PLANNED, not run: push", list(L.get("push", []))))
        s, stuck = stuck_from_pc()
        return segs, stuck
    if L.get("bad"):
        return segs, {"q": L.get("nq"), "target": L["at"](L["d_next"]) if L.get("at") else None,
                      "reason": f"swing step {L.get('k')} ({L.get('d_next', 0):.0f} deg round the hinge) -- "
                                f"{gates(L, pc=False) or why_line(log)}"}
    return segs, {"q": (L.get("plan") or [None])[-1], "target": None, "reason": why_line(log)}


# ============================================================================ rendering
class World:
    """The microwave model (body, control panel, door slab, handle bar) in the render sim, + container."""

    def __init__(self, geo, door_deg, container=False):
        p, c = G.p, G.rb.physics_client_id
        self.p, self.c, self.geo = p, c, geo
        m, n, u = geo["model"], geo["n"], geo["u"]
        self.n, self.u = n, u
        zb, ztop = geo["door_z"]
        H = ztop - zb
        L = m["r"] + G.mc.DOOR_PAST_HANDLE
        T = G.mc.DOOR_T
        yaw_n = float(np.arctan2(n[1], n[0]))
        hinge = geo["hinge"]
        self.ids = []
        zc = zb + H / 2
        body_c = hinge + u * m["face_w"] / 2 - n * (T + BODY_DEPTH / 2)
        self.ids.append(self._box([BODY_DEPTH / 2, m["face_w"] / 2, H / 2], [0.32, 0.32, 0.35, 1], [*body_c[:2], zc], yaw_n))
        if m["face_w"] > L:
            pc_ = hinge + u * (L + m["face_w"]) / 2 - n * T / 2
            self.ids.append(self._box([T / 2, (m["face_w"] - L) / 2, H / 2], [0.2, 0.2, 0.22, 1], [*pc_[:2], zc], yaw_n))
        self.door_local = (L / 2, -T / 2)            # door centre: along u, along n (from the hinge edge)
        self.door = self._box([T / 2, L / 2, H / 2], [0.55, 0.75, 0.95, 0.95], [0, 0, zc], yaw_n)
        self.handle = self._box([0.012, 0.012, HANDLE_LEN / 2], [0.85, 0.85, 0.85, 1], [0, 0, geo["handle"][2]], yaw_n,
                                collide=False)
        self.ids += [self.door, self.handle]
        self.zc, self.yaw_n, self.L = zc, yaw_n, L
        self.container = (self._box([CONTAINER_LEN / 2, CONTAINER_W / 2, CONTAINER_H / 2], [0.95, 0.55, 0.15, 1],
                                    [0, 0, -50], 0.0) if container else None)
        if container:
            self.ids.append(self.container)
        self.angle = door_deg
        self.set_door(door_deg)

    def _box(self, half, rgba, pos, yaw, collide=True):
        from scipy.spatial.transform import Rotation as R
        p, c = self.p, self.c
        vs = p.createVisualShape(p.GEOM_BOX, halfExtents=list(half), rgbaColor=rgba, physicsClientId=c)
        cs = p.createCollisionShape(p.GEOM_BOX, halfExtents=list(half), physicsClientId=c) if collide else -1
        return p.createMultiBody(0, cs, vs, basePosition=(G.base + np.asarray(pos)).tolist(),
                                 baseOrientation=R.from_euler("z", yaw).as_quat().tolist(), physicsClientId=c)

    def sphere(self, pos, r, rgba):
        p, c = self.p, self.c
        vs = p.createVisualShape(p.GEOM_SPHERE, radius=r, rgbaColor=rgba, physicsClientId=c)
        self.ids.append(p.createMultiBody(0, -1, vs, basePosition=(G.base + np.asarray(pos)).tolist(), physicsClientId=c))

    def _rot(self, deg):
        a = self.geo["open_sign"] * np.radians(deg)
        return a, np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])

    def set_door(self, deg):
        from scipy.spatial.transform import Rotation as R
        a, rot = self._rot(deg)
        h = self.geo["hinge"][:2]
        d_c = h + rot @ (self.u[:2] * self.door_local[0] + self.n[:2] * self.door_local[1])
        d_h = h + rot @ (self.u[:2] * self.geo["model"]["r"] + self.n[:2] * self.geo["model"]["proud"])
        q = R.from_euler("z", self.yaw_n + a).as_quat().tolist()
        self.p.resetBasePositionAndOrientation(self.door, (G.base + [d_c[0], d_c[1], self.zc]).tolist(), q,
                                               physicsClientId=self.c)
        self.p.resetBasePositionAndOrientation(self.handle, (G.base + [d_h[0], d_h[1], self.geo["handle"][2]]).tolist(),
                                               q, physicsClientId=self.c)
        self.angle = deg

    def hand_angle(self, tool):
        """Door angle that puts the handle where the held hand is."""
        d = tool[:2] - self.geo["hinge"][:2]
        g = self.u[:2]
        a = np.degrees(np.arctan2(g[0] * d[1] - g[1] * d[0], g @ d)) * self.geo["open_sign"]
        return float(np.clip(a, 0.0, 120.0))

    def place_container(self):
        from scipy.spatial.transform import Rotation as R
        if self.container is None:
            return
        ls = self.p.getLinkState(G.rb.robot_id, G.rb.end_effector_id, physicsClientId=self.c)
        pos, orn = np.asarray(ls[4]), np.asarray(ls[5])
        z = R.from_quat(orn).as_matrix()[:, 2]
        ctr = pos + z * (G.mc.FINGERTIP_PAST_TOOL - 0.03 + CONTAINER_LEN / 2)
        self.p.resetBasePositionAndOrientation(self.container, ctr.tolist(),
                                               (R.from_quat(orn) * R.from_euler("y", 90, degrees=True)).as_quat().tolist(),
                                               physicsClientId=self.c)

    def touching(self):
        p, c = self.p, self.c
        if p.getClosestPoints(G.rb.robot_id, self.door, 0.0, physicsClientId=c):
            return True
        return self.container is not None and bool(p.getClosestPoints(self.container, self.door, 0.0, physicsClientId=c))

    def step(self, q, holding_handle, fingers_closed, push=True):
        set_q(q, fingers_closed)
        self.place_container()
        if holding_handle:
            self.set_door(self.hand_angle(fk(q)[:3]))
            set_q(q, fingers_closed)
            return
        if not push:
            return
        self.set_door(self.angle)
        if not self.touching():
            return
        a0 = self.angle
        for k in range(1, 121):                     # nearest free angle, either way
            for a in (a0 - k, a0 + k):
                if 0.0 <= a <= 120.0:
                    self.set_door(a)
                    if not self.touching():
                        return
        self.set_door(a0)

    def tint(self, rgba):
        for j in range(-1, self.p.getNumJoints(G.rb.robot_id, physicsClientId=self.c)):
            self.p.changeVisualShape(G.rb.robot_id, j, rgbaColor=rgba, physicsClientId=self.c)

    def views(self):
        import cv2
        p, c, B = self.p, self.c, G.base
        mx = float(self.geo["hinge"][0]) - 0.25
        pm = p.computeProjectionMatrixFOV(54, VIEW_W / VIEW_H, 0.05, 6)
        out = []
        for name, eye, tgt, up in (("behind the arm", B + [-0.55, -0.9, 1.05], B + [mx, 0.0, 0.2], [0, 0, 1]),
                                   ("top-down (microwave at top)", B + [mx, 0.0, 1.8], B + [mx, 0.0, 0.0], [1, 0, 0]),
                                   ("hinge side", B + [mx, 1.35, 0.6], B + [mx, 0.0, 0.25], [0, 0, 1])):
            vm = p.computeViewMatrix(eye.tolist(), tgt.tolist(), up)
            _, _, img, _, _ = p.getCameraImage(VIEW_W, VIEW_H, vm, pm, renderer=p.ER_TINY_RENDERER, physicsClientId=c)
            img = np.ascontiguousarray(np.reshape(img, (VIEW_H, VIEW_W, 4))[:, :, :3]).astype(np.uint8)
            cv2.putText(img, name, (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (40, 40, 40), 1, cv2.LINE_AA)
            out.append(img)
        return np.concatenate(out, axis=1)

    def clear(self):
        for bid in self.ids:
            self.p.removeBody(bid, physicsClientId=self.c)
        self.tint([0.75, 0.75, 0.75, 1.0])


def caption(img, title, line, why=None):
    import cv2
    bar = np.full((CAPTION_H, img.shape[1], 3), 255, np.uint8)
    cv2.putText(bar, title, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 0, 0), 1, cv2.LINE_AA)
    cv2.putText(bar, line[:150], (8, 37), cv2.FONT_HERSHEY_SIMPLEX, 0.43, (60, 60, 60), 1, cv2.LINE_AA)
    if why:
        cv2.putText(bar, why[:165], (8, 57), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 0, 0), 1, cv2.LINE_AA)   # RGB red
    return np.concatenate([bar, img], axis=0)


def render(case_dir, geo, q0, door_deg0, moves, planned, stuck, status, title, container, grip0, markers=()):
    """moves: executed (SimArm) [{label, qs, grip, phase}]; planned: [(label, qs)] not run; stuck: dict or None."""
    import cv2, imageio
    w = World(geo, door_deg0, container)
    for mk in markers:
        w.sphere(mk, 0.012, [0.2, 0.35, 0.9, 1.0])
    seq = [(np.asarray(q0), "start", False, grip0 > 0.5)]
    for mv in moves:
        held = mv["phase"] == "open" and mv["grip"] > 0.5 and "gripper" not in mv["label"]
        for q in mv["qs"]:
            seq.append((np.asarray(q), mv["label"], held, mv["grip"] > 0.5))
    n_exec = len(seq)
    last_grip = seq[-1][3]
    for item in planned:
        lab, qs = item[0], item[1]
        held = item[2] if len(item) > 2 else False
        closed = item[3] if len(item) > 3 else last_grip
        seq += [(np.asarray(q), lab, held, closed) for q in qs]
    keep = set(range(len(seq))) if len(seq) <= MAX_FRAMES else set(np.linspace(0, len(seq) - 1, MAX_FRAMES).round().astype(int))
    keep |= {i for i in range(1, len(seq)) if seq[i][1] != seq[i - 1][1]} | {len(seq) - 1}
    frames = []
    for i, (q, lab, held, closed) in enumerate(seq):
        w.step(q, held, closed, push=i < n_exec and "grasp approach" not in lab and "approach to the grasp" not in lab)
        if i not in keep:
            continue
        w.sphere(fk(q)[:3], 0.006, [0.1, 0.7, 0.2, 1.0] if i < n_exec else [0.6, 0.6, 0.1, 1.0])
        set_q(q, closed)
        w.place_container()
        line = (f"{lab}   step {i}/{len(seq) - 1}   J4 {np.degrees(q[3]):.0f}  J6 {np.degrees(q[5]):.0f}   "
                f"door ~{w.angle:.0f} deg")
        frames.append(caption(w.views(), title, line))
    door_end = w.angle
    if stuck:
        if stuck.get("target") is not None:
            w.sphere(stuck["target"], 0.02, [0.9, 0.1, 0.1, 1.0])
        if stuck.get("q") is not None:
            w.tint([0.95, 0.35, 0.35, 1.0])
            set_q(stuck["q"], last_grip)
            w.place_container()
            tag = "nearest pose (breaks a rule)" if stuck.get("ghost") else "the refused pose (red)"
        else:
            tag = "refused (red ball: where the hand had to be)"
        img = caption(w.views(), title, f"{status}: {tag}", "WHY: " + stuck["reason"])
        frames += [img] * 10
    else:
        frames += [frames[-1]] * 6
    wr = imageio.get_writer(str(case_dir / "trajectory.mp4"), fps=FPS, codec="libx264", quality=7, macro_block_size=8)
    for f in frames:
        wr.append_data(f)
    wr.close()
    cv2.imwrite(str(case_dir / "end.png"), cv2.cvtColor(frames[-1], cv2.COLOR_RGB2BGR))
    picks = [frames[0], frames[len(frames) // 3], frames[2 * len(frames) // 3], frames[-1]]
    cv2.imwrite(str(case_dir / "keyframes.png"), cv2.cvtColor(np.concatenate(picks, axis=0), cv2.COLOR_RGB2BGR))
    w.clear()
    return door_end, len(seq)


# ============================================================================ tasks
def pname(i, hx, bz):
    return f"P{i}_handle-{hx * 100:.0f}cm-out_door-bottom-{bz * 100:.0f}cm-up"


def view_joints(geo):
    """View pose for this placement: hand VIEW_REL from the handle (where the 10-21/22 opens started), moved toward
    the microwave 1 cm at a time until J4 is within VIEW_J4_MAX_DEG (handoff: keep the view pose J4 ~-130..-135;
    a start at -146.6 made the open refuse every tilt), but no closer than VIEW_MIN_X to the handle.
    Returns (joints, IK err, hand offset from the handle)."""
    rel = np.asarray(VIEW_REL, float)
    while True:
        q, err = ik(geo["handle"] + rel, np.asarray(VIEW_QUAT), np.radians(VIEW_SEED_DEG))
        if abs(np.degrees(q[3])) <= VIEW_J4_MAX_DEG or -rel[0] <= VIEW_MIN_X + 1e-9:
            return q, err, rel
        rel[0] += 0.01


def open_partial(ai, log, door_file):
    """The open gave up. Before any motion (no tilt passes): the attempt that got furthest -- its approach and its
    swing up to the refused waypoint. Later (push-open): the push-open legs planned up to the refusal."""
    if any(m["phase"] == "push-open" for m in ai.moves) or ai.phase == "push-open":
        door = json.loads(Path(door_file).read_text())
        enc = json.loads(Path(G.mw.PUSH_OPEN).read_text())
        G.cur_kps = G.dr.decode(enc, door)
        return planned_partial("old", None, [k["label"] for k in enc["keypoints"]], log)
    arcs = [i for i, r in enumerate(G.rec) if r["kind"] == "arc"]
    if not arcs:
        segs, stuck = stuck_from_pc()
        return segs, stuck or {"q": None, "target": None, "reason": why_line(log)}
    best = max(arcs, key=lambda i: len(G.rec[i]["qs"]))
    a = G.rec[best]
    lineup = next((G.rec[i] for i in range(best - 1, -1, -1) if G.rec[i]["kind"] == "ok"
                   and str(G.rec[i].get("label", "")).startswith("to line-up")), None)
    straight = next((G.rec[i] for i in range(best - 1, -1, -1) if G.rec[i]["kind"] == "ok"
                     and G.rec[i].get("label") == "straight in"), None)
    segs = []
    if lineup is not None:
        segs.append((f"PLANNED, not run: approach to the grasp ({lineup['label']})", list(lineup["result"]), False, False))
    if straight is not None:
        segs.append(("PLANNED, not run: approach to the grasp (straight in)", list(straight["result"]), False, False))
    segs.append((f"PLANNED, not run: swing open, the furthest attempt ({len(a['qs']) - 1} waypoints passed)",
                 a["qs"][:-1], True, True))
    reason = why_line(log)
    return segs, {"q": a["qs"][-1], "target": None,
                  "reason": f"best attempt: {a.get('msg', '')} (J4 {np.degrees(a['qs'][-1][3]):.1f}, guard "
                            f"{G.mc.J4_GUARD_DEG:.0f}) -- {reason[:90]}"}


def task_open(t):
    """The real open (grasp + swing + push-open) on SimArm, from the view pose."""
    pdir = OUT / t["pname"]
    geo = t["geo"]
    case_dir = pdir / "0_open"
    case_dir.mkdir(parents=True, exist_ok=True)
    door_file = pdir / "door_file_written_by_the_open.json"
    door_file.unlink(missing_ok=True)
    set_door_file(door_file)
    G.mw.detect_handle = lambda: synth_detection(geo)
    q0, err, rel = view_joints(geo)
    ai = SimArm(q0, 0.0, "open")

    def go():
        print(f"view pose: hand = handle + {np.round(rel, 2)} (J4 kept within {VIEW_J4_MAX_DEG:.0f}), IK err "
              f"{err * 100:.2f} cm, J {np.round(np.degrees(q0), 1)}")
        if not G.mw.open_task(ai, True, G.mw.PUSH_OPEN_SWING_DEG):
            return False
        ai.phase = "push-open"
        return G.mw.push_open(ai, True)
    t0 = time.time()
    ok, log = capture(go)
    status = "OK" if ok else "STUCK"
    planned, stuck = ([], None) if ok else open_partial(ai, log, door_file)
    door_end, _ = render(case_dir, geo, q0, 0.0, ai.moves, planned, stuck, status,
                         f"{t['pname']} | the open (simulated)", False, 0.0)
    (case_dir / "log.txt").write_text(log)
    res = {"placement": t["pname"], "case": "0_open", "status": status, "reason": (stuck or {}).get("reason", ""),
           "end_joints": ai.q.tolist(), "door_deg_end_render": door_end, "door_file_written": door_file.exists(),
           "phases": [{"label": m["label"], "steps": len(m["qs"])} for m in ai.moves], "seconds": round(time.time() - t0, 1)}
    (case_dir / "result.json").write_text(json.dumps(res, indent=1))
    return res


def task_close(t):
    pdir = OUT / t["pname"]
    geo, skey, vkey = t["geo"], t["start"], t["variant"]
    kind, side, shifted, cmd = VARIANTS[vkey]
    case_dir = pdir / STARTS[skey][0] / vkey
    case_dir.mkdir(parents=True, exist_ok=True)
    work_door = case_dir / "_door_file.json"           # each close gets its own copy (push-close rewrites the angle)
    work_door.write_text((pdir / "door_file_written_by_the_open.json").read_text())
    q_open, door0 = np.asarray(t["open_end_q"]), t["open_door_deg"]
    ee_open = fk(q_open)
    off = np.asarray(STARTS[skey][1])
    if np.any(off):
        q0, err = ik(ee_open[:3] + off, ee_open[3:7], q_open)
    else:
        q0, err = q_open, 0.0
    set_door_file(work_door)
    door = json.loads(work_door.read_text())
    ai = SimArm(q0, 0.98, "close")                    # holding a container
    G.cur_q0 = q0
    G.rec.clear()
    t0 = time.time()
    markers, labels = [], []
    if kind == "old":
        enc_path = G.dt.CLOSE_PUSH_OFFSET if shifted else G.dt.CLOSE
        enc = json.loads(Path(enc_path).read_text())
        G.cur_kps = G.dr.decode(enc, door)
        markers = [pp for pp, _ in G.cur_kps]
        labels = [k["label"] for k in enc["keypoints"]]
        ok, log = capture(lambda: G.dt.do_close(ai, True, standalone=True, wrist_side="left" if side is None else side,
                                                push_offset=shifted))
    else:
        a = argparse.ArgumentParser()
        G.dp.add_push_args(a, close=True)
        a.add_argument("--retreat-dist", type=float, default=0.20)
        a.add_argument("--smooth", action="store_true")
        a.add_argument("--execute", action="store_true")
        args = a.parse_args(CONTAINER_ARGS + ["--execute"])
        ok, log = capture(lambda: G.dp.push_close(ai, args))
    t_plan = time.time() - t0
    if ok:
        status, where, planned, stuck = "OK", "", [], None
    elif kind == "old" and "out of reach by more than" in log:
        planned, stuck = untrimmed_plan(ai, door, side, labels, log)
    elif kind == "new" and "no swing start clears" in log:
        planned, stuck = swing_start_candidates(log)
    else:
        planned, stuck = planned_partial(kind, side, labels, log)
    if not ok:
        executed = sum(len(m["qs"]) for m in ai.moves)
        status = "STUCK" if (planned or executed or (stuck and stuck.get("q") is not None)) else "REFUSED"
        where = "retract" if (kind == "old" and any(m["label"].startswith("replay") for m in ai.moves)) else ""
        stuck = stuck or {"q": None, "target": None, "reason": why_line(log)}
    title = f"{t['pname']} | {STARTS[skey][0]} | {vkey}"
    door_end, _ = render(case_dir, geo, q0, door0, ai.moves, planned, stuck, status, title, kind == "new", 0.98, markers)
    (case_dir / "log.txt").write_text(f"equivalent command: {cmd}\n(run with --execute on a SIMULATED arm)\n\n" + log)
    res = {"placement": t["pname"], "start": STARTS[skey][0], "variant": vkey, "command": cmd, "status": status,
           "where": where, "reason": (stuck or {}).get("reason", ""), "door_deg_start": door0,
           "start_ik_err_cm": round(float(err) * 100, 2),
           "door_deg_end_render": door_end, "start_joints_deg": np.round(np.degrees(q0), 1).tolist(),
           "executed": [{"label": m["label"], "steps": len(m["qs"])} for m in ai.moves],
           "planned_not_run": [{"label": l, "steps": len(q)} for l, q in planned],
           "stuck_joints_deg": None if not stuck or stuck.get("q") is None else np.round(np.degrees(stuck["q"]), 1).tolist(),
           "plan_s": round(t_plan, 1), "total_s": round(time.time() - t0, 1)}
    (case_dir / "result.json").write_text(json.dumps(res, indent=1))
    work_door.unlink(missing_ok=True)
    return res


def untrimmed_plan(ai, door, side, labels, log):
    """The old close refused because its push end is out of reach (trim_to_reach). Show the same planner on the
    untrimmed keypoints: how far it gets and where it stops."""
    pts = G.cur_kps
    st = ai.get_state()
    G.rec.clear()
    _, log2 = capture(lambda: G.dr.plan(G.scene, G.rb, door, st, pts[1:], G.dt.CLOSE_TRUST_FROM, False, side))
    ok = recs("ok", "plan")
    if ok:
        segs = [(f"PLANNED, not run: close leg {i + 1} ('{labels[i + 1][:24]}')", list(l)) for i, l in enumerate(ok[-1]["result"])]
        return segs, {"q": None, "target": pts[-1][0],
                      "reason": why_line(log) + " -- yet the chained plan (shown) reaches it: only the trim pre-check refused"}
    segs, stuck = planned_partial("old", side, labels, log2)
    stuck = dict(stuck or {"q": None, "target": None, "reason": ""})
    stuck["reason"] = f"{why_line(log)} -- shown: the untrimmed plan, which stops at {stuck.get('reason', '')}"
    return segs, stuck


def swing_start_candidates(log):
    """push-close found no swing start clear of the open door: every candidate it tried (IK, clearance)."""
    gap = G.dp.GAP
    cands, i = [], 0
    while i < len(G.rec):
        r = G.rec[i]
        if r["kind"] == "dp_ik":
            c = G.rec[i + 1]["c"] if i + 1 < len(G.rec) and G.rec[i + 1]["kind"] == "dp_clear" else None
            cands.append((r, c))
        i += 1
    segs = []
    for k, (r, c) in enumerate(cands):
        what = (f"IK {r['err'] * 100:.1f} cm off" if r["err"] >= G.mc.MAX_IK_ERR else
                f"clearance {c[0] * 100:.1f} cm < {gap * 100:.0f} ({c[1]})" if c is not None else "?")
        segs.append((f"PLANNED, not run: swing-start candidate {k + 1}/{len(cands)}: {what}", [r["q"]] * 3))
    last = cands[-1][0] if cands else None
    return segs, {"q": None if last is None else last["q"], "target": None if last is None else last["pos"],
                  "reason": f"{why_line(log)} ({len(cands)} candidates tried; each was either unreachable or closer "
                            f"than {gap * 100:.0f} cm to the open door)"}


def worker(t):
    if not hasattr(G, "mc"):
        _init()
    t = dict(t)
    t["geo"] = {k: (np.asarray(v) if isinstance(v, list) else v) for k, v in t["geo"].items()}
    try:
        return task_open(t) if t["kind"] == "open" else task_close(t)
    except Exception:
        import traceback
        return {"placement": t["pname"], "start": STARTS[t["start"]][0] if t.get("start") else "0_open",
                "variant": t.get("variant", ""), "status": "ERROR", "where": "", "reason": traceback.format_exc()[-600:],
                "end_joints": None, "door_deg_end_render": None}


# ============================================================================ index
def write_index(model, placements, opens, closes):
    col = {"OK": "#1a7f37", "STUCK": "#c0392b", "REFUSED": "#b9770e", "ERROR": "#6c3483"}
    h = ["<!doctype html><meta charset=utf-8><title>Microwave open + close in sim</title>"
         "<style>body{font-family:sans-serif;margin:16px;background:#fafafa}table{border-collapse:collapse;margin-bottom:28px}"
         "td,th{border:1px solid #ccc;padding:6px;vertical-align:top;background:#fff;max-width:400px}th{background:#eee}"
         "video{width:380px}small{color:#555}.s{font-weight:bold}</style>",
         "<h1>Microwave open + close, simulated</h1>",
         f"<p>Microwave model fitted from {model['n_detections']} saved detections: hinge edge to handle "
         f"{model['r'] * 100:.1f} cm, handle {model['proud'] * 100:.1f} cm off the face at {model['handle_dz'] * 100:.1f} cm "
         f"above the door bottom, face {model['face_w'] * 100:.1f} x {model['face_h'] * 100:.1f} cm, facing "
         f"{model['yaw_deg']:+.1f} deg. Assumed: body depth {BODY_DEPTH * 100:.0f} cm, handle bar {HANDLE_LEN * 100:.0f} cm "
         f"(model/microwave_model.json).</p>"
         "<p>The real open and close code ran with <code>--execute</code> on a simulated arm. Green dots: the hand's path as "
         "run; yellow dots: what the planner planned or tried but did not run. STUCK = the planner gave up; the video "
         "then shows what it tried (the furthest attempt, every candidate, or the plan up to the refusal) and ends on "
         "the refused pose in red with the reason. Every close continues from the simulated open at the same placement. The door follows the hand while the handle is held, otherwise it is pushed by whatever "
         "touches it (rough; no physics, no force sim -- the close's torque-stopped push runs to its planned max). "
         "Orange box: the container.</p>"]
    for pn in placements:
        o = next((x for x in opens if x["placement"] == pn), {})
        h.append(f"<h2>{pn}</h2><p><b>Open:</b> <span class=s style='color:{col.get(o.get('status'), '#000')}'>"
                 f"{o.get('status', '?')}</span> <small>{o.get('reason', '')}</small><br>"
                 f"<video controls muted loop preload=none poster='{pn}/0_open/end.png' src='{pn}/0_open/trajectory.mp4'></video>"
                 f"<br><a href='{pn}/0_open/keyframes.png'>keyframes</a> &middot; <a href='{pn}/0_open/log.txt'>log</a></p>")
        if o.get("status") != "OK":
            h.append("<p><i>The open doesn't finish at this placement, so there is no close to test here.</i></p>")
            continue
        h.append("<table><tr><th>close start (after the open)</th>" + "".join(f"<th>{v}<br><small>{VARIANTS[v][3]}</small></th>" for v in VARIANTS) + "</tr>")
        for skey, (sname, _, sdesc) in STARTS.items():
            h.append(f"<tr><th>{sname}<br><small>{sdesc}</small></th>")
            for v in VARIANTS:
                s_ = next((x for x in closes if x["placement"] == pn and x.get("start") == sname and x.get("variant") == v), {})
                rel = f"{pn}/{sname}/{v}"
                h.append(f"<td><span class=s style='color:{col.get(s_.get('status'), '#000')}'>{s_.get('status', '?')}"
                         f"</span><br><small>{s_.get('reason', '')[:260]}</small><br>"
                         f"<video controls muted loop preload=none poster='{rel}/end.png' src='{rel}/trajectory.mp4'></video>"
                         f"<br><a href='{rel}/keyframes.png'>keyframes</a> &middot; <a href='{rel}/log.txt'>log</a></td>")
            h.append("</tr>")
        h.append("</table>")
    (OUT / "index.html").write_text("\n".join(h))
    md = ["# Microwave open + close, simulated", "",
          "Generated by `microwave/tools/viz_close_plans.py`; nothing was sent to the robot. Open `index.html`.",
          "This folder is git-excluded (`.git/info/exclude`).", "",
          "Layout: `<placement>/0_open/` (the simulated open) and `<placement>/<close start>/<close variant>/`, each with "
          "`trajectory.mp4`, `keyframes.png`, `end.png`, `log.txt`, `result.json`. `model/` holds the fitted microwave model.",
          "", "| placement | open | close start | variant | close | reason |", "|---|---|---|---|---|---|"]
    for pn in placements:
        o = next((x for x in opens if x["placement"] == pn), {})
        md.append(f"| {pn} | {o.get('status', '?')} | | | | {o.get('reason', '').replace('|', '/')[:150]} |")
        for s in sorted([x for x in closes if x["placement"] == pn], key=lambda x: (x["start"], x["variant"])):
            md.append(f"| {pn} | | {s['start']} | {s['variant']} | {s['status']} | "
                      f"{s.get('reason', '').replace('|', '/')[:150]} |")
    (OUT / "README.md").write_text("\n".join(md) + "\n")


def main():
    a = argparse.ArgumentParser()
    a.add_argument("--only", default=None, help="placement id, e.g. P3")
    a.add_argument("--jobs", type=int, default=10)
    args = a.parse_args()
    import os
    os.chdir(REPO)
    if OUT.exists() and args.only is None:
        shutil.rmtree(OUT)
    (OUT / "model").mkdir(parents=True, exist_ok=True)
    _init()
    model = fit_model()
    (OUT / "model" / "microwave_model.json").write_text(json.dumps(model, indent=1))
    placements, geos, i = [], {}, 0
    for bz in BOTTOM_ZS:
        for hx in HANDLE_XS:
            i += 1
            pn = pname(i, hx, bz)
            if args.only and not pn.startswith(args.only + "_"):
                continue
            geo = placement_geometry(model, hx, bz)
            placements.append(pn)
            geos[pn] = {k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in geo.items()}
            (OUT / pn).mkdir(parents=True, exist_ok=True)
            (OUT / pn / "placement.json").write_text(json.dumps(geos[pn], indent=1))
    ctx = mp.get_context("spawn")
    print(f"{len(placements)} placements: the opens first", flush=True)
    with ctx.Pool(args.jobs, maxtasksperchild=1) as pool:
        opens = list(pool.imap_unordered(worker, [{"kind": "open", "pname": pn, "geo": geos[pn]} for pn in placements]))
        for o in opens:
            print(f"  open {o['placement']}: {o['status']} {o.get('reason', '')[:200]}", flush=True)
        tasks = []
        for pn in placements:
            o = next(x for x in opens if x["placement"] == pn)
            if o["status"] != "OK":
                continue                      # no open, no close: the open's own result is the finding here
            for skey in STARTS:
                for v in VARIANTS:
                    tasks.append({"kind": "close", "pname": pn, "geo": geos[pn], "start": skey, "variant": v,
                                  "open_end_q": o.get("end_joints"), "open_door_deg": o.get("door_deg_end_render")})
        print(f"{len(tasks)} closes", flush=True)
        closes = []
        for s in pool.imap_unordered(worker, tasks):
            closes.append(s)
            print(f"[{len(closes)}/{len(tasks)}] {s['placement']} {s.get('start')} {s.get('variant')}: {s['status']} "
                  f"-- {s.get('reason', '')[:160]}", flush=True)
    # the index covers every placement on disk (so an --only rerun keeps the others)
    all_p = sorted(d.name for d in OUT.glob("P*_*") if d.is_dir())
    all_o = [json.loads(f.read_text()) for f in OUT.glob("P*_*/0_open/result.json")]
    all_c = [json.loads(f.read_text()) for f in OUT.glob("P*_*/[1-9]_*/*/result.json")]
    write_index(model, all_p, all_o, all_c)
    print(f"done: {OUT / 'index.html'}")


if __name__ == "__main__":
    main()
