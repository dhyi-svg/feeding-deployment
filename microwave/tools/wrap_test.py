"""Supervised +-180 wrap test on a free-spinning joint (J7, or J1) -- MOVES THE ARM.

Question it answers: when a JointCommand crosses +-180 on a free-spinning joint, does Kortex
take the short way (a few deg) or the long way (~350 deg)? Until this is known the microwave
planners refuse any step that crosses the wrap (`continuous_ok` in microwave_common.py).

Sends each target the way the planners do (wrapped to [-180, 180), all 7 joints, others held)
on one RPC connection, while a second connection polls the joint at 50 Hz and calls
stop_action() if it turns the WRONG way by > 3 deg or more than 25 deg past the expected change.
Gripper empty, nothing near the hand, e-stop in reach. Run J7 first; J1 only if J7 passes.

    ARM_RPC_HOST=127.0.0.1 python3 -u microwave/tools/wrap_test.py 7
"""
import sys, threading, time

import numpy as np

from feeding_deployment.control.robot_controller.arm_interface import (
    ARM_RPC_PORT, NUC_HOSTNAME, RPC_AUTHKEY, ArmManager)

ArmManager.register("ArmInterface")


def proxy():
    m = ArmManager(address=(NUC_HOSTNAME, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
    m.connect()
    return m.ArmInterface()


cmd_arm, mon = proxy(), proxy()      # blocking moves on one connection, watching on the other
wrap = lambda d: (d + 180.0) % 360.0 - 180.0


def joints_deg():
    return np.degrees(np.array(mon.get_state()["position"], float))


def move(j, target_deg, label):
    q = joints_deg()
    start = q[j]
    expect = wrap(target_deg - start)                     # short-way change
    q_cmd = q.copy()
    q_cmd[j] = wrap(target_deg)
    print(f"{label}: J{j + 1} {wrap(start):7.1f} -> {wrap(target_deg):7.1f}  (short way {expect:+.1f} deg)", flush=True)
    res = {}
    th = threading.Thread(target=lambda: res.update(ok=cmd_arm.set_joint_position(np.radians(q_cmd).tolist())))
    th.start()
    moved, prev, stopped, peak_wrong = 0.0, start, False, 0.0
    t0 = t_last_motion = time.time()
    # Keep watching after the RPC returns: on 09-25 set_joint_position returned False with 0 deg
    # moved and the arm then executed the whole move unwatched. Stop only once the joint has sat
    # still for 10 s (or 60 s total).
    while (th.is_alive() or time.time() - t_last_motion < 10.0) and time.time() - t0 < 60:
        cur = joints_deg()[j]
        if abs(wrap(cur - prev)) > 0.05:
            t_last_motion = time.time()
        moved += wrap(cur - prev)
        prev = cur
        wrong = -moved * np.sign(expect) if expect != 0 else abs(moved)
        peak_wrong = max(peak_wrong, wrong)
        if not stopped and (wrong > 3.0 or abs(moved) > abs(expect) + 25.0):
            print(f"  !! J{j + 1} going the wrong way / too far (moved {moved:+.1f} deg) -- stop_action", flush=True)
            mon.stop_action()
            stopped = True
        time.sleep(0.02)
    th.join(timeout=5)
    end = joints_deg()[j]
    print(f"  moved {moved:+.1f} deg (expected {expect:+.1f}), now {wrap(end):.1f}, max wrong-way "
          f"{peak_wrong:.1f}, returned {res.get('ok')}", flush=True)
    return (not stopped) and abs(moved - expect) < 2.0


def main():
    joint = int(sys.argv[1]) - 1
    if joint not in (0, 6):
        sys.exit("test J7 (arg 7) or J1 (arg 1)")
    home = joints_deg()[joint]
    if joint == 6:
        seq = [(-160, "approach"), (-175, "approach"), (175, "CROSS"), (-175, "CROSS BACK"),
               (-160, "return"), (home, "return")]
    else:
        seq = [(170, "approach"), (177, "approach"), (-178, "CROSS"), (177, "CROSS BACK"), (home, "return")]
    st = mon.get_state()
    print(f"gripper {st['gripper_pos']:.3f}; J{joint + 1} now {wrap(home):.1f}; speed {mon.get_speed()}")
    for tgt, label in seq:
        if not move(joint, tgt, label):
            sys.exit(f"STOPPED at {label} -> {tgt}: not the short way (or not reached). Check the arm.")
    print(f"J{joint + 1} WRAP TEST PASSED: Kortex took the short way across +-180 both directions.")


if __name__ == "__main__":
    main()
