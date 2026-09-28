import sys
import numpy as np
from scipy.spatial.transform import Rotation as R
sys.path.insert(0, "microwave")
from feeding_deployment.control.robot_controller.arm_client import ArmInterfaceClient
from microwave_common import make_sim, solve_ik
st = ArmInterfaceClient().get_state()
ee, q0 = np.array(st["ee_pos"], float), np.array(st["position"], float)
m = R.from_quat(ee[3:7]).as_matrix()
print("approach (tool z)", np.round(m[:, 2], 3), " tool x", np.round(m[:, 0], 3), " tool y", np.round(m[:, 1], 3))
scene, rb = make_sim()
dirs = {"-approach": -m[:, 2], "world -x": [-1, 0, 0], "world +y": [0, 1, 0], "world -y": [0, -1, 0],
        "world +z": [0, 0, 1], "-approach+z": -m[:, 2] + [0, 0, 1], "-approach -x": -m[:, 2] + [-1, 0, 0]}
for nm, d in dirs.items():
    d = np.asarray(d, float); d /= np.linalg.norm(d)
    q, out = q0.copy(), []
    for k in (1, 2, 3):
        q, err = solve_ik(scene, rb, ee[:3] + d * 0.05 * k / 3, ee[3:7], q)
        out.append(f"{np.degrees(q[5]):.1f}")
    print(f"{nm:12s} dir {np.round(d, 2)}  J6 over 5 cm: {' -> '.join(out)}  (now {np.degrees(q0[5]):.1f})  ik err {err*100:.2f}cm")
# rotating the hand about vertical by +-10 deg in place
for yaw in (-10, 10):
    qq, err = solve_ik(scene, rb, ee[:3], (R.from_euler("z", yaw, degrees=True) * R.from_quat(ee[3:7])).as_quat(), q0)
    print(f"yaw {yaw:+d} deg in place: J6 {np.degrees(qq[5]):.1f}  ik err {err*100:.2f}cm")
