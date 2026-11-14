"""Compare tool XYZ against the designer's published forward-kinematics equations.
Source: https://github.com/Mr-Jos-Technic/Lego-EV3-6DoF/blob/main/6dof_left_accurate.py
Algebraically regrouped equations, lengths in metres; no motor gearing/rounding.
Tool-roll zero and encoder conventions still require physical calibration.
"""
from robot_models import ARM_XML
import numpy as np
import mujoco

m = mujoco.MjModel.from_xml_path(str(ARM_XML))
d = mujoco.MjData(m)
rng = np.random.default_rng(6)
maximum = 0.
for _ in range(100):
    q = rng.uniform([-2,-1,-.8,-2,-1.4,-2], [2,.7,1.7,2,1.4,2])
    yaw, shoulder, elbow, roll, wrist, _ = q
    beta = shoulder + np.pi/2 - elbow
    radial = (.168*np.cos(shoulder+np.pi/2) + .047*np.cos(beta)
              + .152*np.sin(beta) + .115*(np.sin(beta)*np.cos(wrist)
              - np.cos(beta)*np.cos(roll)*np.sin(wrist)))
    lateral = -.115*np.sin(roll)*np.sin(wrist)
    z = (.165 + .168*np.sin(shoulder+np.pi/2) + .047*np.sin(beta)
         - .152*np.cos(beta) - .115*(np.cos(beta)*np.cos(wrist)
         + np.sin(beta)*np.cos(roll)*np.sin(wrist)))
    expected = [np.cos(yaw)*radial-np.sin(yaw)*lateral,
                np.sin(yaw)*radial+np.cos(yaw)*lateral, z]
    d.qpos[:] = q
    mujoco.mj_forward(m, d)
    maximum = max(maximum, np.linalg.norm(d.site('tool_tip').xpos-expected))
assert maximum < 1e-10, maximum
print(f'PASS: 100 configurations agree with published tool XYZ equations; max error {maximum:.2e} m')
