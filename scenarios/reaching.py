"""The same four reachable Cartesian poses and motion reference for both cases."""
import numpy as np
import mujoco

# Joint order: base, shoulder, elbow, arm roll, wrist, tool roll.
# Speed limits shape the reference; they do not clamp physical joint velocity.
SPEED = np.deg2rad([35, 15, 20, 45, 35, 60])
POSE_TIME = 9.0
POSITION_TOLERANCE_MM = 10
ORIENTATION_TOLERANCE_DEG = 3
ARRIVAL_DWELL_SECONDS = .5
INITIAL_ANGLES = np.deg2rad([0, -15, 20, 0, -20, 0])

def demo_targets(m, tip):
    """Return world XYZ (metres) and 3x3 rotation matrices for four tool poses."""
    # Known safe configurations generate reachable XYZ+orientation examples.
    # IK still solves those Cartesian poses; these are not sent to the controller.
    configurations = np.deg2rad([
        [0, -20, 25, 15, -25, 10],
        [35, -35, 40, -30, -40, 45],
        [-30, -10, 15, 35, -20, -35],
        [10, -40, 5, -20, 20, 60],
    ])
    scratch = mujoco.MjData(m)
    targets = []
    for q in configurations:
        scratch.qpos[:] = q
        mujoco.mj_forward(m, scratch)
        targets.append((scratch.site_xpos[tip].copy(), scratch.site_xmat[tip].reshape(3, 3).copy()))
    return targets


