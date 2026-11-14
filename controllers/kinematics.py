"""Cartesian pose error and numerical six-axis inverse kinematics."""
import numpy as np
import mujoco

def rotation_error(target, current):
    """Return the shortest world-frame rotation vector between two 3x3 matrices."""
    quat = np.empty(4)
    mujoco.mju_mat2Quat(quat, (target @ current.T).ravel())
    # q and -q represent the same rotation; choose the shorter angular correction.
    if quat[0] < 0:
        quat *= -1
    norm = np.linalg.norm(quat[1:])
    return 2 * quat[1:] if norm < 1e-10 else quat[1:] * (2*np.arctan2(norm, quat[0])/norm)


def ik(m, tip, position, rotation, seed):
    """Damped least-squares IK for all six pose coordinates; no joint teleport in physics."""
    scratch = mujoco.MjData(m)
    scratch.qpos[:] = seed
    # Jacobian rows map joint velocities to world-frame tool translation/rotation.
    jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
    for _ in range(400):
        mujoco.mj_forward(m, scratch)
        ep = position - scratch.site_xpos[tip]
        er = rotation_error(rotation, scratch.site_xmat[tip].reshape(3, 3))
        if np.linalg.norm(ep) < 2e-5 and np.linalg.norm(er) < 2e-4:
            return scratch.qpos.copy()
        mujoco.mj_jacSite(m, scratch, jp, jr, tip)
        # 0.12 m/radian balances position and rotation residuals.
        jac = np.vstack([jp, .12*jr])
        # Damping limits sensitivity near singular poses; cap each iteration below.
        delta = jac.T @ np.linalg.solve(jac@jac.T + 1e-5*np.eye(6), np.r_[ep, .12*er])
        delta *= min(1, .15/max(np.max(np.abs(delta)), 1e-12))
        scratch.qpos[:] += delta
        for j in range(m.njnt):
            if m.jnt_limited[j]:
                scratch.qpos[j] = np.clip(scratch.qpos[j], *m.jnt_range[j])
    raise RuntimeError('Pose IK did not converge; choose a closer, reachable pose')


