"""Shared PD gains and torque law. No integral term or automatic retuning."""
import numpy as np

# Arrays follow J1 base, J2 shoulder, J3 elbow, J4 arm roll, J5 wrist, J6 tool roll.
# Angles are radians, velocities radians/second and output torques Nm.
KP = np.array([8, 12, 10, 3, 3, 1.5])
KD = np.array([1, 1.4, 1.1, .3, .3, .15])


def torque_command(reference, position, desired_velocity, velocity, compensation):
    """Return an unclipped torque request; actuator limits are applied by the simulation."""
    return KP*(reference-position) + KD*(desired_velocity-velocity) + compensation
