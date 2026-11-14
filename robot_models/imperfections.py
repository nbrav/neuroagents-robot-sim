"""Illustrative output-equivalent mechanics/sensors; NOT calibrated EV3 specifications."""
# Six-element arrays use J1 through J6, matching the XML joint/motor order.
# All angles here are output-equivalent degrees, converted to radians in the loop.
PARAMETERS = {
    'sensor_period_s': .01,
    'feedback_delay_s': .03,
    'angle_noise_std_deg': .08,
    'velocity_noise_std_deg_s': .5,
    'encoder_step_deg': .1,  # Output-equivalent resolution, not motor-shaft resolution.
    # First-order motor torque response, before transmission spring dynamics.
    'torque_time_constant_s': .025,
    'no_load_speed_deg_s': [70, 35, 45, 100, 80, 130],
    'backlash_total_deg': [1, 1.2, 1.2, 1.8, 1.8, 2],
    'transmission_stiffness_Nm_rad': [20, 25, 20, 4, 4, 2],
    'transmission_damping_Nm_s_rad': [.15, .18, .15, .05, .05, .025],
    # Rotor inertia reflected to the joint output, in kg m^2.
    'output_equivalent_motor_inertia': [.02, .02, .02, .008, .008, .005],
    'motor_viscous_drag_Nm_s_rad': .01,
    'joint_friction_Nm': [.015, .025, .02, .006, .006, .003],
}
