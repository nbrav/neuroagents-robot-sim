"""Shared physics loop, imperfect drive/feedback models, and arrival metrics."""
from collections import deque
import numpy as np
import mujoco
from controllers.kinematics import ik, rotation_error
from controllers.pd import torque_command
from robot_models import ARM_XML
from robot_models.imperfections import PARAMETERS
from scenarios.reaching import (
    SPEED, POSE_TIME, INITIAL_ANGLES, demo_targets,
    POSITION_TOLERANCE_MM, ORIENTATION_TOLERANCE_DEG, ARRIVAL_DWELL_SECONDS,
)

PROFILES = ('ideal', 'sampled', 'motors', 'feedback', 'combined')
FIELDS = ['profile', 'pose', 'final_error_mm', 'final_error_deg',
          'success', 'arrival_s', 'rms_position_error_mm', 'contact_steps',
          'torque_limit_fraction', *[f'peak_joint_torque_{j}_Nm' for j in range(1, 7)]]


class Experiment:
    """Advance one shared scenario with an ideal or imperfect control/actuation profile.

    m/d are the true plant model/state. nominal/estimate belong only to the
    compensation calculation; the hard controller cannot access true feedback.
    Joint coordinates and motor arrays share the XML order J1 through J6.
    """
    def __init__(self, profile, seed=7):
        if profile not in PROFILES:
            raise ValueError(f'Unknown experiment profile: {profile}')
        self.profile = profile
        self.motors = profile in ('motors', 'combined')
        self.feedback = profile in ('feedback', 'combined')
        self.m = mujoco.MjModel.from_xml_path(str(ARM_XML))
        self.nominal = mujoco.MjModel.from_xml_path(str(ARM_XML))
        self.estimate = mujoco.MjData(self.nominal)
        self.d = mujoco.MjData(self.m)
        self.tip = self.m.site('tool_tip').id
        self.mocap = int(self.m.body_mocapid[self.m.body('target').id])
        self.targets = demo_targets(self.m, self.tip)
        self.d.qpos[:] = INITIAL_ANGLES
        # ref is the speed-limited reference; goal is the final IK solution.
        self.ref = self.d.qpos.copy()
        self.motor_q = self.d.qpos.copy()
        self.motor_v = np.zeros(6)
        self.effort = np.zeros(6)
        self.command = np.zeros(6)
        self.rng = np.random.default_rng(seed)
        self.sensor_steps = max(1, round(PARAMETERS['sensor_period_s']/self.m.opt.timestep))
        self.delay_samples = round(PARAMETERS['feedback_delay_s']/PARAMETERS['sensor_period_s'])
        # Include the current sample plus the full delay history.
        self.samples = deque([(self.motor_q.copy(), self.motor_v.copy())
                              for _ in range(self.delay_samples+1)], maxlen=self.delay_samples+1)
        if self.motors:
            self.m.dof_frictionloss[:] = PARAMETERS['joint_friction_Nm']
        self.limit = self.m.actuator_ctrlrange[:, 1].copy()
        self.inertia = np.array(PARAMETERS['output_equivalent_motor_inertia'])
        self.stiffness = np.array(PARAMETERS['transmission_stiffness_Nm_rad'])
        self.damping = np.array(PARAMETERS['transmission_damping_Nm_s_rad'])
        self.half_gap = .5*np.deg2rad(PARAMETERS['backlash_total_deg'])
        self.free_speed = np.deg2rad(PARAMETERS['no_load_speed_deg_s'])
        if self.motors:
            # Begin holding the initial load, rather than dropping through slack.
            mujoco.mj_forward(self.m, self.d)
            gravity = self.d.qfrc_bias.copy()
            self.motor_q += np.sign(gravity)*self.half_gap + gravity/self.stiffness
            self.effort[:] = gravity
            self.samples = deque([(self.motor_q.copy(), self.motor_v.copy())
                                  for _ in range(self.delay_samples+1)], maxlen=self.delay_samples+1)
        self.count = self.index = self.finished = 0
        self.results = []
        self.reset_metrics()
        self.goal = ik(self.m, self.tip, *self.targets[0], self.ref)
        self.set_marker()

    def reset_metrics(self):
        """Reset per-target accumulators, preserving the run-wide step count."""
        self.peak = np.zeros(6)
        self.contacts = self.saturated = self.steps = self.dwell = 0
        self.arrival = None
        self.squared_error = 0.

    def set_marker(self):
        """Move the non-colliding mocap target and its orientation axes."""
        p, r = self.targets[self.index]
        self.d.mocap_pos[self.mocap] = p
        mujoco.mju_mat2Quat(self.d.mocap_quat[self.mocap], r.ravel())

    def sensed(self):
        """Sample encoder-equivalent position/velocity at controller update times.

        Copies prevent noise from modifying the real state. A full deque is
        prefilled at startup so the oldest sample models feedback delay.
        """
        q = self.motor_q.copy() if self.motors else self.d.qpos.copy()
        v = self.motor_v.copy() if self.motors else self.d.qvel.copy()
        if self.feedback:
            q += self.rng.normal(0, np.deg2rad(PARAMETERS['angle_noise_std_deg']), 6)
            quantum = np.deg2rad(PARAMETERS['encoder_step_deg'])
            q = np.round(q/quantum)*quantum
            v += self.rng.normal(0, np.deg2rad(PARAMETERS['velocity_noise_std_deg_s']), 6)
            self.samples.append((q, v))
            q, v = self.samples[0]
        return q, v

    def step(self):
        """Advance reference, controller, drive mechanics, physics, then evaluation."""
        m, d = self.m, self.d
        dt = m.opt.timestep
        mujoco.mj_forward(m, d)
        previous = self.ref.copy()
        self.ref += np.clip(self.goal-self.ref, -SPEED*dt, SPEED*dt)
        desired_v = (self.ref-previous)/dt
        # Ideal baseline is unchanged. Imperfect modes use a 100 Hz held command.
        if self.profile == 'ideal':
            self.command = torque_command(self.ref, d.qpos, desired_v, d.qvel, d.qfrc_bias)
        elif self.count % self.sensor_steps == 0:
            q, v = self.sensed()
            # Gravity is estimated from encoder feedback, never the true plant state.
            # Zero estimated velocity means this adds gravity, not exact bias cancellation.
            self.estimate.qpos[:] = q
            self.estimate.qvel[:] = 0
            mujoco.mj_forward(self.nominal, self.estimate)
            self.command = torque_command(self.ref, q, desired_v, v, self.estimate.qfrc_bias)
        if self.motors:
            # Conservative symmetric torque-speed envelope. No regen model.
            available = self.limit*np.maximum(0, 1-np.abs(self.motor_v)/self.free_speed)
            requested = np.clip(self.command, -available, available)
            self.effort += (1-np.exp(-dt/PARAMETERS['torque_time_constant_s']))*(requested-self.effort)
            self.effort = np.clip(self.effort, -available, available)
            gap = self.motor_q-d.qpos
            # No spring force inside the backlash gap; engage either tooth flank.
            deflection = np.sign(gap)*np.maximum(np.abs(gap)-self.half_gap, 0)
            engaged = np.abs(gap) > self.half_gap
            spring = self.stiffness*deflection + engaged*self.damping*(self.motor_v-d.qvel)
            # Idealized transmission safety/slip limit; spring reaction acts on rotor.
            torque = np.clip(spring, -self.limit, self.limit)
            self.motor_v += dt*(self.effort-torque-PARAMETERS['motor_viscous_drag_Nm_s_rad']*self.motor_v)/self.inertia
            self.motor_q += dt*self.motor_v
        else:
            available = self.limit
            torque = np.clip(self.command, -available, available)
        d.ctrl[:] = torque
        self.saturated += int(np.any(np.abs(self.command) > available+1e-9))
        self.peak = np.maximum(self.peak, np.abs(torque))
        self.contacts += int(d.ncon > 0)
        mujoco.mj_step(m, d)
        mujoco.mj_forward(m, d)
        p, r = self.targets[self.index]
        mm = 1000*np.linalg.norm(d.site_xpos[self.tip]-p)
        deg = np.rad2deg(np.linalg.norm(rotation_error(r, d.site_xmat[self.tip].reshape(3, 3))))
        self.steps += 1
        self.count += 1
        self.squared_error += mm*mm
        # Ground truth is only used for evaluation, not imperfect controller feedback.
        self.dwell = self.dwell+1 if mm <= POSITION_TOLERANCE_MM and deg <= ORIENTATION_TOLERANCE_DEG else 0
        if self.dwell*dt >= ARRIVAL_DWELL_SECONDS and self.arrival is None:
            self.arrival = self.steps*dt  # Time until confirmed arrival (includes dwell).
        if not np.all(np.isfinite(d.qpos)) or np.max(np.abs(d.qvel)) > 200:
            raise RuntimeError(f'Unstable simulation in {self.profile}')
        if self.steps*dt + 1e-9 >= POSE_TIME:
            # Success requires both an observed dwell and final accuracy.
            success = self.arrival is not None and mm <= POSITION_TOLERANCE_MM and deg <= ORIENTATION_TOLERANCE_DEG
            row = [self.profile, self.index+1, mm, deg, success,
                   self.arrival if self.arrival is not None else '',
                   np.sqrt(self.squared_error/self.steps), self.contacts,
                   self.saturated/self.steps, *self.peak.tolist()]
            # CSV row order is defined by FIELDS at the top of this module.
            self.results.append(row)
            print(f'{self.profile:8} pose {self.index+1}: {mm:7.2f} mm / {deg:6.2f} deg; '
                  f"{'REACHED' if success else 'MISSED'}; arrival {self.arrival}", flush=True)
            self.finished += 1
            self.index = (self.index+1) % len(self.targets)
            self.goal = ik(m, self.tip, *self.targets[self.index], self.goal)
            self.reset_metrics()
            self.set_marker()


