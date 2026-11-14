# The two cases

The experiment asks: **Can the same PD controller reach the same tool poses when sensing and actuation become imperfect?**

| Property | Easy | Hard |
|---|---|---|
| Robot and target poses | Same six-axis geometry and four reachable poses | Same |
| IK and reference speeds | Same | Same |
| P and D gains | Same | Same; deliberately not retuned |
| Joint feedback | Exact simulator state | Motor-side encoder estimate |
| Feedback delay | None | 30 ms |
| Controller rate | 500 Hz | 100 Hz held command |
| Feedback imperfections | None | Quantization, angle and velocity noise |
| Motor response | Immediate ideal torque | 25 ms lag, rotor inertia, torque–speed envelope |
| Transmission | Rigid | Lost motion and torsional spring/damper |
| Added friction | None beyond XML damping | Joint friction plus rotor drag |
| Compensation | Exact simulated gravity and dynamic bias | Gravity estimate from imperfect encoder angles |
| Evaluation | Same arrival criteria and metrics | Same |

The PD law is `Kp*(reference-position) + Kd*(desired_velocity-velocity) + compensation`. IK converts the desired tool pose into joint angles. A speed limiter creates a gradual joint reference. Compensation is included before applying torque limits. The harder actuator model adds motor response and transmission dynamics after this torque request.

The hard case uses a virtual motor on each joint. Its encoder sees motor-side position expressed in output-joint units. A spring with a slack region connects the motor to the actual joint. This lets the encoder report the desired position while the loaded joint remains offset. This compliance represents a simplified transmission; the arm's individual structural beams remain rigid.

Truth from the simulator evaluates success, but does not feed the hard controller. Velocity feedback is a separately perturbed signal rather than differentiation of noisy position. Noise is seeded for repeatability.

A reach succeeds after continuously remaining within 10 mm and 3 degrees for 0.5 seconds, provided it also meets those limits at the end of the nine-second target interval. Arrival time includes the confirmation half-second. The target sequence keeps moving on schedule; it does not wait forever for a struggling controller.

The included seed-7 comparison gives 4/4 successes in easy and 0/4 in hard. This demonstrates sensitivity to modeling assumptions and feedback delay; it does not establish how the physical arm will perform. Some hard runs make contact. Current PD gains were chosen for the ideal case and are intentionally unchanged.

`compare --breakdown` provides diagnostic runs: ideal, sampled control only, motor effects only, feedback effects only, then combined effects. The sampling-only run helps separate slower controller updates from the added imperfections.

Change gains in `controllers/pd.py`, targets, reference speeds and arrival criteria in `scenarios/reaching.py`, and added hardware settings in `robot_models/imperfections.py`. Keep the reference results as a baseline when trying changes. No tuning was performed during the file reorganization.
