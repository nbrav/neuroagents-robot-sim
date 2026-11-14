# Simulation results

`reference/comparison.csv` is the full five-profile comparison for noise seed 7 using the included model, scenario and unchanged PD gains. `ideal` maps to the easy case; `combined` maps to hard. `sampled`, `motors` and `feedback` isolate the additional effects. The cleanup was checked against this file; all recorded values stayed unchanged.

Save future runs under `results/runs/` (ignored by Git). For example, from the project root:

```bash
python3 run.py compare --csv results/runs/comparison.csv
```

| Column | Meaning |
|---|---|
| `profile` | Applied controller/actuator/sensor configuration |
| `pose` | Target number, 1–4; repeats in interactive runs |
| `final_error_mm` | Euclidean tool position error at the end of the interval |
| `final_error_deg` | Shortest tool orientation error at the end of the interval |
| `success` | Arrival dwell observed and final errors inside tolerances |
| `arrival_s` | Time from target activation until confirmed arrival, including dwell; blank if never confirmed |
| `rms_position_error_mm` | RMS position error across the full movement, including travel toward the target |
| `contact_steps` | Number of physics steps with at least one detected contact |
| `torque_limit_fraction` | Fraction of steps when at least one requested motor torque exceeds available torque |
| `peak_joint_torque_1_Nm` … `peak_joint_torque_6_Nm` | Largest absolute applied joint torque per joint |

A failed final success check can coexist with an arrival time if the arm briefly settled and later moved out of tolerance. Contact counts refer to the simplified collision geometry. The motor model's available torque changes with rotor speed; it is distinct from the transmission's fixed joint torque bounds.

Reference results describe the uncalibrated simulation only. Do not interpret them as measured EV3 performance.
