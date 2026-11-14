# NeuroAgents — six-axis robot simulation

Compare the same **PD controller with model compensation** under two simulation conditions:

| Case | Conditions | Reference result |
| --- | --- | --- |
| **Easy** | Ideal motors, rigid transmission, exact feedback | 4/4 targets reached |
| **Hard** | Delays, noise, gear play, compliance and motor limits | 0/4 targets reached with unchanged gains |

Both cases use the same six-axis LEGO-style robot, four target tool poses and reference speeds. The model is inspired by [Mr Jos's LEGO EV3 arm](https://github.com/Mr-Jos-Technic/Lego-EV3-6DoF). Hardware properties are illustrative, not calibrated EV3 measurements. Hard-case failures are expected experiment results.

## 1. Install

**Use Python 3.11** for the tested dependency versions. No physical robot or EV3 software is needed. A graphical desktop is needed only for the interactive viewer.

Download/copy the repository and open a terminal in the directory containing `run.py` and `requirements.txt`. Folder names can contain spaces; quote paths when changing directories. On the original Mac:

```bash
cd "$HOME/Desktop/MuJoCo tests"
```

### macOS / Linux

```bash
python3.11 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

If `python3` already points to Python 3.11, it can replace `python3.11` above. Activate `.venv` again when opening a new terminal. The MuJoCo package includes the physics engine; no separate engine download is required.

### Windows (PowerShell)

```powershell
py -3.11 -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

For the commands below, replace `python` with `.\.venv\Scripts\python.exe` on Windows. This avoids needing to activate the environment.

Dependencies are pinned in `requirements.txt` to the local reference environment: **MuJoCo 3.17.0 and NumPy 1.26.1**. macOS is the locally tested platform; Linux/Windows commands are provided but have not been tested here.

## 2. Run

From the project directory, with the environment active:

| Task | macOS | Linux / Windows |
| --- | --- | --- |
| Easy viewer | `mjpython run.py easy` | `python run.py easy` |
| Hard viewer | `mjpython run.py hard` | `python run.py hard` |
| Compare without a window | `python run.py compare` | `python run.py compare` |

**macOS requires `mjpython` for the viewer.** Use ordinary Python for headless runs. Close the viewer window to stop the interactive demo.

In the viewer, yellow marks the desired tool position, RGB axes show its desired orientation, and green marks the actual tool reference. Targets switch every nine simulation seconds, even after a miss. The viewer repeats the sequence; headless runs execute one four-target cycle per case. The fork is fixed and does not open or close.

### Record or isolate effects

```bash
# Compare both cases and save completed target results.
python run.py compare --seed 7 --csv results/runs/comparison.csv

# Also isolate sampling, motor and feedback effects.
python run.py compare --breakdown --csv results/runs/breakdown.csv

# Run just one case without a viewer.
python run.py easy --headless
python run.py hard --headless --seed 7

# See all command options.
python run.py --help
```

Use `mjpython run.py hard --csv results/runs/hard.csv` to record an interactive macOS run. CSV output is saved when the run ends, includes completed target intervals only, and replaces an existing file at the supplied path. Close the viewer normally to finish recording.

## 3. Files and directories

```text
.
├── run.py                         # CLI, viewer, headless runs and CSV export
├── requirements.txt               # Tested Python dependencies
├── controllers/
│   ├── pd.py                      # Shared proportional/derivative gains and torque law
│   └── kinematics.py              # Numerical pose IK and shortest rotation error
├── robot_models/
│   ├── mr_jos_6dof.xml             # Geometry, joints, estimated masses and torque limits
│   └── imperfections.py           # Hard-case delay, noise, motor and transmission parameters
├── scenarios/
│   └── reaching.py                # Four target poses, initial angles, speeds and tolerances
├── simulation/
│   └── experiment.py              # Physics loop, feedback/drive dynamics and metrics
├── checks/
│   ├── check_geometry.py          # Compare tool XYZ with published robot equations
│   └── check_feedback.py          # Check delay, quantization, repeatability and torque bounds
├── docs/
│   ├── cases.md                   # Detailed easy/hard comparison and controller explanation
│   └── model_assumptions.md        # Robot sources, dimensions and modeling limitations
├── results/
│   ├── README.md                  # CSV column definitions and reference-run interpretation
│   └── reference/comparison.csv   # Included seed-7 results for all five diagnostic profiles
└── .gitignore                     # Excludes environments, caches and new run results
```

`__init__.py` files mark Python packages; `robot_models/__init__.py` also locates the XML relative to the project. Move/share the entire folder together. Model paths do not depend on a particular user's Desktop location.

## 4. Controller and evaluation

**Tool pose → numerical inverse kinematics → speed-limited joint reference → PD + compensation → actuators → robot motion.** There is no integral term, learning or automatic gain tuning. The reference limits joint command speed; it does not guarantee a straight tool path.

Easy runs control and physics at 500 Hz with exact state and dynamic bias compensation. Hard holds the PD request between 100 Hz updates, using gravity estimated from imperfect motor-side measurements. It adds 30 ms feedback delay, encoder quantization/noise, 25 ms motor lag, speed-dependent torque limits, friction, backlash and transmission compliance. Structural links remain rigid.

Success requires remaining within **10 mm and 3° for 0.5 seconds**, and ending the target interval inside those tolerances. Arrival time includes the confirmation period. Each case's full cycle is 36 simulation seconds; wall-clock runtime depends on the computer.

Seed-7 reference results: easy final position errors are below 0.01 mm; hard errors are approximately 36–211 mm. These are simulation results, not hardware accuracy. Some hard-case movements make contact. See [case details](docs/cases.md), [model assumptions](docs/model_assumptions.md), and [result definitions](results/README.md).

## 5. Change or validate the experiment

| Change | File |
| --- | --- |
| P/D gains | `controllers/pd.py` |
| Targets, initial pose, reference speeds, success criteria | `scenarios/reaching.py` |
| Feedback delay/noise, motor lag, backlash and stiffness | `robot_models/imperfections.py` |
| Robot geometry, masses, travel and torque bounds | `robot_models/mr_jos_6dof.xml` |

Arrays follow J1 base, J2 shoulder, J3 elbow, J4 forearm roll, J5 wrist, J6 tool roll. Runtime units are radians, metres and N·m; parameter names identify degree-valued inputs.

```bash
python -m checks.check_geometry
python -m checks.check_feedback
python run.py compare
```

The easy headless case checks accuracy and no contacts. Hard records misses while checking finite state and bounded applied torque. Keep reference results unchanged and write new runs under `results/runs/`.

## Troubleshooting and sharing

- **`mjpython` not found / missing imports:** activate `.venv` and install requirements using that environment's Python.
- **Viewer fails on macOS:** launch it with `mjpython`; first check physics with `python run.py compare`.
- **No graphics available:** use `--headless` or `compare`; they do not open a window.
- **`No module named controllers` or missing model:** keep the complete directory structure and use the commands from the project root. Run checks with `python -m checks.check_geometry`, not by launching that check file directly.
- **Different numerical results:** confirm dependency versions and noise seed. Reference results do not guarantee identical floating-point output across platforms.

Share the source directories, documentation, requirements and reference results. Omit `.venv/`, `__pycache__/`, `.DS_Store` and optional `results/runs/` outputs. No remote repository is configured in this folder; clone/publish instructions depend on where the team chooses to host it.
