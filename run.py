"""Run the two reaching cases or compare their shared evaluation results."""
from pathlib import Path
import argparse
import csv
import time
import numpy as np
from simulation.experiment import Experiment, FIELDS
from scenarios.reaching import POSE_TIME


def run_headless(experiment):
    """Evaluate one complete cycle; hard-case misses are valid measured outcomes."""
    for _ in range(round(len(experiment.targets)*POSE_TIME/experiment.m.opt.timestep)):
        experiment.step()
    assert len(experiment.results) == len(experiment.targets)
    assert all(np.all(np.array(row[9:]) <= experiment.limit+1e-8)
               for row in experiment.results), 'Applied torque exceeded configured limits'
    if experiment.profile == 'ideal':
        assert all(row[4] and row[2] < 5 and row[3] < 2 and row[7] == 0
                   for row in experiment.results), 'Easy case failed baseline accuracy check'


def run_viewer(experiment):
    """Run physics at its own timestep and refresh the viewer at about 60 Hz."""
    import mujoco
    from mujoco import viewer as mjviewer
    e = experiment
    with mjviewer.launch_passive(e.m, e.d) as viewer:
        with viewer.lock():
            viewer.cam.lookat[:] = [.12, 0, .23]
            viewer.cam.distance = .95
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -22
        batch = max(1, round(1/(60*e.m.opt.timestep)))
        while viewer.is_running():
            start = time.perf_counter()
            with viewer.lock():
                for _ in range(batch):
                    e.step()
                mujoco.mj_forward(e.m, e.d)
            viewer.sync()
            time.sleep(max(0, batch*e.m.opt.timestep-(time.perf_counter()-start)))


def main():
    parser = argparse.ArgumentParser(description='Six-axis arm: easy PD baseline and harder imperfect simulation.')
    parser.add_argument('case', choices=['easy', 'hard', 'compare'],
                        help='compare runs both cases without a viewer')
    parser.add_argument('--headless', action='store_true', help='Run one four-target cycle without a window')
    parser.add_argument('--seed', type=int, default=7, help='Noise seed for reproducible hard-case runs')
    parser.add_argument('--csv', type=Path, help='Save completed target results here (replaces this file)')
    parser.add_argument('--breakdown', action='store_true',
                        help='With compare, also isolate sampling, motor and feedback effects')
    args = parser.parse_args()
    if args.breakdown and args.case != 'compare':
        parser.error('--breakdown is only used with compare')
    profiles = ['ideal', 'combined'] if args.case == 'compare' else [
        'ideal' if args.case == 'easy' else 'combined']
    if args.breakdown:
        profiles = ['ideal', 'sampled', 'motors', 'feedback', 'combined']
    # Collect completed targets even if viewing stops or an exception interrupts a run.
    rows = []
    try:
        for profile in profiles:
            e = Experiment(profile, args.seed)
            print(f"Starting {'EASY' if profile == 'ideal' else 'HARD' if profile == 'combined' else profile} case", flush=True)
            try:
                if args.headless or args.case == 'compare':
                    run_headless(e)
                else:
                    run_viewer(e)
            finally:
                rows.extend(e.results)
    finally:
        if args.csv:
            args.csv.parent.mkdir(parents=True, exist_ok=True)
            with args.csv.open('w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(FIELDS)
                writer.writerows(rows)
            print(f'Saved {args.csv.resolve()}')
    for profile in profiles:
        completed = [row for row in rows if row[0] == profile]
        if completed:
            print(f'{profile}: {sum(bool(row[4]) for row in completed)}/{len(completed)} successful reaches')


if __name__ == '__main__':
    main()
