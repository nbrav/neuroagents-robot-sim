"""Check delay timing, quantization, repeatability and torque bounds."""
import numpy as np
from simulation.experiment import Experiment


def main():
    e = Experiment('feedback')
    initial = e.samples[0][0].copy()
    e.d.qpos[:] += .1
    for _ in range(3):
        q, _ = e.sensed()
        assert np.array_equal(q, initial), 'Feedback arrived before the configured delay'
    q, _ = e.sensed()
    assert np.all(np.abs(q-initial) > .08), 'Delayed sample did not arrive'
    quantum = np.deg2rad(.1)
    assert np.allclose(q/quantum, np.round(q/quantum))
    for seed in [7, 8]:
        a, b = Experiment('combined', seed), Experiment('combined', seed)
        for _ in range(1000):
            a.step()
            b.step()
        assert np.array_equal(a.d.qpos, b.d.qpos), 'Seeded runs diverged'
        assert np.all(np.abs(a.d.ctrl) <= a.limit+1e-9)
    print('PASS: delay timing, encoder quantization, seeded repeatability and torque bounds')


if __name__ == '__main__':
    main()
