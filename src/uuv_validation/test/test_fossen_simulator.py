"""The vehicle model moves as the same equations do on Fossen's Python Vehicle Simulator."""

import io
import os
import subprocess

import fossen_heavy
import numpy as np
import pytest

STEP_S = 0.001
# The whole velocity trajectory must stay this close to the reference's, as a fraction of the
# fastest the reference goes. The two agree to under 0.1%, the difference between their
# integrators. Sway's linear damping 3% out moves sway's decay 0.5% and fails.
TRAJECTORY_TOLERANCE = 0.0025


def simulate(velocity, seconds):
    """Run uuv_dynamics from a level start and return its body velocity at every step."""
    command = [
        os.environ['SIMULATE'], '--seconds', str(seconds), '--step', str(STEP_S),
        '--velocity', *map(str, velocity)]
    output = subprocess.run(command, check=True, capture_output=True, text=True).stdout
    return np.loadtxt(io.StringIO(output), delimiter=',', skiprows=1)[:, 1:7]


def worst_difference(ours, reference):
    """Return the largest velocity difference, as a fraction of the reference's top speed."""
    speeds = np.linalg.norm(reference, axis=1)
    return np.linalg.norm(ours - reference, axis=1).max() / speeds.max()


@pytest.mark.parametrize('axis', range(6), ids=['surge', 'sway', 'heave', 'roll', 'pitch', 'yaw'])
def test_free_decay_matches_fossens_simulator(axis):
    velocity = [0.0] * 6
    velocity[axis] = 0.5  # m/s or rad/s

    ours = simulate(velocity, seconds=10)
    reference = fossen_heavy.simulate(velocity, np.zeros(6), seconds=10, step=STEP_S)

    assert worst_difference(ours, reference) < TRAJECTORY_TOLERANCE
