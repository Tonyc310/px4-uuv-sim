"""The vehicle model moves as the same equations do on Fossen's Python Vehicle Simulator."""

import io
import os
import subprocess

import fossen_heavy
import numpy as np
import pytest

STEP_S = 0.001
AT_REST = [0.0] * 6
# Pulse widths for the eight thrusters, us: 1500 idles, 1700 pushes along a thruster's axis, 1300
# against it. The front horizontals' axes point aft, so they reverse to push the vehicle forward.
IDLE = [1500] * 8
FORWARD = [1300, 1300, 1700, 1700, 1500, 1500, 1500, 1500]
GENTLY_FORWARD = [1460, 1460, 1540, 1540, 1500, 1500, 1500, 1500]
DOWN = [1500, 1500, 1500, 1500, 1700, 1700, 1700, 1700]
TURN_RIGHT = [1700, 1300, 1300, 1700, 1500, 1500, 1500, 1500]
# The whole velocity trajectory must stay this close to the reference's, as a fraction of the
# fastest the reference goes. The two agree to 0.13% at worst, the difference between their
# integrators. Sway's linear damping 3% out moves sway's decay 0.5% and fails.
TRAJECTORY_TOLERANCE = 0.0025
# A step response's steady speed doesn't depend on the integrator: the two agree on it to better
# than a millionth, and a quadratic damping coefficient 1% out moves it 0.5%. Its rise from 10% to
# 90% of that speed does: the reference's forward Euler lags by about a step, 0.7 to 0.9 ms here,
# whether the rise takes 88 ms or 3 s.
STEADY_SPEED_TOLERANCE = 0.001
RISE_TIME_TOLERANCE_S = 0.002


def simulate(velocity, pulses, seconds):
    """Run uuv_dynamics from a level start and return its body velocity at every step."""
    command = [
        os.environ['SIMULATE'], '--seconds', str(seconds), '--step', str(STEP_S),
        '--velocity', *map(str, velocity), '--pulses', *map(str, pulses)]
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

    ours = simulate(velocity, IDLE, seconds=10)
    reference = fossen_heavy.simulate(velocity, np.zeros(6), seconds=10, step=STEP_S)

    assert worst_difference(ours, reference) < TRAJECTORY_TOLERANCE


def steady_speed(speeds):
    """Return the speed the response settles at: its mean over the last second."""
    return speeds[-round(1 / STEP_S):].mean()


def has_settled(speeds):
    """Return whether the speed changed by under 0.1% of its steady value in the last second."""
    last_second = speeds[-round(1 / STEP_S):]
    return np.ptp(last_second) < 0.001 * abs(steady_speed(speeds))


def first_time_at(fraction, level):
    """Return the seconds at which a response first reaches a level, found between two steps."""
    after = np.argmax(fraction >= level)
    before = after - 1
    return (before + (level - fraction[before]) / (fraction[after] - fraction[before])) * STEP_S


def rise_time(speeds):
    """Return the seconds the response takes from 10% to 90% of its steady speed."""
    fraction = speeds / steady_speed(speeds)
    return first_time_at(fraction, 0.9) - first_time_at(fraction, 0.1)


# Surge is driven gently, with 2 N, and given a minute: the vehicle pitches up under forward
# thrust, and above about 5 N its surge speed keeps oscillating instead of settling.
@pytest.mark.parametrize('pulses, axis, seconds', [
    pytest.param(GENTLY_FORWARD, 0, 60, id='surge'),
    pytest.param(DOWN, 2, 10, id='heave'),
    pytest.param(TURN_RIGHT, 5, 10, id='yaw'),
])
def test_step_response_matches_fossens_simulator(pulses, axis, seconds):
    ours = simulate(AT_REST, pulses, seconds)[:, axis]
    reference = fossen_heavy.simulate(
        AT_REST, fossen_heavy.thrust(pulses), seconds, step=STEP_S)[:, axis]

    assert has_settled(reference)
    assert steady_speed(ours) == pytest.approx(steady_speed(reference), rel=STEADY_SPEED_TOLERANCE)
    assert rise_time(ours) == pytest.approx(rise_time(reference), abs=RISE_TIME_TOLERANCE_S)


def alone(thruster):
    """Return the pulse widths that run one thruster and idle the rest."""
    pulses = list(IDLE)
    pulses[thruster] = 1700
    return pulses


@pytest.mark.parametrize('pulses', [
    pytest.param(alone(0), id='front-right horizontal alone'),
    pytest.param(alone(1), id='front-left horizontal alone'),
    pytest.param(alone(2), id='rear-right horizontal alone'),
    pytest.param(alone(3), id='rear-left horizontal alone'),
    pytest.param(alone(4), id='front-right vertical alone'),
    pytest.param(alone(5), id='front-left vertical alone'),
    pytest.param(alone(6), id='rear-right vertical alone'),
    pytest.param(alone(7), id='rear-left vertical alone'),
    pytest.param(FORWARD, id='the horizontals pushing forward'),
])
def test_thrust_response_matches_fossens_simulator(pulses):
    ours = simulate(AT_REST, pulses, seconds=5)
    reference = fossen_heavy.simulate(AT_REST, fossen_heavy.thrust(pulses), seconds=5, step=STEP_S)

    assert worst_difference(ours, reference) < TRAJECTORY_TOLERANCE
