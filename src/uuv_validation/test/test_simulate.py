"""The simulate program says so when it can't do what it was asked."""

import os
import subprocess

import pytest

USAGE_ERROR = 2
RUN = ['--seconds', '1', '--step', '0.01']


@pytest.mark.parametrize('arguments', [
    pytest.param([], id='no arguments'),
    pytest.param(['--seconds', '1'], id='no step'),
    pytest.param(['--seconds', '1', '--step', '0'], id='a step of zero'),
    pytest.param(['--seconds', 'long', '--step', '0.01'], id='a word where a number belongs'),
    pytest.param(['--seconds', '1', '--step', '0.3'], id='a step that does not divide the time'),
    pytest.param(RUN + ['--velocity', '1', '0', '0'], id='three velocities'),
    pytest.param(RUN + ['--pulses'] + ['1500'] * 7, id='seven pulse widths'),
    pytest.param(RUN + ['--seconds', '2'], id='an option given twice'),
    pytest.param(RUN + ['--depth', '5'], id='an option it does not have'),
])
def test_unusable_arguments_are_refused(arguments):
    result = subprocess.run([os.environ['SIMULATE'], *arguments], capture_output=True, text=True)

    assert result.returncode == USAGE_ERROR
    assert result.stdout == ''
    assert 'usage' in result.stderr


def test_a_trajectory_that_cannot_be_written_is_reported():
    # Linux's /dev/full refuses every write, as a full disk would.
    with open('/dev/full', 'w') as full:
        result = subprocess.run([os.environ['SIMULATE'], *RUN], stdout=full)

    assert result.returncode not in (0, USAGE_ERROR)
