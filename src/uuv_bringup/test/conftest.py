"""PX4's software-in-the-loop simulation with its quadcopter, headless, for the tests."""

import os
from pathlib import Path
import signal
import subprocess
import time

import pytest

PX4_SITL = Path(__file__).resolve().parent.parent / 'scripts' / 'px4_sitl'
PX4_PARAM = Path(os.environ['PX4_BUILD']) / 'bin' / 'px4-param'
DEADLINE_S = 60.0  # for each line PX4 logs; it boots in about 10 s


class Px4:
    """A running PX4, read through its log and changed through its parameters."""

    def __init__(self, log):
        self.log = log

    def wait_for(self, text):
        deadline = time.monotonic() + DEADLINE_S
        while text not in self.log.read_text(errors='replace'):
            assert time.monotonic() < deadline, f'PX4 never logged "{text}"'
            time.sleep(0.1)

    def set_param(self, name, value):
        subprocess.run([PX4_PARAM, 'set', name, value], check=True)


@pytest.fixture
def px4(tmp_path):
    log = tmp_path / 'px4.log'
    with log.open('w') as out:
        process = subprocess.Popen(
            [PX4_SITL, 'gz_x500', '-d'], env=dict(os.environ, HEADLESS='1'),
            stdout=out, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        started = Px4(log)
        started.wait_for('Startup script returned successfully')
        yield started
    finally:
        # PX4 and the Gazebo server it started, which share its session.
        os.killpg(process.pid, signal.SIGTERM)
        process.wait()
