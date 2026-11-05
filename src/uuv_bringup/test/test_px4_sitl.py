"""PX4's software-in-the-loop simulation boots with its quadcopter in Gazebo, ready for takeoff."""

import os
from pathlib import Path
import signal
import subprocess
import time

PX4_SITL = Path(__file__).resolve().parent.parent / 'scripts' / 'px4_sitl'
PX4_PARAM = Path(os.environ['PX4_BUILD']) / 'bin' / 'px4-param'
DEADLINE_S = 60.0  # for each line PX4 logs; it boots in about 10 s


def wait_for(log, text):
    deadline = time.monotonic() + DEADLINE_S
    while text not in log.read_text(errors='replace'):
        assert time.monotonic() < deadline, f'PX4 never logged "{text}"'
        time.sleep(0.1)


def test_quadcopter_is_ready_for_takeoff(tmp_path):
    log = tmp_path / 'px4.log'
    with log.open('w') as out:
        px4 = subprocess.Popen(
            [PX4_SITL, 'gz_x500', '-d'], env=dict(os.environ, HEADLESS='1'),
            stdout=out, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        wait_for(log, 'Startup script returned successfully')
        # The airframe makes arming wait for a ground station's link, which a test doesn't have.
        # PX4's environment override can't set it: 0 is the default, which the airframe replaces.
        subprocess.run([PX4_PARAM, 'set', 'NAV_DLL_ACT', '0'], check=True)
        wait_for(log, 'Ready for takeoff!')
    finally:
        # PX4 and the Gazebo server it started, which share its session.
        os.killpg(px4.pid, signal.SIGTERM)
        px4.wait()
