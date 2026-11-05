"""PX4's state reaches ROS 2 through the Micro XRCE-DDS Agent."""

import subprocess
import time

from px4_msgs.msg import VehicleStatus
import pytest
import rclpy
from rclpy.qos import qos_profile_sensor_data

# PX4's client in SITL looks for the agent on this UDP port. The topic's name carries its version.
AGENT_PORT = '8888'
VEHICLE_STATUS = '/fmu/out/vehicle_status_v1'
DEADLINE_S = 30.0


@pytest.fixture
def agent():
    process = subprocess.Popen(
        ['MicroXRCEAgent', 'udp4', '-p', AGENT_PORT],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    yield process
    process.terminate()
    process.wait()


def test_vehicle_status_reaches_ros2(px4, agent):
    rclpy.init()
    try:
        node = rclpy.create_node('bridge_test')
        received = []
        # PX4 publishes best effort, which a reliable subscription wouldn't hear.
        node.create_subscription(
            VehicleStatus, VEHICLE_STATUS, received.append, qos_profile_sensor_data)
        deadline = time.monotonic() + DEADLINE_S
        while not received:
            assert time.monotonic() < deadline, f'nothing arrived on {VEHICLE_STATUS}'
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
    finally:
        rclpy.shutdown()
