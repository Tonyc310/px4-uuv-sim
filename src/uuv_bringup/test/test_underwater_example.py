"""Gazebo's underwater example runs, and its vehicle moves under its thruster's command."""

import gz.math7  # noqa: F401  Lets the bindings return poses.
from gz.msgs10.double_pb2 import Double
from gz.sim8 import Link, Model, TestFixture, World, world_entity
from gz.transport13 import Node

# Installed with Gazebo: MBARI's Tethys LRAUV with buoyancy, hydrodynamics and a thruster.
WORLD = 'auv_controls.sdf'
THRUST_TOPIC = '/model/tethys/joint/propeller_joint/cmd_thrust'
FORWARD_THRUST_N = -31.0  # the example's own command
STEPS = 5000  # 5 s of the world's 1 ms steps


def test_moves_under_thrust():
    node = Node()
    thruster = node.advertise(THRUST_TOPIC, Double)
    fixture = TestFixture(WORLD)
    positions = []

    # Gazebo's transport delivers the command asynchronously, so it goes out before every step.
    def command(info, ecm):
        thruster.publish(Double(data=FORWARD_THRUST_N))

    def record(info, ecm):
        model = Model(World(world_entity(ecm)).model_by_name(ecm, 'tethys'))
        positions.append(Link(model.canonical_link(ecm)).world_pose(ecm).pos())

    fixture.on_pre_update(command)
    fixture.on_post_update(record)
    fixture.finalize()
    fixture.server().run(True, STEPS, False)

    assert len(positions) == STEPS
    assert positions[0].distance(positions[-1]) > 1.0  # 2.4 m when written
