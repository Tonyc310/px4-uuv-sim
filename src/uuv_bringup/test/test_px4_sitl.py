"""PX4's software-in-the-loop simulation boots with its quadcopter in Gazebo, ready for takeoff."""


def test_quadcopter_is_ready_for_takeoff(px4):
    # The airframe makes arming wait for a ground station's link, which a test doesn't have.
    # PX4's environment override can't set it: 0 is the default, which the airframe replaces.
    px4.set_param('NAV_DLL_ACT', '0')
    px4.wait_for('Ready for takeoff!')
