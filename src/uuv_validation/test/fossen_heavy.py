"""
The BlueROV2 Heavy built on Fossen's Python Vehicle Simulator, as a reference for uuv_dynamics.

It shares nothing with the C++ on purpose. Wu's values are typed here again, the Coriolis matrix
and the restoring forces are Fossen's own functions, the attitude is in Euler angles where the C++
uses a quaternion, and the integration is the forward Euler his simulator's vehicles use where the
C++ uses Runge-Kutta. Everything is in Fossen's frames: north-east-down, and forward-right-down.
"""

import numpy as np
from python_vehicle_simulator.lib.gnc import attitudeEuler, gvect, m2c, Smtrx

# C.-J. Wu, "6-DoF Modelling and Control of a Remotely Operated Vehicle", Flinders University,
# 2018, Tables 5.1 to 5.3, p. 48.
MASS = 11.5  # kg
WEIGHT = MASS * 9.81  # N
BUOYANCY = 114.8  # N
CENTRE_OF_GRAVITY = np.array([0.0, 0.0, 0.02])  # m, from the body origin
CENTRE_OF_BUOYANCY = np.zeros(3)
INERTIA = np.diag([0.16, 0.16, 0.16])  # kg m^2, about the body origin's axes
ADDED_MASS = np.array([-5.5, -12.7, -14.57, -0.12, -0.12, -0.12])
LINEAR_DAMPING = np.array([-4.03, -6.22, -5.18, -0.07, -0.07, -0.07])
QUADRATIC_DAMPING = np.array([-18.18, -21.66, -36.99, -1.55, -1.55, -1.55])

_COUPLING = MASS * Smtrx(CENTRE_OF_GRAVITY)
_RIGID_BODY_MASS = np.block([[MASS * np.eye(3), -_COUPLING], [_COUPLING, INERTIA]])
_TOTAL_MASS = _RIGID_BODY_MASS + np.diag(-ADDED_MASS)
_TOTAL_MASS_INVERSE = np.linalg.inv(_TOTAL_MASS)


def _acceleration(eta, nu, tau):
    coriolis = m2c(_TOTAL_MASS, nu)
    damping = np.diag(-LINEAR_DAMPING - QUADRATIC_DAMPING * np.abs(nu))
    restoring = gvect(
        WEIGHT, BUOYANCY, eta[4], eta[3], CENTRE_OF_GRAVITY, CENTRE_OF_BUOYANCY)
    return _TOTAL_MASS_INVERSE @ (tau - coriolis @ nu - damping @ nu - restoring)


def simulate(velocity, tau, seconds, step):
    """Return the body velocity at every step from a level start, one row per step."""
    eta = np.zeros(6)
    nu = np.array(velocity, dtype=float)
    velocities = [nu.copy()]
    for _ in range(round(seconds / step)):
        # The order Fossen's vehicles use: the velocity first, then the pose from the new velocity.
        nu = nu + step * _acceleration(eta, nu, np.asarray(tau, dtype=float))
        eta = attitudeEuler(eta, nu, step)
        velocities.append(nu.copy())
    return np.array(velocities)
