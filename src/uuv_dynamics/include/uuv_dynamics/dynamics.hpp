#ifndef UUV_DYNAMICS__DYNAMICS_HPP_
#define UUV_DYNAMICS__DYNAMICS_HPP_

#include <Eigen/Geometry>

#include "uuv_dynamics/types.hpp"
#include "uuv_dynamics/vehicle.hpp"

// Fossen's equations of motion for a marine craft, M nu_dot + C(nu) nu + D(nu) nu + g(eta) = tau
// (T. I. Fossen, Handbook of Marine Craft Hydrodynamics and Motion Control, 2nd ed., Wiley, 2021).
// All in the body frame, forward-right-down, about the body origin: nu is (u, v, w) in m/s then
// (p, q, r) in rad/s, and tau is a force in N then a moment in N m.
namespace uuv_dynamics
{

Matrix6d rigid_body_mass(const Vehicle & vehicle);

// The water the vehicle drags along when it accelerates.
Matrix6d added_mass(const Vehicle & vehicle);

// Coriolis and centripetal forces of a body with the given inertia matrix, moving at nu.
Matrix6d coriolis(const Matrix6d & inertia, const Vector6d & nu);

Matrix6d damping(const Vehicle & vehicle, const Vector6d & nu);

// Weight and buoyancy. attitude: the rotation from the body frame (FRD) to the world frame (NED).
Vector6d restoring(const Vehicle & vehicle, const Eigen::Quaterniond & attitude);

// nu_dot: the equations of motion solved for the acceleration.
Vector6d acceleration(
  const Vehicle & vehicle, const Vector6d & nu, const Eigen::Quaterniond & attitude,
  const Vector6d & tau);

// The wrench, about the body origin, that a rigid-body simulator applying the weight itself must
// add for the vehicle to move as these equations say: thrust, buoyancy and the water's forces.
Vector6d external_wrench(
  const Vehicle & vehicle, const Vector6d & nu, const Eigen::Quaterniond & attitude,
  const Vector6d & tau);

}  // namespace uuv_dynamics

#endif  // UUV_DYNAMICS__DYNAMICS_HPP_
