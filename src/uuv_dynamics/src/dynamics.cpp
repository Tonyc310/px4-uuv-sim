#include "uuv_dynamics/dynamics.hpp"

#include <Eigen/Cholesky>

namespace uuv_dynamics
{
namespace
{

// S(a) b = a x b.
Eigen::Matrix3d skew(const Eigen::Vector3d & a)
{
  Eigen::Matrix3d s;
  s << 0.0, -a.z(), a.y(), a.z(), 0.0, -a.x(), -a.y(), a.x(), 0.0;
  return s;
}

// The share of g(eta) from one vertical force acting at a point on the body; down is the world's
// down seen in the body frame.
Vector6d restoring_from(
  double downward_force, const Eigen::Vector3d & at, const Eigen::Vector3d & down)
{
  const Eigen::Vector3d force = downward_force * down;
  Vector6d g;
  g << -force, -at.cross(force);
  return g;
}

Eigen::Vector3d world_down(const Eigen::Quaterniond & attitude)
{
  return attitude.conjugate() * Eigen::Vector3d::UnitZ();
}

Vector6d buoyancy_restoring(const Vehicle & vehicle, const Eigen::Quaterniond & attitude)
{
  return restoring_from(-vehicle.buoyancy, vehicle.centre_of_buoyancy, world_down(attitude));
}

}  // namespace

Matrix6d rigid_body_mass(const Vehicle & vehicle)
{
  // A centre of gravity off the origin couples linear and angular motion.
  const Eigen::Matrix3d coupling = vehicle.mass * skew(vehicle.centre_of_gravity);
  Matrix6d mass;
  mass << vehicle.mass * Eigen::Matrix3d::Identity(), -coupling, coupling,
    Eigen::Matrix3d(vehicle.inertia.asDiagonal());
  return mass;
}

Matrix6d added_mass(const Vehicle & vehicle)
{
  return Matrix6d((-vehicle.added_mass).asDiagonal());
}

Matrix6d coriolis(const Matrix6d & inertia, const Vector6d & nu)
{
  // From the inertia matrix as Fossen computes it (Sagatun and Fossen's theorem; m2c() in his
  // Python Vehicle Simulator): skew-symmetric for any velocity, and with the centre-of-gravity
  // terms that Wu's simplified matrix (eq. 4.40) leaves out.
  const Eigen::Vector3d linear_momentum = inertia.topRows<3>() * nu;
  const Eigen::Vector3d angular_momentum = inertia.bottomRows<3>() * nu;
  Matrix6d c = Matrix6d::Zero();
  c.topRightCorner<3, 3>() = -skew(linear_momentum);
  c.bottomLeftCorner<3, 3>() = -skew(linear_momentum);
  c.bottomRightCorner<3, 3>() = -skew(angular_momentum);
  return c;
}

Matrix6d damping(const Vehicle & vehicle, const Vector6d & nu)
{
  // |nu|, not nu: quadratic drag opposes the motion in either direction.
  const Vector6d coefficients =
    -(vehicle.linear_damping + vehicle.quadratic_damping.cwiseProduct(nu.cwiseAbs()));
  return Matrix6d(coefficients.asDiagonal());
}

Vector6d restoring(const Vehicle & vehicle, const Eigen::Quaterniond & attitude)
{
  // Weight acts down and buoyancy up in the world frame, at their own centres on the body.
  return restoring_from(vehicle.mass * kGravity, vehicle.centre_of_gravity, world_down(attitude)) +
         buoyancy_restoring(vehicle, attitude);
}

Vector6d acceleration(
  const Vehicle & vehicle, const Vector6d & nu, const Eigen::Quaterniond & attitude,
  const Vector6d & tau)
{
  const Matrix6d mass = rigid_body_mass(vehicle) + added_mass(vehicle);
  const Vector6d net =
    tau - coriolis(mass, nu) * nu - damping(vehicle, nu) * nu - restoring(vehicle, attitude);
  return mass.ldlt().solve(net);
}

Vector6d external_wrench(
  const Vehicle & vehicle, const Vector6d & nu, const Eigen::Quaterniond & attitude,
  const Vector6d & tau)
{
  // Fossen's equation less the simulator's share: M_RB, C_RB and the weight. The added-mass force
  // uses the acceleration that solves the full equations now, not the simulator's last one: with
  // more added mass than mass, as the Heavy has in heave and sway, feeding back the last step's
  // acceleration multiplies its error by more than one each step, and the simulation diverges.
  const Matrix6d added = added_mass(vehicle);
  return tau - buoyancy_restoring(vehicle, attitude) - damping(vehicle, nu) * nu -
         coriolis(added, nu) * nu - added * acceleration(vehicle, nu, attitude, tau);
}

}  // namespace uuv_dynamics
