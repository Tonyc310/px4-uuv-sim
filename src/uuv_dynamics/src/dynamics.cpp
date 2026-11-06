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
  const Eigen::Vector3d down = attitude.conjugate() * Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d weight = vehicle.mass * kGravity * down;
  const Eigen::Vector3d buoyancy = -vehicle.buoyancy * down;
  Vector6d g;
  g << -(weight + buoyancy),
    -(vehicle.centre_of_gravity.cross(weight) + vehicle.centre_of_buoyancy.cross(buoyancy));
  return g;
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

}  // namespace uuv_dynamics
