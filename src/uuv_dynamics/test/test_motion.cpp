#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <cmath>
#include <string>
#include <vector>

#include "uuv_dynamics/motion.hpp"
#include "uuv_dynamics/vehicle.hpp"

namespace uuv_dynamics
{
namespace
{

constexpr double kTolerance = 1e-9;

const Eigen::Vector3d kOrigin = Eigen::Vector3d::Zero();
const Vector6d kNoThrust = Vector6d::Zero();

Vector6d twist(double u, double v, double w, double p, double q, double r)
{
  Vector6d nu;
  nu << u, v, w, p, q, r;
  return nu;
}

Eigen::Quaterniond turn(double degrees, const Eigen::Vector3d & axis)
{
  return Eigen::Quaterniond(Eigen::AngleAxisd(degrees * EIGEN_PI / 180.0, axis));
}

// The Heavy trimmed neutral with both centres at the origin: nothing turns it or pulls it.
Vehicle balanced_heavy()
{
  Vehicle vehicle = bluerov2_heavy();
  vehicle.buoyancy = vehicle.mass * kGravity;
  vehicle.centre_of_gravity = Eigen::Vector3d::Zero();
  vehicle.centre_of_buoyancy = Eigen::Vector3d::Zero();
  return vehicle;
}

State run(const Vehicle & vehicle, State state, double seconds, double dt)
{
  for (int i = 0; i < std::lround(seconds / dt); ++i) {
    state = step(vehicle, state, kNoThrust, dt);
  }
  return state;
}

TEST(Motion, LinearlyDampedSurgeDecaysExponentially)
{
  // With linear drag alone, m_u u_dot = X_u u, so u(t) = u0 exp(X_u t / m_u), where m_u is the mass
  // plus the surge added mass. Steps this long put fourth order's error (about 1e-9) a hundred
  // times under a third-order method's, so the tolerance tells them apart.
  Vehicle vehicle = balanced_heavy();
  vehicle.quadratic_damping = Vector6d::Zero();
  const double seconds = 2.0;
  const double surge_mass = vehicle.mass - vehicle.added_mass(0);
  const double expected = std::exp(vehicle.linear_damping(0) * seconds / surge_mass);

  const State start{kOrigin, Eigen::Quaterniond::Identity(), twist(1, 0, 0, 0, 0, 0)};
  const State end = run(vehicle, start, seconds, 0.1);

  EXPECT_NEAR(end.nu(0), expected, 1e-8);
}

struct KinematicsCase
{
  std::string name;
  Eigen::Quaterniond attitude;
  Vector6d nu;
  Eigen::Vector3d expected_position;
  Eigen::Quaterniond expected_attitude;
};

TEST(Motion, BodyVelocityMovesTheVehicleInTheWorld)
{
  // Undamped, the vehicle keeps its velocity, so after a second it has moved by its body velocity
  // turned into the world frame, and turned by its body rates about its own axes.
  Vehicle vehicle = balanced_heavy();
  vehicle.linear_damping = Vector6d::Zero();
  vehicle.quadratic_damping = Vector6d::Zero();
  const Eigen::Vector3d y = Eigen::Vector3d::UnitY();
  const Eigen::Vector3d z = Eigen::Vector3d::UnitZ();
  const double cos30 = std::sqrt(0.75);
  const std::vector<KinematicsCase> cases = {
    {"surging while heading east moves east",
     turn(90, z),
     twist(1, 0, 0, 0, 0, 0),
     {0, 1, 0},
     turn(90, z)},
    {"surging nose up 30 degrees climbs",
     turn(30, y),
     twist(1, 0, 0, 0, 0, 0),
     {cos30, 0, -0.5},
     turn(30, y)},
    {"yawing nose up turns about the body's own down axis", turn(30, y), twist(0, 0, 0, 0, 0, 0.5),
     kOrigin, turn(30, y) * turn(0.5 * 180.0 / EIGEN_PI, z)},
  };

  for (const auto & c : cases) {
    const State end = run(vehicle, {kOrigin, c.attitude, c.nu}, 1.0, 0.01);

    EXPECT_LT((end.position - c.expected_position).norm(), kTolerance) << c.name;
    EXPECT_LT(end.attitude.angularDistance(c.expected_attitude), kTolerance) << c.name;
  }
}

}  // namespace
}  // namespace uuv_dynamics
