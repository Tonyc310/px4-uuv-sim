#include <gtest/gtest.h>

#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <cmath>
#include <string>
#include <vector>

#include "uuv_dynamics/dynamics.hpp"
#include "uuv_dynamics/vehicle.hpp"

namespace uuv_dynamics
{
namespace
{

constexpr double kTolerance = 1e-9;

const Eigen::Quaterniond kLevel = Eigen::Quaterniond::Identity();
const Vector6d kAtRest = Vector6d::Zero();
const Vector6d kNoThrust = Vector6d::Zero();

Vector6d twist(double u, double v, double w, double p, double q, double r)
{
  Vector6d nu;
  nu << u, v, w, p, q, r;
  return nu;
}

// The Heavy with its buoyancy trimmed to its weight, so one effect can be tested without the
// vehicle's net lift.
Vehicle neutral_heavy()
{
  Vehicle vehicle = bluerov2_heavy();
  vehicle.buoyancy = vehicle.mass * kGravity;
  return vehicle;
}

Matrix6d total_mass(const Vehicle & vehicle)
{
  return rigid_body_mass(vehicle) + added_mass(vehicle);
}

int sign_of(double value) { return (value > kTolerance) - (value < -kTolerance); }

struct VelocityCase
{
  std::string name;
  Vector6d nu;
};

TEST(Dynamics, MassMatrixIsSymmetricPositiveDefinite)
{
  const Matrix6d mass = total_mass(bluerov2_heavy());

  EXPECT_LT((mass - mass.transpose()).norm(), kTolerance) << "not symmetric";
  EXPECT_EQ(mass.llt().info(), Eigen::Success) << "not positive definite";
}

TEST(Dynamics, CoriolisForcesDoNoWork)
{
  const Matrix6d mass = total_mass(bluerov2_heavy());
  const std::vector<VelocityCase> cases = {
    {"surge", twist(1.0, 0.0, 0.0, 0.0, 0.0, 0.0)},
    {"yaw", twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.5)},
    {"turning while surging", twist(1.0, 0.0, 0.0, 0.0, 0.0, 0.5)},
    {"all six at once", twist(0.5, -0.3, 0.2, 0.1, -0.2, 0.3)},
  };

  for (const auto & c : cases) {
    const double power = c.nu.dot(coriolis(mass, c.nu) * c.nu);
    EXPECT_NEAR(power, 0.0, kTolerance) << c.name;
  }
}

TEST(Dynamics, TurningVehicleSkidsOutwardAsCoriolisPredicts)
{
  // Seen from a vehicle turning right, its straight-ahead motion swings to the left:
  // v_dot = -(m - X_udot) u r / (m - Y_vdot), the rigid body's v_dot = -u r weighted by the water
  // each axis carries. The centre of gravity at the origin keeps roll out of the sway equation.
  Vehicle vehicle = neutral_heavy();
  vehicle.centre_of_gravity = Eigen::Vector3d::Zero();
  const double u = 1.0;
  const double r = 0.5;
  const double expected =
    -(vehicle.mass - vehicle.added_mass(0)) * u * r / (vehicle.mass - vehicle.added_mass(1));

  const Vector6d result = acceleration(vehicle, twist(u, 0.0, 0.0, 0.0, 0.0, r), kLevel, kNoThrust);

  EXPECT_NEAR(result(1), expected, kTolerance);
}

TEST(Dynamics, DampingTakesEnergyOut)
{
  const Vehicle heavy = bluerov2_heavy();
  const std::vector<VelocityCase> cases = {
    {"surge forward", twist(1.0, 0.0, 0.0, 0.0, 0.0, 0.0)},
    {"surge backward", twist(-1.0, 0.0, 0.0, 0.0, 0.0, 0.0)},
    {"sway right", twist(0.0, 1.0, 0.0, 0.0, 0.0, 0.0)},
    {"sway left", twist(0.0, -1.0, 0.0, 0.0, 0.0, 0.0)},
    {"heave down", twist(0.0, 0.0, 1.0, 0.0, 0.0, 0.0)},
    {"heave up", twist(0.0, 0.0, -1.0, 0.0, 0.0, 0.0)},
    {"roll right", twist(0.0, 0.0, 0.0, 1.0, 0.0, 0.0)},
    {"roll left", twist(0.0, 0.0, 0.0, -1.0, 0.0, 0.0)},
    {"pitch up", twist(0.0, 0.0, 0.0, 0.0, 1.0, 0.0)},
    {"pitch down", twist(0.0, 0.0, 0.0, 0.0, -1.0, 0.0)},
    {"yaw right", twist(0.0, 0.0, 0.0, 0.0, 0.0, 1.0)},
    {"yaw left", twist(0.0, 0.0, 0.0, 0.0, 0.0, -1.0)},
  };

  for (const auto & c : cases) {
    const double power = c.nu.dot(damping(heavy, c.nu) * c.nu);
    EXPECT_GT(power, 0.0) << c.name;
  }
}

TEST(Dynamics, LevelNeutralVehicleFeelsNoRestoringForce)
{
  const Vector6d result = restoring(neutral_heavy(), kLevel);

  EXPECT_LT(result.norm(), kTolerance) << result.transpose();
}

struct BuoyancyCase
{
  std::string name;
  double net_buoyancy;  // N, buoyancy less weight
};

TEST(Dynamics, NetBuoyancySetsTheVerticalAcceleration)
{
  // Down is positive, and the net force moves the vehicle and the water it carries in heave.
  const std::vector<BuoyancyCase> cases = {
    {"2 N buoyant rises", 2.0},
    {"2 N heavy sinks", -2.0},
  };

  for (const auto & c : cases) {
    Vehicle vehicle = bluerov2_heavy();
    vehicle.buoyancy = vehicle.mass * kGravity + c.net_buoyancy;
    const double expected = -c.net_buoyancy / (vehicle.mass - vehicle.added_mass(2));

    const Vector6d result = acceleration(vehicle, kAtRest, kLevel, kNoThrust);

    EXPECT_NEAR(result(2), expected, kTolerance) << c.name;
  }
}

struct TiltCase
{
  std::string name;
  Vehicle vehicle;
  int axis;  // 0 roll, 1 pitch
  double degrees;
};

TEST(Dynamics, TiltedVehicleIsTurnedBackToLevel)
{
  // The centre of gravity hangs below the centre of buoyancy like a keel, so weight and buoyancy
  // form a couple against any roll or pitch, whichever of the two centres is off the origin.
  Vehicle buoyancy_above_origin = neutral_heavy();
  buoyancy_above_origin.centre_of_gravity = Eigen::Vector3d::Zero();
  buoyancy_above_origin.centre_of_buoyancy = {0.0, 0.0, -0.02};
  const std::vector<TiltCase> cases = {
    {"right side down", neutral_heavy(), 0, 10.0},
    {"left side down", neutral_heavy(), 0, -10.0},
    {"nose up", neutral_heavy(), 1, 10.0},
    {"nose down", neutral_heavy(), 1, -10.0},
    {"right side down, buoyancy above the origin", buoyancy_above_origin, 0, 10.0},
    {"nose up, buoyancy above the origin", buoyancy_above_origin, 1, 10.0},
  };

  for (const auto & c : cases) {
    const Eigen::Quaterniond tilted(
      Eigen::AngleAxisd(c.degrees * EIGEN_PI / 180.0, Eigen::Vector3d::Unit(c.axis)));

    const double angular = acceleration(c.vehicle, kAtRest, tilted, kNoThrust)(3 + c.axis);

    EXPECT_EQ(sign_of(angular), -sign_of(c.degrees)) << c.name << ": " << angular;
  }
}

TEST(Dynamics, SurgeThrustPitchesTheNoseDown)
{
  // Thrust at the origin pushes above the centre of gravity, so it pitches the nose down as it
  // drives the vehicle forward. Surge and pitch couple through m z_g; solving the two equations:
  // q_dot = -m z_g T / ((m - X_udot)(I_y - M_qdot) - (m z_g)^2).
  const Vehicle vehicle = neutral_heavy();
  const double thrust = 40.0;
  const double coupling = vehicle.mass * vehicle.centre_of_gravity.z();
  const double surge_mass = vehicle.mass - vehicle.added_mass(0);
  const double pitch_inertia = vehicle.inertia.y() - vehicle.added_mass(4);
  const double expected = -coupling * thrust / (surge_mass * pitch_inertia - coupling * coupling);

  const Vector6d tau = twist(thrust, 0.0, 0.0, 0.0, 0.0, 0.0);
  const Vector6d result = acceleration(vehicle, kAtRest, kLevel, tau);

  EXPECT_NEAR(result(4), expected, kTolerance);
}

struct SurgeCase
{
  std::string name;
  double fraction_of_steady_speed;
  int expected_sign;
};

TEST(Dynamics, SurgeSettlesWhereThrustEqualsDrag)
{
  // Drag in surge is -(X_u u + X_u|u| u |u|); this is the speed where it equals the thrust, 1.377
  // m/s for 40 N with Wu's coefficients.
  const Vehicle vehicle = neutral_heavy();
  const double thrust = 40.0;
  const double linear = -vehicle.linear_damping(0);
  const double quadratic = -vehicle.quadratic_damping(0);
  const double steady_speed =
    (-linear + std::sqrt(linear * linear + 4.0 * quadratic * thrust)) / (2.0 * quadratic);
  const std::vector<SurgeCase> cases = {
    {"below the steady speed it speeds up", 0.9, 1},
    {"at the steady speed it holds", 1.0, 0},
    {"above the steady speed it slows", 1.1, -1},
  };

  for (const auto & c : cases) {
    const Vector6d nu = twist(c.fraction_of_steady_speed * steady_speed, 0.0, 0.0, 0.0, 0.0, 0.0);
    const Vector6d tau = twist(thrust, 0.0, 0.0, 0.0, 0.0, 0.0);

    const Vector6d result = acceleration(vehicle, nu, kLevel, tau);

    EXPECT_EQ(sign_of(result(0)), c.expected_sign) << c.name << ": " << result(0);
  }
}

struct StateCase
{
  std::string name;
  Vector6d nu;
  Eigen::Quaterniond attitude;
  Vector6d tau;
};

TEST(Dynamics, ExternalWrenchCompletesARigidBodySimulator)
{
  // A rigid-body simulator moves the body by M_RB nu_dot + C_RB(nu) nu = the wrenches applied to
  // it, and applies the weight at the centre of gravity itself. With the external wrench added, it
  // must accelerate as Fossen's equations say.
  const Vehicle heavy = bluerov2_heavy();
  const Eigen::Quaterniond tilted = Eigen::Quaterniond(
    Eigen::AngleAxisd(0.35, Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitY()));
  const std::vector<StateCase> cases = {
    {"at rest, level", kAtRest, kLevel, kNoThrust},
    {"at rest, tilted", kAtRest, tilted, kNoThrust},
    {"moving in all six and thrusting, tilted", twist(0.5, -0.3, 0.2, 0.1, -0.2, 0.3), tilted,
     twist(10.0, -5.0, 3.0, 1.0, -0.5, 0.8)},
  };

  for (const auto & c : cases) {
    const Matrix6d rigid = rigid_body_mass(heavy);
    const Eigen::Vector3d weight =
      heavy.mass * kGravity * (c.attitude.conjugate() * Eigen::Vector3d::UnitZ());
    Vector6d weight_wrench;
    weight_wrench << weight, heavy.centre_of_gravity.cross(weight);
    const Vector6d nu_dot = acceleration(heavy, c.nu, c.attitude, c.tau);
    const Vector6d expected = rigid * nu_dot + coriolis(rigid, c.nu) * c.nu - weight_wrench;

    const Vector6d result = external_wrench(heavy, c.nu, c.attitude, c.tau);

    EXPECT_LT((result - expected).norm(), kTolerance) << c.name << ": " << result.transpose();
  }
}

}  // namespace
}  // namespace uuv_dynamics
