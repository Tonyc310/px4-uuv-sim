#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <cmath>
#include <cstddef>
#include <vector>

#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/TestFixture.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "uuv_dynamics/motion.hpp"
#include "uuv_dynamics/vehicle.hpp"

namespace uuv_validation
{
namespace
{

constexpr double kStepSeconds = 0.001;  // the world's step
constexpr int kSteps = 10000;           // 10 s
constexpr int kStepsPerSecond = 1000;
constexpr double kSurgeForce = 10.0;  // N, through the body origin

// The two agree on the steady speed to a few billionths and on the 10-90% rise time to 0.3 ms.
// A quadratic damping coefficient 1% out moves the steady speed 0.4%, and surge's added mass 3%
// out moves the rise time 16 ms.
constexpr double kSteadySpeedTolerance = 0.001;  // as a fraction of the steady speed
constexpr double kRiseTimeToleranceSeconds = 0.002;

// The body's surge speed after each step in Gazebo, pushed forward by a constant force, with
// Gazebo's own Hydrodynamics system as the water.
std::vector<double> gazebo_surge_speeds()
{
  std::vector<double> speeds;
  gz::sim::Link link;
  gz::sim::TestFixture fixture(GAZEBO_HYDRODYNAMICS_WORLD);
  fixture
    .OnPreUpdate([&](const gz::sim::UpdateInfo & info, gz::sim::EntityComponentManager & ecm) {
      if (info.iterations == 1) {
        const gz::sim::Model model(
          ecm.EntityByComponents(gz::sim::components::Model(), gz::sim::components::Name("heavy")));
        link = gz::sim::Link(model.LinkByName(ecm, "base_link"));
        link.EnableVelocityChecks(ecm);
      }
      link.AddWorldWrench(ecm, {kSurgeForce, 0, 0}, {0, 0, 0});
    })
    .OnPostUpdate([&](const gz::sim::UpdateInfo &, const gz::sim::EntityComponentManager & ecm) {
      // The body never turns, so the world's x is its surge.
      speeds.push_back(link.WorldLinearVelocity(ecm).value().X());
    })
    .Finalize();
  fixture.Server()->Run(true, kSteps, false);
  return speeds;
}

// The same push on uuv_dynamics' model of the same body: no weight or buoyancy to speak of, and
// the force through the centre of gravity, as in the Gazebo world.
std::vector<double> library_surge_speeds()
{
  uuv_dynamics::Vehicle body = uuv_dynamics::bluerov2_heavy();
  body.buoyancy = body.mass * uuv_dynamics::kGravity;
  body.centre_of_gravity = Eigen::Vector3d::Zero();
  uuv_dynamics::Vector6d tau = uuv_dynamics::Vector6d::Zero();
  tau(0) = kSurgeForce;
  uuv_dynamics::State state{
    Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), uuv_dynamics::Vector6d::Zero()};
  std::vector<double> speeds;
  for (int i = 0; i < kSteps; ++i) {
    state = uuv_dynamics::step(body, state, tau, kStepSeconds);
    speeds.push_back(state.nu(0));
  }
  return speeds;
}

// The speed a response settles at: its mean over the last second.
double steady_speed(const std::vector<double> & speeds)
{
  double sum = 0.0;
  for (std::size_t i = speeds.size() - kStepsPerSecond; i < speeds.size(); ++i) {
    sum += speeds[i];
  }
  return sum / kStepsPerSecond;
}

// The seconds at which a response first reaches a fraction of its steady speed, found between two
// steps.
double first_time_at(const std::vector<double> & speeds, double fraction)
{
  const double level = fraction * steady_speed(speeds);
  std::size_t after = 1;
  while (speeds[after] < level) {
    ++after;
  }
  const double within_step = (level - speeds[after - 1]) / (speeds[after] - speeds[after - 1]);
  // speeds[i] is the speed after i + 1 steps.
  return (static_cast<double>(after) + within_step) * kStepSeconds;
}

double rise_time(const std::vector<double> & speeds)
{
  return first_time_at(speeds, 0.9) - first_time_at(speeds, 0.1);
}

TEST(GazeboHydrodynamics, SurgeStepResponseMatchesGazebosOwnSystem)
{
  const std::vector<double> gazebo = gazebo_surge_speeds();
  const std::vector<double> library = library_surge_speeds();

  EXPECT_NEAR(
    steady_speed(library), steady_speed(gazebo), kSteadySpeedTolerance * steady_speed(gazebo));
  EXPECT_NEAR(rise_time(library), rise_time(gazebo), kRiseTimeToleranceSeconds);
}

}  // namespace
}  // namespace uuv_validation
