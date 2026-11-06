#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "uuv_dynamics/thrusters.hpp"

namespace uuv_dynamics
{
namespace
{

constexpr double kTolerance = 1e-9;

// Blue Robotics publishes thrust in kilograms-force.
double kgf(double value) { return value * 9.80665; }

Vector6d six(double a, double b, double c, double d, double e, double f)
{
  Vector6d v;
  v << a, b, c, d, e, f;
  return v;
}

struct PulseCase
{
  std::string name;
  double pulse_width_us;
  double expected;  // N
};

TEST(Thrusters, T200ThrustFollowsBlueRoboticsData)
{
  // From the "16 V" sheet of Blue Robotics' T200 performance data, measured every 4 us.
  const std::vector<PulseCase> cases = {
    {"full reverse", 1100, kgf(-4.069476)},
    {"the last point before the dead band", 1468, kgf(-0.036287)},
    {"the dead band's lower edge", 1472, 0.0},
    {"neutral", 1500, 0.0},
    {"the dead band's upper edge", 1528, 0.0},
    {"the first point after the dead band", 1532, kgf(0.040823)},
    {"a measured point", 1700, kgf(1.823440)},
    {"halfway between two measured points", 1702, kgf((1.823440 + 1.877871) / 2)},
    {"full forward", 1900, kgf(5.245035)},
    {"below the range holds full reverse", 1000, kgf(-4.069476)},
    {"above the range holds full forward", 2000, kgf(5.245035)},
    {"no pulse at all stops the thruster", 0, 0.0},
  };

  for (const auto & c : cases) {
    EXPECT_NEAR(t200_thrust(c.pulse_width_us), c.expected, kTolerance) << c.name;
  }
}

TEST(Thrusters, EachThrusterGetsItsOwnCommandsThrust)
{
  const std::vector<double> pulses = {1100, 1200, 1300, 1400, 1600, 1700, 1800, 1900};

  const auto thrusts = heavy_thrusts(pulses);

  ASSERT_TRUE(thrusts.has_value());
  for (std::size_t i = 0; i < pulses.size(); ++i) {
    EXPECT_EQ((*thrusts)[i], t200_thrust(pulses[i])) << "thruster " << i;
  }
}

struct CommandCase
{
  std::string name;
  std::vector<double> pulses;
};

TEST(Thrusters, MalformedCommandsGiveNoThrusts)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  const std::vector<CommandCase> cases = {
    {"seven values", {1500, 1500, 1500, 1500, 1500, 1500, 1500}},
    {"nine values", {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500}},
    {"a value that isn't a number", {1500, 1500, 1500, nan, 1500, 1500, 1500, 1500}},
    {"an infinite value", {1500, 1500, 1500, 1500, 1500, 1500, 1500, infinity}},
  };

  for (const auto & c : cases) {
    EXPECT_FALSE(heavy_thrusts(c.pulses).has_value()) << c.name;
  }
}

struct WrenchCase
{
  std::string name;
  HeavyThrusts thrusts;
  Vector6d expected;
};

TEST(Thrusters, ThrustsCombineIntoForceAndMoment)
{
  // Each thrust pushes along its axis at its thruster: the force, plus r x f about the origin. The
  // horizontal thrusters sit at the corners (156 mm forward or aft, 111 mm to the side) 85 mm below
  // the origin, angled 45 degrees; positive thrust on the front pair pushes aft and outward.
  const double f = 10.0;                // N
  const double d = f / std::sqrt(2.0);  // a horizontal thrust's forward and sideways parts
  const std::vector<WrenchCase> cases = {
    {"the front-right horizontal alone",
     {f, 0, 0, 0, 0, 0, 0, 0},
     six(-d, d, 0, -0.085 * d, -0.085 * d, (0.156 + 0.111) * d)},
    {"the horizontals pushing forward pitch the nose up",
     {-f, -f, f, f, 0, 0, 0, 0},
     six(4 * d, 0, 0, 0, 0.085 * 4 * d, 0)},
    {"the rear-left horizontal alone",
     {0, 0, 0, f, 0, 0, 0, 0},
     six(d, -d, 0, 0.085 * d, 0.085 * d, (0.156 + 0.111) * d)},
    {"the verticals equally push straight down",
     {0, 0, 0, 0, f, f, f, f},
     six(0, 0, 4 * f, 0, 0, 0)},
    {"the right verticals down and the left up roll right side down",
     {0, 0, 0, 0, f, -f, f, -f},
     six(0, 0, 0, 4 * 0.218 * f, 0, 0)},
    {"the front verticals down and the rear up pitch the nose down",
     {0, 0, 0, 0, f, f, -f, -f},
     six(0, 0, 0, 0, -4 * 0.120 * f, 0)},
  };

  for (const auto & c : cases) {
    const Vector6d result = thrust_wrench(bluerov2_heavy_thrusters(), c.thrusts);

    EXPECT_LT((result - c.expected).norm(), kTolerance) << c.name << ": " << result.transpose();
  }
}

}  // namespace
}  // namespace uuv_dynamics
