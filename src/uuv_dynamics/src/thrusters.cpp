#include "uuv_dynamics/thrusters.hpp"

#include <algorithm>
#include <cmath>

namespace uuv_dynamics
{
namespace
{

// Blue Robotics, "T200 Public Performance Data 10-20V, September 2019", sheet "16 V", column
// "Force (Kg f)": thrust in kilograms-force at each pulse width from 1100 to 1900 us, every 4 us.
// https://cad.bluerobotics.com/T200-Public-Performance-Data-10-20V-September-2019.xlsx
// (sha256 1399b76df7a176cfbb08760c11731199053336e7d92182dece11824c73285085).
constexpr double kFirstPulseUs = 1100.0;
constexpr double kPulseStepUs = 4.0;
constexpr std::array<double, 201> kThrustKgf = {
  -4.069476, -4.050577, -4.023361, -3.964394, -3.900891, -3.869140, -3.823781, -3.801101, -3.751206,
  -3.714918, -3.655952, -3.587913, -3.519874, -3.451835, -3.397404, -3.311222, -3.252255, -3.202360,
  -3.107105, -3.039066, -2.993707, -2.939276, -2.862166, -2.821342, -2.771447, -2.707944, -2.658049,
  -2.580938, -2.549187, -2.508364, -2.453933, -2.381358, -2.354142, -2.277032, -2.236209, -2.195385,
  -2.118275, -2.081987, -2.023020, -1.977661, -1.941374, -1.859727, -1.814368, -1.764473, -1.700970,
  -1.669219, -1.610252, -1.555821, -1.514997, -1.487782, -1.437887, -1.397063, -1.347168, -1.301809,
  -1.256450, -1.202019, -1.161196, -1.120372, -1.102229, -1.052333, -1.020582, -0.979759, -0.943471,
  -0.902648, -0.870897, -0.821002, -0.784714, -0.743891, -0.716675, -0.680388, -0.648637, -0.616885,
  -0.580598, -0.539774, -0.512559, -0.480808, -0.444520, -0.417305, -0.385553, -0.353802, -0.322050,
  -0.294835, -0.258547, -0.235868, -0.208652, -0.181437, -0.149685, -0.127006, -0.104326, -0.086182,
  -0.068039, -0.049895, -0.036287, 0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,
  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,
  0.040823,  0.054431,  0.077111,  0.099790,  0.127006,  0.154221,  0.181437,  0.217724,  0.249476,
  0.290299,  0.322050,  0.358338,  0.399161,  0.435448,  0.471736,  0.512559,  0.557918,  0.603277,
  0.639565,  0.680388,  0.721211,  0.775642,  0.816466,  0.870897,  0.907184,  0.948007,  0.993366,
  1.034190,  1.097693,  1.143052,  1.183875,  1.238306,  1.279129,  1.329025,  1.387992,  1.437887,
  1.483246,  1.537677,  1.587572,  1.651075,  1.691898,  1.759937,  1.823440,  1.877871,  1.932302,
  1.991269,  2.045700,  2.122811,  2.177242,  2.218065,  2.277032,  2.381358,  2.431253,  2.517436,
  2.576403,  2.653513,  2.730624,  2.757839,  2.839486,  2.889381,  2.975564,  3.052674,  3.107105,
  3.170608,  3.215967,  3.302150,  3.374724,  3.420084,  3.510802,  3.606056,  3.683167,  3.737598,
  3.819245,  3.887283,  3.955322,  4.059648,  4.145831,  4.254693,  4.304588,  4.381699,  4.508704,
  4.526848,  4.649318,  4.708285,  4.785396,  4.835291,  4.930545,  5.012192,  5.075694,  5.143733,
  5.175485,  5.220844,  5.245035,
};

constexpr double kNewtonsPerKgf = 9.80665;  // standard gravity, by the unit's definition

}  // namespace

double t200_thrust(double pulse_width_us)
{
  if (pulse_width_us <= 0.0) {
    return 0.0;
  }
  const double last = static_cast<double>(kThrustKgf.size() - 1);
  const double position = std::clamp((pulse_width_us - kFirstPulseUs) / kPulseStepUs, 0.0, last);
  const auto below = static_cast<std::size_t>(std::min(std::floor(position), last - 1.0));
  const double fraction = position - static_cast<double>(below);
  return kNewtonsPerKgf *
         (kThrustKgf[below] + fraction * (kThrustKgf[below + 1] - kThrustKgf[below]));
}

HeavyThrusters bluerov2_heavy_thrusters()
{
  // Positions: C.-J. Wu (2018), Table 4.2, p. 40, from Blue Robotics' Heavy retrofit drawings,
  // taken from the body origin. Order and thrust directions: PX4's BlueROV2 Heavy airframe
  // (60002), so PX4's motor i drives thruster i. The horizontal thrusters point 45 degrees off the
  // vehicle's axis; the vertical ones push down.
  const double h = std::sqrt(0.5);
  return {{
    {{0.156, 0.111, 0.085}, {-h, h, 0.0}},
    {{0.156, -0.111, 0.085}, {-h, -h, 0.0}},
    {{-0.156, 0.111, 0.085}, {h, h, 0.0}},
    {{-0.156, -0.111, 0.085}, {h, -h, 0.0}},
    {{0.120, 0.218, 0.0}, {0.0, 0.0, 1.0}},
    {{0.120, -0.218, 0.0}, {0.0, 0.0, 1.0}},
    {{-0.120, 0.218, 0.0}, {0.0, 0.0, 1.0}},
    {{-0.120, -0.218, 0.0}, {0.0, 0.0, 1.0}},
  }};
}

std::optional<HeavyThrusts> heavy_thrusts(const std::vector<double> & pulse_widths_us)
{
  if (pulse_widths_us.size() != kHeavyThrusterCount) {
    return std::nullopt;
  }
  HeavyThrusts thrusts;
  for (std::size_t i = 0; i < kHeavyThrusterCount; ++i) {
    if (!std::isfinite(pulse_widths_us[i])) {
      return std::nullopt;
    }
    thrusts[i] = t200_thrust(pulse_widths_us[i]);
  }
  return thrusts;
}

Vector6d thrust_wrench(const HeavyThrusters & thrusters, const HeavyThrusts & thrusts)
{
  Vector6d tau = Vector6d::Zero();
  for (std::size_t i = 0; i < thrusters.size(); ++i) {
    const Eigen::Vector3d force = thrusts[i] * thrusters[i].axis;
    tau.head<3>() += force;
    tau.tail<3>() += thrusters[i].position.cross(force);
  }
  return tau;
}

}  // namespace uuv_dynamics
