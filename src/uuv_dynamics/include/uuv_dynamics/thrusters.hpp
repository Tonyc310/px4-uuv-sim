#ifndef UUV_DYNAMICS__THRUSTERS_HPP_
#define UUV_DYNAMICS__THRUSTERS_HPP_

#include <Eigen/Core>
#include <array>
#include <cstddef>
#include <optional>
#include <vector>

#include "uuv_dynamics/types.hpp"

namespace uuv_dynamics
{

// A Blue Robotics T200's thrust at 16 V for a command pulse width in microseconds, in N, from Blue
// Robotics' measurements. Positive pushes along the thruster's axis. Pulse widths outside
// 1100-1900 us give the nearer end's thrust, except no pulse at all (0, as PX4 sends while
// disarmed), on which the ESC stops the thruster. The pulse width must be finite.
double t200_thrust(double pulse_width_us);

struct Thruster
{
  Eigen::Vector3d position;  // m, from the body origin, in the body frame (FRD)
  Eigen::Vector3d axis;      // unit vector: the way positive thrust pushes the vehicle
};

constexpr std::size_t kHeavyThrusterCount = 8;
using HeavyThrusters = std::array<Thruster, kHeavyThrusterCount>;
using HeavyThrusts = std::array<double, kHeavyThrusterCount>;  // N, each along its thruster's axis

// The BlueROV2 Heavy's thrusters, in the order of PX4's motors 1-8 for this airframe.
HeavyThrusters bluerov2_heavy_thrusters();

// Each thruster's T200 thrust for a command of one pulse width per thruster, in thruster order;
// none if the command has the wrong count or a value that isn't a finite number.
std::optional<HeavyThrusts> heavy_thrusts(const std::vector<double> & pulse_widths_us);

// tau: the thrusts' force, and their moment about the body origin.
Vector6d thrust_wrench(const HeavyThrusters & thrusters, const HeavyThrusts & thrusts);

}  // namespace uuv_dynamics

#endif  // UUV_DYNAMICS__THRUSTERS_HPP_
