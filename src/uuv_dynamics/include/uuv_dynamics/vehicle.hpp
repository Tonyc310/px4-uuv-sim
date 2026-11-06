#ifndef UUV_DYNAMICS__VEHICLE_HPP_
#define UUV_DYNAMICS__VEHICLE_HPP_

#include <Eigen/Core>

#include "uuv_dynamics/types.hpp"

namespace uuv_dynamics
{

constexpr double kGravity = 9.81;  // m/s^2

// A vehicle's parameters in Fossen's body frame (forward-right-down), about the body origin. The
// hydrodynamic derivatives keep the literature's SNAME names and signs: negative where the water
// resists the motion.
struct Vehicle
{
  double mass;                         // kg
  double buoyancy;                     // N, fully submerged
  Eigen::Vector3d centre_of_gravity;   // m, from the body origin
  Eigen::Vector3d centre_of_buoyancy;  // m, from the body origin
  Eigen::Vector3d inertia;             // kg m^2, moments about the body axes through the origin
  Vector6d added_mass;         // X_udot, Y_vdot, Z_wdot (kg); K_pdot, M_qdot, N_rdot (kg m^2/rad)
  Vector6d linear_damping;     // X_u, Y_v, Z_w (N s/m); K_p, M_q, N_r (N m s/rad)
  Vector6d quadratic_damping;  // X_u|u|, Y_v|v|, Z_w|w| (N s^2/m^2); K_p|p|, ... (N m s^2/rad^2)
};

// The BlueROV2 Heavy's parameters: literature estimates, not identified on the vehicle.
Vehicle bluerov2_heavy();

}  // namespace uuv_dynamics

#endif  // UUV_DYNAMICS__VEHICLE_HPP_
