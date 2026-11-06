#ifndef UUV_DYNAMICS__MOTION_HPP_
#define UUV_DYNAMICS__MOTION_HPP_

#include <Eigen/Geometry>

#include "uuv_dynamics/types.hpp"
#include "uuv_dynamics/vehicle.hpp"

namespace uuv_dynamics
{

// A vehicle's state in Fossen's frames.
struct State
{
  Eigen::Vector3d position;     // m, of the body origin in the world frame (NED)
  Eigen::Quaterniond attitude;  // the rotation from the body frame (FRD) to the world frame
  Vector6d nu;                  // the body's velocity in the body frame, as in dynamics.hpp
};

// The state dt seconds later (fourth-order Runge-Kutta), with tau held over the step.
State step(const Vehicle & vehicle, const State & state, const Vector6d & tau, double dt);

}  // namespace uuv_dynamics

#endif  // UUV_DYNAMICS__MOTION_HPP_
