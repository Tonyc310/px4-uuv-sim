#ifndef UUV_DYNAMICS__FRAMES_HPP_
#define UUV_DYNAMICS__FRAMES_HPP_

#include <Eigen/Geometry>

#include "uuv_dynamics/types.hpp"

// Gazebo and ROS use east-north-up world and forward-left-up body frames (REP 103); Fossen's
// equations and PX4 use north-east-down and forward-right-down.
namespace uuv_dynamics
{

Eigen::Vector3d ned_from_enu(const Eigen::Vector3d & enu);
Eigen::Vector3d enu_from_ned(const Eigen::Vector3d & ned);

Vector6d frd_from_flu(const Vector6d & flu);
Vector6d flu_from_frd(const Vector6d & frd);

// An attitude is the rotation from the body frame to the world frame.
Eigen::Quaterniond ned_frd_from_enu_flu(const Eigen::Quaterniond & enu_flu);
Eigen::Quaterniond enu_flu_from_ned_frd(const Eigen::Quaterniond & ned_frd);

}  // namespace uuv_dynamics

#endif  // UUV_DYNAMICS__FRAMES_HPP_
