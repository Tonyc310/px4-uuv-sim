#ifndef UUV_DYNAMICS__TYPES_HPP_
#define UUV_DYNAMICS__TYPES_HPP_

#include <Eigen/Core>

namespace uuv_dynamics
{

// A body twist (linear velocity, then angular) or wrench (force, then moment).
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

}  // namespace uuv_dynamics

#endif  // UUV_DYNAMICS__TYPES_HPP_
