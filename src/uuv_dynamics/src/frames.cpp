#include "uuv_dynamics/frames.hpp"

namespace uuv_dynamics
{
namespace
{

// Both changes of axes are half turns. ENU to NED swaps east and north and negates up: a half turn
// about the axis midway between east and north. FLU to FRD negates left and up: a half turn about
// forward.
const Eigen::Quaterniond kNedFromEnu(
  Eigen::AngleAxisd(EIGEN_PI, Eigen::Vector3d(1.0, 1.0, 0.0).normalized()));
const Eigen::Quaterniond kFrdFromFlu(Eigen::AngleAxisd(EIGEN_PI, Eigen::Vector3d::UnitX()));

Vector6d rotated(const Eigen::Quaterniond & rotation, const Vector6d & twist)
{
  Vector6d result;
  result << rotation * twist.head<3>(), rotation * twist.tail<3>();
  return result;
}

}  // namespace

Eigen::Vector3d ned_from_enu(const Eigen::Vector3d & enu) { return kNedFromEnu * enu; }

Eigen::Vector3d enu_from_ned(const Eigen::Vector3d & ned) { return kNedFromEnu.conjugate() * ned; }

Vector6d frd_from_flu(const Vector6d & flu) { return rotated(kFrdFromFlu, flu); }

Vector6d flu_from_frd(const Vector6d & frd) { return rotated(kFrdFromFlu.conjugate(), frd); }

Eigen::Quaterniond ned_frd_from_enu_flu(const Eigen::Quaterniond & enu_flu)
{
  return kNedFromEnu * enu_flu * kFrdFromFlu.conjugate();
}

Eigen::Quaterniond enu_flu_from_ned_frd(const Eigen::Quaterniond & ned_frd)
{
  return kNedFromEnu.conjugate() * ned_frd * kFrdFromFlu;
}

}  // namespace uuv_dynamics
