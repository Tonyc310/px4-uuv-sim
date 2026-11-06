#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <string>
#include <vector>

#include "uuv_dynamics/frames.hpp"

namespace uuv_dynamics
{
namespace
{

// The conversions only swap and negate components, so they're exact up to rounding.
constexpr double kTolerance = 1e-12;

const Eigen::Vector3d kX = Eigen::Vector3d::UnitX();
const Eigen::Vector3d kY = Eigen::Vector3d::UnitY();
const Eigen::Vector3d kZ = Eigen::Vector3d::UnitZ();

Eigen::Quaterniond turn(double degrees, const Eigen::Vector3d & axis)
{
  return Eigen::Quaterniond(Eigen::AngleAxisd(degrees * EIGEN_PI / 180.0, axis));
}

TEST(Frames, WorldVectorMapsFromEnuToNed)
{
  const Eigen::Vector3d east_north_up(1.0, 2.0, 3.0);
  const Eigen::Vector3d north_east_down(2.0, 1.0, -3.0);

  const Eigen::Vector3d result = ned_from_enu(east_north_up);

  EXPECT_LT((result - north_east_down).norm(), kTolerance) << result.transpose();
}

TEST(Frames, BodyTwistMapsFromFluToFrd)
{
  Vector6d forward_left_up;
  forward_left_up << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
  Vector6d forward_right_down;
  forward_right_down << 1.0, -2.0, -3.0, 4.0, -5.0, -6.0;

  const Vector6d result = frd_from_flu(forward_left_up);

  EXPECT_LT((result - forward_right_down).norm(), kTolerance) << result.transpose();
}

struct AttitudeCase
{
  std::string name;
  Eigen::Quaterniond enu_flu;
  Eigen::Quaterniond ned_frd;
};

TEST(Frames, AttitudeMapsFromEnuFluToNedFrd)
{
  // Heading turns about up in ENU and about down in NED. Pitch turns about left in FLU and about
  // right in FRD, so nose up is a negative pitch in one and a positive pitch in the other. Roll
  // turns about forward in both.
  const std::vector<AttitudeCase> cases = {
    {"level, heading north", turn(90, kZ), Eigen::Quaterniond::Identity()},
    {"level, heading east", Eigen::Quaterniond::Identity(), turn(90, kZ)},
    {"nose up 10 degrees, heading north", turn(90, kZ) * turn(-10, kY), turn(10, kY)},
    {"right side down 10 degrees, heading north", turn(90, kZ) * turn(10, kX), turn(10, kX)},
  };

  for (const auto & c : cases) {
    const Eigen::Quaterniond result = ned_frd_from_enu_flu(c.enu_flu);
    EXPECT_LT(result.angularDistance(c.ned_frd), kTolerance) << c.name;
  }
}

TEST(Frames, EachConversionRoundTrips)
{
  const Eigen::Vector3d vector(1.0, 2.0, 3.0);
  Vector6d twist;
  twist << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
  const Eigen::Quaterniond attitude = turn(30, kZ) * turn(-20, kY) * turn(10, kX);

  EXPECT_LT((enu_from_ned(ned_from_enu(vector)) - vector).norm(), kTolerance) << "world vector";
  EXPECT_LT((flu_from_frd(frd_from_flu(twist)) - twist).norm(), kTolerance) << "body twist";
  EXPECT_LT(
    enu_flu_from_ned_frd(ned_frd_from_enu_flu(attitude)).angularDistance(attitude), kTolerance)
    << "attitude";
}

}  // namespace
}  // namespace uuv_dynamics
