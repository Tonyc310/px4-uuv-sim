#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <optional>
#include <string>
#include <vector>

#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/TestFixture.hh"
#include "gz/sim/components/AngularVelocityCmd.hh"
#include "gz/sim/components/LinearVelocityCmd.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/PoseCmd.hh"
#include "uuv_dynamics/frames.hpp"
#include "uuv_dynamics/motion.hpp"
#include "uuv_dynamics/vehicle.hpp"

namespace uuv_gazebo
{
namespace
{

using uuv_dynamics::State;
using uuv_dynamics::Vector6d;

constexpr double kStep = 0.001;  // s, the world's step
constexpr int kSteps = 2000;     // 2 s
// The test's setup takes two steps: one to set the starting state, one to let go of it.
constexpr int kSetupSteps = 2;

constexpr double kPositionTolerance = 0.01;                  // m
constexpr double kAttitudeTolerance = 0.5 * EIGEN_PI / 180;  // rad
constexpr double kVelocityTolerance = 0.01;  // as a fraction of the case's largest speed

Eigen::Vector3d eigen(const gz::math::Vector3d & v) { return {v.X(), v.Y(), v.Z()}; }

gz::math::Vector3d gazebo(const Eigen::Vector3d & v) { return {v.x(), v.y(), v.z()}; }

Eigen::Quaterniond eigen(const gz::math::Quaterniond & q) { return {q.W(), q.X(), q.Y(), q.Z()}; }

// Gazebo's state of the link in uuv_dynamics' terms. Written here, not shared with the plugin, so a
// conversion error in one can't hide in the other.
State state_of(const gz::sim::Link & link, const gz::sim::EntityComponentManager & ecm)
{
  const gz::math::Pose3d pose = link.WorldPose(ecm).value();
  const Eigen::Quaterniond enu_flu = eigen(pose.Rot());
  Vector6d flu;
  flu << enu_flu.conjugate() * eigen(link.WorldLinearVelocity(ecm).value()),
    enu_flu.conjugate() * eigen(link.WorldAngularVelocity(ecm).value());
  return {
    uuv_dynamics::ned_from_enu(eigen(pose.Pos())), uuv_dynamics::ned_frd_from_enu_flu(enu_flu),
    uuv_dynamics::frd_from_flu(flu)};
}

struct StartCase
{
  std::string name;
  Eigen::Quaterniond enu_flu;  // attitude in Gazebo's frames
  Vector6d flu_velocity;       // body velocity in Gazebo's body frame
};

Vector6d twist(double u, double v, double w, double p, double q, double r)
{
  Vector6d nu;
  nu << u, v, w, p, q, r;
  return nu;
}

// Runs Gazebo from the case's starting state and returns its state after setup and kSteps later.
std::pair<State, State> run_gazebo(const StartCase & c)
{
  std::optional<State> start;
  std::optional<State> end;
  gz::sim::Model model;
  gz::sim::Link link;
  gz::sim::TestFixture fixture(WATER_WORLD);
  fixture
    .OnPreUpdate([&](const gz::sim::UpdateInfo & info, gz::sim::EntityComponentManager & ecm) {
      if (info.iterations == 1) {
        model = gz::sim::Model(ecm.EntityByComponents(
          gz::sim::components::Model(), gz::sim::components::Name("bluerov2_heavy")));
        link = gz::sim::Link(model.LinkByName(ecm, "base_link"));
        link.EnableVelocityChecks(ecm);
        model.SetWorldPoseCmd(
          ecm, gz::math::Pose3d(
                 {0, 0, 0}, {c.enu_flu.w(), c.enu_flu.x(), c.enu_flu.y(), c.enu_flu.z()}));
        link.SetLinearVelocity(ecm, gazebo(c.flu_velocity.head<3>()));
        link.SetAngularVelocity(ecm, gazebo(c.flu_velocity.tail<3>()));
      } else if (info.iterations == 2) {
        // Gazebo applies these commands every step until they're removed.
        ecm.RemoveComponent<gz::sim::components::WorldPoseCmd>(model.Entity());
        ecm.RemoveComponent<gz::sim::components::LinearVelocityCmd>(link.Entity());
        ecm.RemoveComponent<gz::sim::components::AngularVelocityCmd>(link.Entity());
      }
    })
    .OnPostUpdate(
      [&](const gz::sim::UpdateInfo & info, const gz::sim::EntityComponentManager & ecm) {
        if (info.iterations == kSetupSteps) {
          start = state_of(link, ecm);
        } else if (info.iterations == kSetupSteps + kSteps) {
          end = state_of(link, ecm);
        }
      })
    .Finalize();
  fixture.Server()->Run(true, kSetupSteps + kSteps, false);
  return {start.value(), end.value()};
}

TEST(FossenDynamics, GazeboFollowsTheLibrary)
{
  // Each start exercises a different part of the plugin's boundary with Gazebo: buoyancy against
  // Gazebo's weight, the attitude and the righting moment, linear velocity and drag, angular
  // velocity and drag. The last two start turned away from the world's axes, so a velocity or a
  // torque left in the wrong frame acts along the wrong axis.
  const auto turn = [](double degrees, const Eigen::Vector3d & axis) {
    return Eigen::Quaterniond(Eigen::AngleAxisd(degrees * EIGEN_PI / 180, axis));
  };
  const std::vector<StartCase> cases = {
    {"rising from rest", Eigen::Quaterniond::Identity(), twist(0, 0, 0, 0, 0, 0)},
    {"righting from 20 degrees nose up", turn(-20, Eigen::Vector3d::UnitY()),
     twist(0, 0, 0, 0, 0, 0)},
    {"surge slowing from 1 m/s, heading 60 degrees off east", turn(60, Eigen::Vector3d::UnitZ()),
     twist(1.0, 0, 0, 0, 0, 0)},
    {"yaw slowing from 0.5 rad/s, rolled 20 degrees", turn(20, Eigen::Vector3d::UnitX()),
     twist(0, 0, 0, 0, 0, 0.5)},
  };
  const uuv_dynamics::Vehicle heavy = uuv_dynamics::bluerov2_heavy();

  for (const auto & c : cases) {
    const auto [start, gazebo_end] = run_gazebo(c);
    State expected = start;
    for (int i = 0; i < kSteps; ++i) {
      expected = uuv_dynamics::step(heavy, expected, Vector6d::Zero(), kStep);
    }
    const double scale = std::max(start.nu.norm(), expected.nu.norm());

    EXPECT_LT((gazebo_end.position - expected.position).norm(), kPositionTolerance) << c.name;
    EXPECT_LT(gazebo_end.attitude.angularDistance(expected.attitude), kAttitudeTolerance) << c.name;
    EXPECT_LT((gazebo_end.nu - expected.nu).norm(), kVelocityTolerance * scale)
      << c.name << ": Gazebo " << gazebo_end.nu.transpose() << ", library "
      << expected.nu.transpose();
  }
}

}  // namespace
}  // namespace uuv_gazebo
