#include <Eigen/Geometry>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "gz/common/Console.hh"
#include "gz/msgs/actuators.pb.h"
#include "gz/plugin/Register.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/System.hh"
#include "gz/transport/Node.hh"
#include "uuv_dynamics/dynamics.hpp"
#include "uuv_dynamics/frames.hpp"
#include "uuv_dynamics/thrusters.hpp"
#include "uuv_dynamics/vehicle.hpp"

namespace uuv_gazebo
{
namespace
{

Eigen::Vector3d eigen(const gz::math::Vector3d & v) { return {v.X(), v.Y(), v.Z()}; }

Eigen::Quaterniond eigen(const gz::math::Quaterniond & q) { return {q.W(), q.X(), q.Y(), q.Z()}; }

gz::math::Vector3d gazebo(const Eigen::Vector3d & v) { return {v.x(), v.y(), v.z()}; }

}  // namespace

// Gives a link the BlueROV2 Heavy's water physics and thrusters from uuv_dynamics. Gazebo moves the
// rigid body and applies its weight; this applies thrust, buoyancy and the water's forces, each
// step, at the link's origin, which must be the vehicle's centre of buoyancy. The world's gravity
// must be uuv_dynamics::kGravity. Thruster commands arrive as PX4's Gazebo bridge sends them: one
// pulse width in us per thruster, on /<model name>/command/motor_speed. Until the first one, the
// thrusters are idle.
class FossenDynamics : public gz::sim::System,
                       public gz::sim::ISystemConfigure,
                       public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity & entity, const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm, gz::sim::EventManager &) override
  {
    const std::string name = sdf->Get<std::string>("link_name", "base_link").first;
    const gz::sim::Model model(entity);
    link_ = gz::sim::Link(model.LinkByName(ecm, name));
    if (!link_.Valid(ecm)) {
      gzerr << "FossenDynamics: the model has no link named [" << name << "]\n";
      return;
    }
    link_.EnableVelocityChecks(ecm);
    const std::string topic = "/" + model.Name(ecm) + "/command/motor_speed";
    if (!node_.Subscribe(topic, &FossenDynamics::OnCommand, this)) {
      gzerr << "FossenDynamics: can't subscribe to [" << topic << "]\n";
    }
  }

  void PreUpdate(const gz::sim::UpdateInfo & info, gz::sim::EntityComponentManager & ecm) override
  {
    const auto pose = link_.WorldPose(ecm);
    const auto linear = link_.WorldLinearVelocity(ecm);
    const auto angular = link_.WorldAngularVelocity(ecm);
    // Velocities appear after the first step.
    if (info.paused || !pose || !linear || !angular) {
      return;
    }
    // Gazebo's world frame is ENU and the link's frame FLU; its velocities are in the world frame,
    // and the linear one is the link origin's.
    const Eigen::Quaterniond enu_flu = eigen(pose->Rot());
    uuv_dynamics::Vector6d flu;
    flu << enu_flu.conjugate() * eigen(*linear), enu_flu.conjugate() * eigen(*angular);
    const uuv_dynamics::Vector6d wrench = uuv_dynamics::flu_from_frd(uuv_dynamics::external_wrench(
      vehicle_, uuv_dynamics::frd_from_flu(flu), uuv_dynamics::ned_frd_from_enu_flu(enu_flu),
      uuv_dynamics::thrust_wrench(thrusters_, thrusts())));
    link_.AddWorldWrench(
      ecm, gazebo(enu_flu * wrench.head<3>()), gazebo(enu_flu * wrench.tail<3>()));
  }

private:
  // Called on Gazebo's transport thread. A malformed command is dropped, and the last good one
  // stands.
  void OnCommand(const gz::msgs::Actuators & command)
  {
    const auto thrusts = uuv_dynamics::heavy_thrusts(
      std::vector<double>(command.velocity().begin(), command.velocity().end()));
    if (!thrusts) {
      gzwarn << "FossenDynamics: dropped a command that isn't one finite pulse width for each of "
             << uuv_dynamics::kHeavyThrusterCount << " thrusters\n";
      return;
    }
    const std::lock_guard<std::mutex> lock(mutex_);
    thrusts_ = *thrusts;
  }

  uuv_dynamics::HeavyThrusts thrusts()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return thrusts_;
  }

  gz::sim::Link link_;
  const uuv_dynamics::Vehicle vehicle_ = uuv_dynamics::bluerov2_heavy();
  const uuv_dynamics::HeavyThrusters thrusters_ = uuv_dynamics::bluerov2_heavy_thrusters();
  std::mutex mutex_;
  uuv_dynamics::HeavyThrusts thrusts_{};
  // Last, so it's destroyed first: it stops the callbacks before what they use goes.
  gz::transport::Node node_;
};

}  // namespace uuv_gazebo

GZ_ADD_PLUGIN(
  uuv_gazebo::FossenDynamics, gz::sim::System, uuv_gazebo::FossenDynamics::ISystemConfigure,
  uuv_gazebo::FossenDynamics::ISystemPreUpdate)
