#include "uuv_dynamics/motion.hpp"

#include "uuv_dynamics/dynamics.hpp"

namespace uuv_dynamics
{
namespace
{

// The state's rate of change. The attitude's is a quaternion's four coefficients, not a rotation.
struct Rates
{
  Eigen::Vector3d velocity;
  Eigen::Vector4d attitude;
  Vector6d acceleration;
};

Rates rates_of(const Vehicle & vehicle, const State & state, const Vector6d & tau)
{
  // The body's velocity seen in the world frame moves its origin; q_dot = q (0, omega) / 2 turns it
  // about its own axes, since omega is in the body frame.
  const Eigen::Quaterniond omega(0.0, state.nu(3), state.nu(4), state.nu(5));
  return {
    state.attitude * state.nu.head<3>(), 0.5 * (state.attitude * omega).coeffs(),
    acceleration(vehicle, state.nu, state.attitude, tau)};
}

State advanced(const State & state, const Rates & rates, double dt)
{
  State next{
    state.position + dt * rates.velocity, state.attitude, state.nu + dt * rates.acceleration};
  next.attitude.coeffs() += dt * rates.attitude;
  // Each stage evaluates the dynamics at its own attitude, which must be a rotation.
  next.attitude.normalize();
  return next;
}

Rates weighted(const Rates & k1, const Rates & k2, const Rates & k3, const Rates & k4)
{
  return {
    (k1.velocity + 2.0 * k2.velocity + 2.0 * k3.velocity + k4.velocity) / 6.0,
    (k1.attitude + 2.0 * k2.attitude + 2.0 * k3.attitude + k4.attitude) / 6.0,
    (k1.acceleration + 2.0 * k2.acceleration + 2.0 * k3.acceleration + k4.acceleration) / 6.0};
}

}  // namespace

State step(const Vehicle & vehicle, const State & state, const Vector6d & tau, double dt)
{
  const Rates k1 = rates_of(vehicle, state, tau);
  const Rates k2 = rates_of(vehicle, advanced(state, k1, dt / 2.0), tau);
  const Rates k3 = rates_of(vehicle, advanced(state, k2, dt / 2.0), tau);
  const Rates k4 = rates_of(vehicle, advanced(state, k3, dt), tau);
  return advanced(state, weighted(k1, k2, k3, k4), dt);
}

}  // namespace uuv_dynamics
