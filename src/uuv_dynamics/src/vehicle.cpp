#include "uuv_dynamics/vehicle.hpp"

namespace uuv_dynamics
{

// C.-J. Wu, "6-DoF Modelling and Control of a Remotely Operated Vehicle", Master's thesis, Flinders
// University, 2018. Wu estimated these from published data on the earlier BlueROV and the Heavy's
// specifications; the tank tests he planned weren't run. For lack of data, roll and pitch take
// yaw's inertia, added mass and damping, and the centre of gravity's 2 cm below the centre of
// buoyancy is his "representative value" (p. 47).
Vehicle bluerov2_heavy()
{
  Vehicle heavy;
  heavy.mass = 11.5;  // Table 5.1, p. 48
  // Table 5.1, p. 48. Its weight, 112.8 N, is the mass times kGravity, so it isn't kept separately.
  heavy.buoyancy = 114.8;
  heavy.centre_of_buoyancy = {0.0, 0.0, 0.0};  // Table 5.1, p. 48: the body origin
  heavy.centre_of_gravity = {0.0, 0.0, 0.02};  // Table 5.1, p. 48: below the centre of buoyancy
  // Table 5.1, p. 48. About the body axes, as Wu's rigid-body mass matrix uses them (eq. 4.38,
  // p. 35).
  heavy.inertia = {0.16, 0.16, 0.16};
  heavy.added_mass << -5.5, -12.7, -14.57, -0.12, -0.12, -0.12;            // Table 5.2, p. 48
  heavy.linear_damping << -4.03, -6.22, -5.18, -0.07, -0.07, -0.07;        // Table 5.3, p. 48
  heavy.quadratic_damping << -18.18, -21.66, -36.99, -1.55, -1.55, -1.55;  // Table 5.3, p. 48
  return heavy;
}

}  // namespace uuv_dynamics
