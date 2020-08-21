#include "control/pdControl.hpp"

namespace robot
{
// One link robust control //
template <typename Matrix, typename Robot>
void PDControl<Matrix, Robot>::execute(Robot& robot)
{
  // Calculate and save the error //
  robot->e = robot->thetaF - robot->theta_d;
  robot->de = robot->dthetaF - robot->dtheta_d;

  // Calculate the motor control torque for each link //
  robot->u = robot->motorGearRatio.inverse()*(mKp*robot->e + mKd*robot->de);
}

} // namespace robot
