#include "control/oneLinkRobustControl.hpp"

namespace robot
{
// One link robust control //
void oneLinkRobustControl::execute(sharedOneLinkRobot& robot)
{
  // Calculate and save the error //
  ScalarF e = robot->thetaF - robot->theta_d;
  ScalarF de = robot->dthetaF - robot->dtheta_d;

  robot->e = e;
  robot->de = de;

  // Calculate passivity terms //
  ScalarF v = calculateV(robot, mLambda);
  ScalarF a = calculateA(robot, mLambda);
  ScalarF r = calculateR(robot, mLambda);

  // Calculate regressor matrix //
  Matrix1x2F Y = oneLinkRegressor(robot, v, a);

  // Update the estimated parameters //

  Vector2x1F F = Y.transpose()*r;
  float normF = std::sqrt(F(0)*F(0) + F(1)*F(1));

  Vector2x1F delp = normF > mEpsilon ? -mRho*Y.transpose()*r/normF : -(mRho/mEpsilon)*Y.transpose()*r;

  // Calculate the motor control torque for each link //
  robot->u = robot->motorGearRatio.inverse()*(Y*(robot->parameters + delp) - mK*r);
}

} // namespace robot
