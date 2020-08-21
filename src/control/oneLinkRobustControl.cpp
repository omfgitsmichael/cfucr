#include "control/oneLinkRobustControl.hpp"

namespace robot
{
// One link robust control //
void oneLinkRobustControl::execute(sharedOneLinkRobot& robot)
{
  // Calculate and save the error //
  robot->e = robot->thetaF - robot->theta_d;
  robot->de = robot->dthetaF - robot->dtheta_d;

  // Calculate passivity terms //
  ScalarF v = calculateV<sharedOneLinkRobot, ScalarF, ScalarF>(robot, mLambda);
  ScalarF a = calculateA<sharedOneLinkRobot, ScalarF, ScalarF>(robot, mLambda);
  ScalarF r = calculateR<sharedOneLinkRobot, ScalarF, ScalarF>(robot, mLambda);

  // Calculate regressor matrix //
  Matrix1x2F Y = oneLinkRegressor(robot, v, a);

  // Update the estimated parameters //

  Vector2x1F F = Y.transpose()*r;
  float normF = std::sqrt(F(0)*F(0) + F(1)*F(1));

  Vector2x1F delp = normF > mEpsilon ? (-mRho/normF)*F : (-mRho/mEpsilon)*F;

  // Calculate the motor control torque for each link //
  robot->u = robot->motorGearRatio.inverse()*(Y*(robot->parameters + delp) - mK*r);
}

} // namespace robot
