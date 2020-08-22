#include "control/oneLinkControl.hpp"

namespace robot
{
// One link adaptive control //
void OneLinkControl::executeAdaptiveControl(sharedOneLinkRobot& robot)
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
  Vector2x1F F = -mGamma.inverse()*Y.transpose()*r;
  Vector2x1F p = robot->parameters + F*mDelt;
  robot->parameters = p;

  // Calculate the motor control torque for each link //
  robot->u = robot->motorGearRatio.inverse()*(Y*p - mK*r);
}

// One link robust control //
void OneLinkControl::executeRobustControl(sharedOneLinkRobot& robot)
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

// One link robust control //
void OneLinkControl::executePDControl(sharedOneLinkRobot& robot)
{
  // Calculate and save the error //
  robot->e = robot->thetaF - robot->theta_d;
  robot->de = robot->dthetaF - robot->dtheta_d;

  // Calculate the motor control torque for each link //
  robot->u = robot->motorGearRatio.inverse()*(mKp*robot->e + mKd*robot->de);
}
} // namespace robot


