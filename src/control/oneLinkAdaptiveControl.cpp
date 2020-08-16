#include <cmath>

#include "controller/oneLinkAdaptiveControl.hpp"

namespace robot
{
// One Link Adaptive Control //
void oneLinkAdaptiveControl::execute(sharedOneLinkRobot& robot)
{
  // Calculate and Save the Error //
  ScalarF e = robot->thetaF - robot->theta_d;
  ScalarF de = robot->dthetaF - robot->dtheta_d;

  robot->e = e;
  robot->de = de;

  // Calculate Passivity Terms //
  ScalarF v = calculateOneLinkV(robot);
  ScalarF a = calculateOneLinkA(robot);
  ScalarF r = calculateOneLinkR(robot);

  // Calculate Regressor Matrix //
  Matrix1x2F Y = oneLinkRegressor(robot, v, a);

  // Update The Estimated Parameters //
  Vector2x1F F = -mGamma.inverse()*Y.transpose()*r;
  Vector2x1F p = robot->parameters + F*mDelt;
  robot->parameters = p;

  // Calculate the Motor Control Torque For Each Link //
  robot->u = robot->motorGearRatio.inverse()*(Y*p + mK*r);
}

// Functions to Calculate Regressor Matrix //
Matrix1x2F oneLinkRegressor(sharedOneLinkRobot& robot, ScalarF& v, ScalarF& a)
{
  ScalarF thetaF = robot->thetaF;

  Matrix1x2F regressor;
  regressor(0,0) = a(0);
  regressor(0,1) = std::cos(thetaF(0));

  return regressor;
}

// Functions to Calculate Passivity Terms //
ScalarF oneLinkAdaptiveControl::calculateOneLinkV(sharedOneLinkRobot& robot)
{
  ScalarF e = robot->e;
  ScalarF dtheta_d = robot->dtheta_d;
  ScalarF v = dtheta_d - mLambda*e;

  return v;
}

ScalarF oneLinkAdaptiveControl::calculateOneLinkA(sharedOneLinkRobot& robot)
{

  ScalarF de = robot->de;
  ScalarF ddtheta_d = robot->ddtheta_d;
  ScalarF a = ddtheta_d - mLambda*de;

  return a;
}

ScalarF oneLinkAdaptiveControl::calculateOneLinkR(sharedOneLinkRobot& robot)
{
  ScalarF e = robot->e;
  ScalarF de = robot->de;
  ScalarF r = de + mLambda*e;

  return r;
}

} // namespace robot
