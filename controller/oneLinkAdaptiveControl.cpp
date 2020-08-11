#include <cmath>

#include "controller/oneLinkAdaptiveControl.hpp"

namespace robot
{
// One Link Adaptive Control //
void oneLinkAdaptiveControl::execute(sharedOneLinkRobot& robot, const float theta_d, const float dtheta_d, const float ddtheta_d)
{
  // Store the Latest Desired Trajectory in the Robot //
  robot->theta_d = theta_d;
  robot->dtheta_d = dtheta_d;
  robot->ddtheta_d = ddtheta_d;

  // Calculate and Store the Error //
  float thetaF = robot->thetaF;
  float dthetaF = robot->dthetaF;

  float e = thetaF - theta_d;
  float de = dthetaF - dtheta_d;

  robot->e = e;
  robot->de = de;

  // Calculate Passivity Terms //
  float v = calculateOneLinkV(robot);
  float a = calculateOneLinkA(robot);
  float r = calculateOneLinkR(robot);

  // Calculate Regressor Matrix //
  Matrix1x2F Y = oneLinkRegressor(robot, v, a);

  // Update The Estimated Parameters //
  Vector2x1F F = -mGamma.inverse()*Y.transpose()*r;
  Vector2x1F p = robot->parameters + F*mDelt;
  robot->parameters = p;

  // Calculate the Control //
  robot->u = Y*p + mK*r;
}

// Functions to Calculate Regressor Matrix //
Matrix1x2F oneLinkRegressor(sharedOneLinkRobot& robot, const float& v, const float& a)
{
  float thetaF = robot->thetaF;

  Matrix1x2F regressor;
  regressor(0,0) = a;
  regressor(0,1) = std::cos(thetaF);

  return regressor;
}

// Functions to Calculate Passivity Terms //
float oneLinkAdaptiveControl::calculateOneLinkV(sharedOneLinkRobot& robot)
{
  const float e = robot->e;
  const float dtheta_d = robot->dtheta_d;
  const float v = dtheta_d - mLambda*e;

  return v;
}

float oneLinkAdaptiveControl::calculateOneLinkA(sharedOneLinkRobot& robot)
{

  const float de = robot->de;
  const float ddtheta_d = robot->ddtheta_d;
  const float a = ddtheta_d - mLambda*de;

  return a;
}

float oneLinkAdaptiveControl::calculateOneLinkR(sharedOneLinkRobot& robot)
{
  const float e = robot->e;
  const float de = robot->de;
  const float r = de + mLambda*e;

  return r;
}

} // namespace robot
