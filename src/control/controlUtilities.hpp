#ifndef CONTROL_UTILITIES_HPP_
#define CONTROL_UTILITIES_HPP_

#include <cmath>
#include <string>

#include "mathUtilities/matrixUtilities.hpp"
#include "types/oneLinkRobot.hpp"
#include "types/twoLinkRobot.hpp"
#include "types/threeLinkRobot.hpp"

namespace robot
{
// Functions to calculate passivity terms //
template <typename Robot, typename Vector, typename Matrix>
inline Vector calculateV(Robot& robot, Matrix& lambda)
{
  Vector e = robot->e;
  Vector dtheta_d = robot->dtheta_d;
  Vector v = dtheta_d - lambda*e;

  return v;
}

template <typename Robot, typename Vector, typename Matrix>
inline Vector calculateA(Robot& robot, Matrix& lambda)
{

  Vector de = robot->de;
  Vector ddtheta_d = robot->ddtheta_d;
  Vector a = ddtheta_d - lambda*de;

  return a;
}

template <typename Robot, typename Vector, typename Matrix>
inline Vector calculateR(Robot& robot, Matrix& lambda)
{
  Vector e = robot->e;
  Vector de = robot->de;
  Vector r = de + lambda*e;

  return r;
}

// One link regressor matrix //
inline Matrix1x2F oneLinkRegressor(sharedOneLinkRobot& robot, ScalarF& v, ScalarF& a)
{
  ScalarF thetaF = robot->thetaF;

  Matrix1x2F regressor;
  regressor(0,0) = a(0);
  regressor(0,1) = std::cos(thetaF(0));

  return regressor;
}

// Two link regressor matrix //
inline Matrix2x5F twoLinkRegressor(sharedTwoLinkRobot& robot, Vector2x1F& v, Vector2x1F& a)
{
  Vector2x1F thetaF = robot->thetaF;

  Matrix2x5F regressor;
  regressor(0,0) = a(0);
  regressor(0,1) = std::cos(thetaF(0));

  return regressor;
}

// Three link regressor matrix //
inline Matrix3x9F threeLinkRegressor(sharedThreeLinkRobot& robot, Vector3x1F& v, Vector3x1F& a)
{
  Vector3x1F thetaF = robot->thetaF;

  Matrix3x9F regressor;
  regressor(0,0) = a(0);
  regressor(0,1) = std::cos(thetaF(0));

  return regressor;
}

} // namespace robot
#endif // CONTROL_UTILITIES_HPP_
