#ifndef CONTROL_UTILITIES_HPP_
#define CONTROL_UTILITIES_HPP_

#include <cmath>
#include <string>

#include "mathUtilities/matrixUtilities.hpp"
#include "types/oneLinkRobot.hpp"

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
Matrix1x2F oneLinkRegressor(sharedOneLinkRobot& robot, ScalarF& v, ScalarF& a);

} // namespace robot
#endif // CONTROL_UTILITIES_HPP_
