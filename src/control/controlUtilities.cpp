#include "control/controlUtilities.hpp"

namespace robot
{
// Functions to calculate passivity terms //
template <typename Robot, typename Vector, typename Matrix>
Vector calculateV(Robot& robot, Matrix& lambda)
{
  Vector e = robot->e;
  Vector dtheta_d = robot->dtheta_d;
  Vector v = dtheta_d - lambda*e;

  return v;
}

template <typename Robot, typename Vector, typename Matrix>
Vector calculateA(Robot& robot, Matrix& lambda)
{

  Vector de = robot->de;
  Vector ddtheta_d = robot->ddtheta_d;
  Vector a = ddtheta_d - lambda*de;

  return a;
}

template <typename Robot, typename Vector, typename Matrix>
Vector calculateR(Robot& robot, Matrix& lambda)
{
  Vector e = robot->e;
  Vector de = robot->de;
  Vector r = de + lambda*e;

  return r;
}

// One link regressor matrix //
Matrix1x2F oneLinkRegressor(sharedOneLinkRobot& robot, ScalarF& v, ScalarF& a)
{
  ScalarF thetaF = robot->thetaF;

  Matrix1x2F regressor;
  regressor(0,0) = a(0);
  regressor(0,1) = std::cos(thetaF(0));

  return regressor;
}

} // namespace robot