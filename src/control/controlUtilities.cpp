#include "control/controlUtilities.hpp"

namespace robot
{
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