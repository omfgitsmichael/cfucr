#ifndef CONTROL_HPP_
#define CONTROL_HPP_

#include <string>

#include "types/oneLinkRobot.hpp"
#include "math-utilities/include/matrix_utilities.hpp"

namespace robot
{
static const char *oneLinkAdaptive = "oneLinkAdaptive";

class oneLinkAdaptiveControl
{
public:
  oneLinkAdaptiveControl()
  {
  }

  ~oneLinkAdaptiveControl()
  {
  }
  // One Link Adaptive Control //
  void execute(sharedOneLinkRobot& robot, const float theta_d, const float dtheta_d, const float ddtheta_d);

  // Functions to Calculate Regressor Matrix //
  Matrix1x2F oneLinkRegressor(sharedOneLinkRobot& robot, const float& v, const float& a);

  // Functions to Calculate Passivity Terms //
  float calculateOneLinkV(sharedOneLinkRobot& robot);
  float calculateOneLinkA(sharedOneLinkRobot& robot);
  float calculateOneLinkR(sharedOneLinkRobot& robot);

  // Register the Class //
  void registerOneLinkAdaptive(std::string id);

private:
  float mK;
  float mLambda;
  Matrix2x2F mGamma;
  float mDelt;

};

} // namespace robot
#endif // CONTROL_HPP_
