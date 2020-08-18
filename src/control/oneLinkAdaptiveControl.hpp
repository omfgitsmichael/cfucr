#ifndef ONE_LINK_ADAPTIVE_CONTROL_HPP_
#define ONE_LINK_ADAPTIVE_CONTROL_HPP_

#include "control/controlUtilities.hpp"

namespace robot
{

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
  void execute(sharedOneLinkRobot& robot);

private:
  ScalarF mK;
  ScalarF mLambda;
  Matrix2x2F mGamma;
  float mDelt;
};

} // namespace robot
#endif // ONE_LINK_ADAPTIVE_CONTROL_HPP_
