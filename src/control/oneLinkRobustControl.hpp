#ifndef ONE_LINK_ROBUST_CONTROL_HPP_
#define ONE_LINK_ROBUST_CONTROL_HPP_

#include "control/controlUtilities.hpp"

namespace robot
{

class oneLinkRobustControl
{
public:
  oneLinkRobustControl()
  {
  }

  ~oneLinkRobustControl()
  {
  }
  
  // One link robust control //
  void execute(sharedOneLinkRobot& robot);

private:
  ScalarF mK;
  ScalarF mLambda;
  float mRho;
  float mEpsilon;
};

} // namespace robot
#endif // ONE_LINK_ROBUST_CONTROL_HPP_
