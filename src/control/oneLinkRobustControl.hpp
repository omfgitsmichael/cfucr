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

  // Public member variables //
  ScalarF mK;
  ScalarF mLambda;
  float mRho;
  float mEpsilon;

  // Public member variables: not used --  hacky, not sure what else to do //
  Matrix2x2F mGamma;
  float mDelt;
  float mKp;
  float mKd;

};

} // namespace robot
#endif // ONE_LINK_ROBUST_CONTROL_HPP_
