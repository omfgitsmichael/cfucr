#ifndef ONE_LINK_ADAPTIVE_CONTROL_HPP_
#define ONE_LINK_ADAPTIVE_CONTROL_HPP_

#include "control/controlUtilities.hpp"

namespace robot
{

class oneLinkAdaptiveControl
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  oneLinkAdaptiveControl()
  {
  }

  ~oneLinkAdaptiveControl()
  {
  }
  
  // One link adaptive control //
  void execute(sharedOneLinkRobot& robot);

  // Public member variables //
  ScalarF mK;
  ScalarF mLambda;
  Matrix2x2F mGamma;
  float mDelt;

  // Public member variables: not used --  hacky, not sure what else to do //
  float mRho;
  float mEpsilon;
  ScalarF mKp;
  ScalarF mKd;
  
};

} // namespace robot
#endif // ONE_LINK_ADAPTIVE_CONTROL_HPP_
