#ifndef ONE_LINK_CONTROL_HPP_
#define ONE_LINK_CONTROL_HPP_

#include "control/controlUtilities.hpp"

namespace robot
{

class OneLinkControl
{
public:
  
  OneLinkControl()
  {
  }

  ~OneLinkControl()
  {
  }
  
  // One link adaptive control //
  void executeAdaptiveControl(sharedOneLinkRobot& robot);

  // One link robust control //
  void executeRobustControl(sharedOneLinkRobot& robot);

  // One link PD control //
  void executePDControl(sharedOneLinkRobot& robot);

  // Public member variables //
  std::string controlType;
  
  // Adaptive and robust control specific //
  ScalarF mK;
  ScalarF mLambda;

  // Adaptive control specific //
  Matrix2x2F mGamma;
  float mDelt;

  // Robust control specific //
  float mRho;
  float mEpsilon;

  // PD control specific //
  ScalarF mKp;
  ScalarF mKd;
};

} // namespace robot
#endif // ONE_LINK_CONTROL_HPP_
