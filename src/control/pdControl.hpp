#ifndef PD_CONTROL_HPP_
#define PD_CONTROL_HPP_

#include "control/controlUtilities.hpp"

namespace robot
{

template <typename Matrix, typename Robot>
class PDControl
{
public:
  PDControl()
  {
  }

  ~PDControl()
  {
  }
  
  // PD control //
  void execute(Robot& robot);

  // Public member variables //
  Matrix mKp;
  Matrix mKd;

  // Public member variables: not used --  hacky, not sure what else to do //
  float mRho;
  float mEpsilon;
  ScalarF mK;
  ScalarF mLambda;
  Matrix2x2F mGamma;
  float mDelt;

};

} // namespace robot
#endif // PD_CONTROL_HPP_
