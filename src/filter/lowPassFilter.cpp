#include "filter/lowPassFilter.hpp"

namespace robot
{
// Execute Low Pass Filter //
template <typename Robot>
void lowPassFilter::execute(Robot& robot)
{
  // Nested for loop between the number of robot links and filter order //
  for (unsigned int j = 0; j < robot->numberLinks; j++)
  {
    for (unsigned int k = 0; k < mFilterOrder; k++)
    {
      robot->thetaF(j) = filter(robot->theta(j), mPreviousIntegralOutputQ[j][k], mAlphaQ[k]);
      robot->dthetaF(j) = filter(robot->dtheta(j), mPreviousIntegralOutputdQ[j][k], mAlphadQ[k]);
    }
  }
}

float filter(float& y, float& yInt, float& alpha)
{
  float yOut1;

  yOut1 = alpha*y + (1-alpha)*yInt;
  yInt = yOut1;

  return yOut1;
}

} // namespace robot
