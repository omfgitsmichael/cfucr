#include "filter/filter.hpp"

namespace robot
{
// Execute Low Pass Filter //
template <typename Robot>
void Filter<Robot>::executeLowPassFilter(Robot& robot)
{
  // Nested for loop between the number of robot links and filter order //
  for (unsigned int i = 0; i < robot->numberLinks; i++)
  {
    for (unsigned int j = 0; j < mFilterOrder; j++)
    {
      robot->thetaF(i) = lowPassFilter(robot->theta(i), mPreviousIntegralOutputQ[i][j], mAlphaQ[j]);
      robot->dthetaF(i) = lowPassFilter(robot->dtheta(i), mPreviousIntegralOutputdQ[i][j], mAlphadQ[j]);
    }
  }
}

template <typename Robot>
float Filter<Robot>::lowPassFilter(float& y, float& yInt, float& alpha)
{
  float yOut1;

  yOut1 = alpha*y + (1-alpha)*yInt;
  yInt = yOut1;

  return yOut1;
}

} // namespace robot
