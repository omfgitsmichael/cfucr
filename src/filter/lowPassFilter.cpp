#include "filter/lowPassFilter.hpp"

namespace robot
{
// One Link Adaptive Control //
template <typename Robot>
void lowPassFilter::execute(Robot& robot)
{
  // Nested for loop between the number of robot links and filter order //
  for (unsigned int j = 0; j < robot->numberLinks; j++)
  {
    for (unsigned int k = 0; k < filterOrder; k++)
    {
      robot->thetaF(j) = filter(robot->theta(j), previousIntegralOutputQ[j][k], alphaQ[k]);
      robot->dthetaF(j) = filter(robot->dtheta(j), previousIntegralOutputdQ[j][k], alphadQ[k]);
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
