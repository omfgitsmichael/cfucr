#include "filter/lowPassFilter.hpp"

namespace robot
{
// One Link Adaptive Control //
template <typename Robot>
void lowPassFilter::execute(Robot& robot)
{
  // Nested for loop for the number of filters and
  for (unsigned int j = 0; j < robot->numberLinks; j++)
  {
    for (unsigned int k = 0; k < filterOrder; k++)
    {
      std::tie(robot->thetaF(j), previousOutputQ[j][k]) = filter(robot->theta(j), previousOutputQ[j][k], alphaQ[k]);
      std::tie(robot->dthetaF(j), previousOutputdQ[j][k]) = filter(robot->dtheta(j), previousOutputdQ[j][k], alphadQ[k]);
    }
  }
}

std::tuple<float, float> filter(float y, float yInt, float alpha)
{
  float yOut1;
  float yOut2;

  yOut1 = alpha*y + (1-alpha)*yInt;
  yOut2 = yOut1;

  return std::make_tuple(yOut1, yOut2);
}

} // namespace robot
