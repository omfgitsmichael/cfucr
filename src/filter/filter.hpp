#ifndef FILTER_HPP_
#define FILTER_HPP_

#include <string>
#include <vector>

#include "mathUtilities/matrixUtilities.hpp"

namespace robot
{
  
template <typename Robot>
class Filter
{
public:
  Filter()
  {
  }

  ~Filter()
  {
  }

  // Low Pass Filter //
  // Execute Low Pass Filter //
  void executeLowPassFilter(Robot& robot)
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

  float lowPassFilter(float& y, float& yInt, float& alpha)
  {
    float yOut1;

    yOut1 = alpha*y + (1-alpha)*yInt;
    yInt = yOut1;

    return yOut1;
  }
  
    // Public member variables //
    std::string filterType;

    // Low pass filter variables //
    std::vector<float> mAlphaQ;
    std::vector<float> mAlphadQ;
    std::vector<std::vector<float>> mPreviousIntegralOutputQ;
    std::vector<std::vector<float>> mPreviousIntegralOutputdQ;
    unsigned int mFilterOrder;
  };

} // namespace robot

#endif // FILTER_HPP_
