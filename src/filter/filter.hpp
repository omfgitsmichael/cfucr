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
  void executeLowPassFilter(Robot& robot);
  float lowPassFilter(float& y, float& yInt, float& alpha);
 
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
