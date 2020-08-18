#ifndef LOW_PASS_FILTER_HPP_
#define LOW_PASS_FILTER_HPP_

#include <string>
#include <vector>

#include "mathUtilities/matrixUtilities.hpp"

namespace robot
{
  
template <typename Robot>
class lowPassFilter
{
public:
  lowPassFilter()
  {
  }

  ~lowPassFilter()
  {
  }

  // Low Pass Filter //
  void execute(Robot& robot);
  float filter(float& y, float& yInt, float& alpha);

private:
  std::vector<float> mAlphaQ;
  std::vector<float> mAlphadQ;
  std::vector<std::vector<float>> mPreviousIntegralOutputQ;
  std::vector<std::vector<float>> mPreviousIntegralOutputdQ;
  unsigned int mFilterOrder;
};

} // namespace robot

#endif // LOW_PASS_FILTER_HPP_
