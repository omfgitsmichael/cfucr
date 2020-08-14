#ifndef LOW_PASS_FILTER_HPP_
#define LOW_PASS_FILTER_HPP_

#include <string>
#include <vector>

#include "mathUtilities/matrixUtilities.hpp"

namespace robot
{
static const char *lowPass = "lowPass";

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

  // Register the Class //
  void registerLowPass(std::string id);

private:
  std::vector<float> alphaQ;
  std::vector<float> alphadQ;
  std::vector<std::vector<float>> previousIntegralOutputQ;
  std::vector<std::vector<float>> previousIntegralOutputdQ;
  unsigned int filterOrder;
};

} // namespace robot

#endif // LOW_PASS_FILTER_HPP_
