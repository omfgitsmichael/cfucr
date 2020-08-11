#ifndef LOWPASSFILTER_HPP_
#define LOWPASSFILTER_HPP_

#include <string>
#include <vector>

#include "math-utilities/include/matrix_utilities.hpp"

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

  // Register the Class //
  void registerLowPass(std::string id);

private:
  std::vector<float> alpha;
  std::vector<float> previousOutput;
  unsigned int filterOrder;
};

}

#endif // LOWPASSFILTER_HPP_
