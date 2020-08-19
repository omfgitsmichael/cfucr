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

private:
  Matrix mKp;
  Matrix mKd;
};

} // namespace robot
#endif // PD_CONTROL_HPP_
