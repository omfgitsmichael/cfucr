#include "controller.hpp"

namespace robot
{

template<typename Robot>
void Controller::execute(Robot& robot)
{
  // Execute the Filter and Control Algorithms //
  mFilter->execute(robot);
  mControl->execute(robot);
}

} // namespace robot