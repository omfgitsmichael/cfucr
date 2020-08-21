#include "controller.hpp"

namespace robot
{

template <typename Robot, typename Control, typename Filter>
void Controller<Robot, Control, Filter>::execute(Robot& robot)
{
  // Execute the filter and control algorithms //
  mFilter.execute(robot);
  mControl.execute(robot);
}

} // namespace robot