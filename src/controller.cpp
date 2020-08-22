#include "controller.hpp"

namespace robot
{

template <typename Robot, typename Control, typename Filter>
void Controller<Robot, Control, Filter>::execute(Robot& robot)
{
  // Execute the filter algorithms //
  if (mFilter.filterType.compare("lowPassFilter") > 0)
  {
    mFilter.executeLowPassFilter(robot);
  }
  
  // Execute the control algorithms //
  if (mControl.controlType.compare("adaptiveControl") > 0)
  {
    mControl.executeAdaptiveControl(robot);
  }
  else if (mControl.controlType.compare("robustControl") > 0)
  {
    mControl.executeRobustControl(robot);
  }
  else if (mControl.controlType.compare("pdControl") > 0)
  {
    mControl.executePDControl(robot);
  }
}

} // namespace robot