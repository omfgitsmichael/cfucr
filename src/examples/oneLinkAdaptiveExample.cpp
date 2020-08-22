#include "controller.hpp"
#include "control/oneLinkAdaptiveControl.hpp"
#include "filter/lowPassFilter.hpp"
#include "types/oneLinkRobot.hpp"

int main()
{
  const char* configFile = "../../../configurations/sampleOneLinkAdaptiveConfig.xml";
  robot::sharedOneLinkRobot robot = std::make_shared<robot::OneLinkRobot>();

  robot::Controller<robot::sharedOneLinkRobot, robot::oneLinkAdaptiveControl, robot::lowPassFilter<robot::sharedOneLinkRobot>> controller(configFile, robot);

  return 0;
}