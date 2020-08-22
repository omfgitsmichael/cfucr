#include "controller.hpp"
#include "control/oneLinkControl.hpp"
#include "filter/filter.hpp"
#include "types/oneLinkRobot.hpp"

int main()
{
  const char* configFile = "../../../configurations/sampleOneLinkAdaptiveConfig.xml";
  robot::sharedOneLinkRobot robot = std::make_shared<robot::OneLinkRobot>();

  robot::Controller<robot::sharedOneLinkRobot, robot::OneLinkControl, robot::Filter<robot::sharedOneLinkRobot>> controller(configFile, robot);

  return 0;
}