#include <iostream>

#include "controller.hpp"
#include "control/oneLinkControl.hpp"
#include "filter/filter.hpp"
#include "types/oneLinkRobot.hpp"

int main()
{
  const char* configFile = "../../../configurations/sampleOneLinkAdaptiveConfig.xml";
  robot::sharedOneLinkRobot robot = std::make_shared<robot::OneLinkRobot>();

  robot::Controller<robot::sharedOneLinkRobot, robot::OneLinkControl, robot::Filter<robot::sharedOneLinkRobot>> controller(configFile, robot);

  robot::OneLinkControl control = controller.control();
  robot::Filter<robot::sharedOneLinkRobot> filter = controller.filter();
  
  // Print out all of the initialize information to confirm it has initialized properly //
  std::cout << "Robot Stuff:" << std::endl;
  std::cout << robot->numberLinks << std::endl;
  std::cout << robot->parameters << std::endl;
  std::cout << robot->motorGearRatio << std::endl;
  
  std::cout << "\nController Stuff:" << std::endl;
  std::cout << control.mK << std::endl;
  std::cout << control.mLambda << std::endl;
  std::cout << control.mGamma << std::endl;
  std::cout << control.mDelt << std::endl;

  std::cout << "\nFilter Stuff:" << std::endl;
  std::cout << filter.mFilterOrder << std::endl;
  for (unsigned int i = 0; i < robot->numberLinks; i++)
  {
    for (unsigned int j = 0; j < filter.mFilterOrder; j++)
    {
      std::cout << filter.mPreviousIntegralOutputQ[i][j] << " " << filter.mAlphaQ[j] << "\t" << filter.mPreviousIntegralOutputdQ[i][j] << " " << filter.mAlphadQ[j] << std::endl;
    }
  }

  // Lets test to see if the execute function is working or if it needs to be modified //
  robot->theta(0) = 0.00175f; // measured 0.1 deg
  robot->dtheta(0) = 0.000175f; // measured 0.01 deg/sec

  robot->theta_d(0) = 0.01f;
  robot->dtheta_d(0) = 0.01f;
  robot->ddtheta_d(0) = 0.01f;

  controller.execute(robot);

  std::cout << "\nControl output:" << std::endl;
  std::cout << robot->u << std::endl;

  return 0;
}