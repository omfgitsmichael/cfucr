#include <iostream>
#include <chrono>

#include "controller.hpp"
#include "control/twoLinkControl.hpp"
#include "filter/filter.hpp"
#include "types/twoLinkRobot.hpp"

int main(int argc, char* argv[])
{ 
  std::cout << argc << std::endl;
  char * configFile1;
  if (argc == 2)
  {
    configFile1 = argv[1];
  }
  else
  {
    std::cout << "Incorrect number of arguments!" << std::endl;

    return 0;
  }

  robot::sharedTwoLinkRobot robot = std::make_shared<robot::TwoLinkRobot>();

  robot::Controller<robot::sharedTwoLinkRobot, robot::TwoLinkControl, robot::Filter<robot::sharedTwoLinkRobot>> controller(configFile1, robot);

  robot::TwoLinkControl control = controller.control();
  robot::Filter<robot::sharedTwoLinkRobot> filter = controller.filter();
  
  // Print out all of the initialize information to confirm it has initialized properly //
  std::cout << "Robot Stuff:" << std::endl;
  std::cout << robot->numberLinks << std::endl;
  std::cout << robot->parameters << std::endl;
  std::cout << robot->motorGearRatio << std::endl;
  
  std::cout << "\nController Stuff:" << std::endl;
  std::cout << "Adaptive Stuff:" << std::endl;
  std::cout << control.mK << std::endl;
  std::cout << control.mLambda << std::endl;
  std::cout << control.mGamma << std::endl;
  std::cout << control.mDelt << std::endl;
  std::cout << "Robust Stuff:" << std::endl;
  std::cout << control.mK << std::endl;
  std::cout << control.mLambda << std::endl;
  std::cout << control.mRho << std::endl;
  std::cout << control.mEpsilon << std::endl;
  std::cout << "PD Stuff:" << std::endl;
  std::cout << control.mKp << std::endl;
  std::cout << control.mKd << std::endl;

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

  robot->theta(1) = 0.00175f; // measured 0.1 deg
  robot->dtheta(1) = 0.000175f; // measured 0.01 deg/sec

  robot->theta_d(1) = 0.01f;
  robot->dtheta_d(1) = 0.01f;
  robot->ddtheta_d(1) = 0.01f;
  
  auto t1 = std::chrono::high_resolution_clock::now();
  controller.execute(robot);
  auto t2 = std::chrono::high_resolution_clock::now();

  std::chrono::duration<float> timeDuration = t2 - t1;

  std::cout << "It took " << timeDuration.count() << " seconds to run execute!" << std::endl;

  std::cout << "\nControl output:" << std::endl;
  std::cout << robot->u << std::endl;

  return 0;
}