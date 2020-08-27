#include <iostream>
#include <chrono>

#include "controller.hpp"
#include "control/twoLinkControl.hpp"
#include "filter/filter.hpp"
#include "types/twoLinkRobot.hpp"

int main(int argc, char* argv[])
{ 
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
  
  // Initialize the robot //
  robot::sharedTwoLinkRobot robot = std::make_shared<robot::TwoLinkRobot>();
  
  // Initialize the controller with the desired classes, the config file, and the robot //
  robot::Controller<robot::sharedTwoLinkRobot, robot::TwoLinkControl, robot::Filter<robot::sharedTwoLinkRobot>> controller(configFile1, robot);
  
  // If the user desires they can grab the current state controller and filter //
  robot::TwoLinkControl control = controller.control();
  robot::Filter<robot::sharedTwoLinkRobot> filter = controller.filter();

  // To run the controller all the user needs to do is populate the robot with measured and desired states and then execute the controller //
  robot->theta(0) = 0.4716f; // measured rad
  robot->theta(1) = -0.7399f; // measured rad

  robot->dtheta(0) = -0.3680f; // measured rad/sec
  robot->dtheta(1) = 0.5872f; // measured rad/sec

  robot->theta_d(0) = 0.4933f; // desired rad
  robot->dtheta_d(0) = -0.4539f; // desired rad/sec
  robot->ddtheta_d(0) = -3.0612f; // desired rad/sec^2

  robot->theta_d(1) = -0.7399f; // desired rad
  robot->dtheta_d(1) = 0.6809f; // desired rad/sec
  robot->ddtheta_d(1) = 4.5918f; // desired rad/sec^2
  
  auto t1 = std::chrono::high_resolution_clock::now();
  controller.execute(robot);
  auto t2 = std::chrono::high_resolution_clock::now();

  std::chrono::duration<float> timeDuration = t2 - t1;
  std::cout << "It took " << timeDuration.count() << " seconds to run execute!" << std::endl;

  std::cout << "\nControl output:" << std::endl;
  std::cout << robot->u << std::endl;

  return 0;
}