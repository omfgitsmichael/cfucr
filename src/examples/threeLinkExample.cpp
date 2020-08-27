#include <iostream>
#include <chrono>

#include "controller.hpp"
#include "control/threeLinkControl.hpp"
#include "filter/filter.hpp"
#include "types/threeLinkRobot.hpp"

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
  robot::sharedThreeLinkRobot robot = std::make_shared<robot::ThreeLinkRobot>();
  
  // Initialize the controller with the desired classes, the config file, and the robot //
  robot::Controller<robot::sharedThreeLinkRobot, robot::ThreeLinkControl, robot::Filter<robot::sharedThreeLinkRobot>> controller(configFile1, robot);
  
  // If the user desires they can grab the current state controller and filter //
  robot::ThreeLinkControl control = controller.control();
  robot::Filter<robot::sharedThreeLinkRobot> filter = controller.filter();

  // To run the controller all the user needs to do is populate the robot with measured and desired states and then execute the controller //
  robot->theta(0) = 0.00175f; // measured
  robot->theta(1) = 0.00175f; // measured
  robot->theta(2) = 0.00175f; // measured

  robot->dtheta(0) = 0.000175f; // measured
  robot->dtheta(1) = 0.000175f; // measured
  robot->dtheta(2) = 0.000175f; // measured

  robot->theta_d(0) = 0.01f;
  robot->dtheta_d(0) = 0.01f;
  robot->ddtheta_d(0) = 0.01f;

  robot->theta_d(1) = 0.01f;
  robot->dtheta_d(1) = 0.01f;
  robot->ddtheta_d(1) = 0.01f;

  robot->theta_d(2) = 0.01f;
  robot->dtheta_d(2) = 0.01f;
  robot->ddtheta_d(2) = 0.01f;
  
  auto t1 = std::chrono::high_resolution_clock::now();
  controller.execute(robot);
  auto t2 = std::chrono::high_resolution_clock::now();

  std::chrono::duration<float> timeDuration = t2 - t1;
  std::cout << "It took " << timeDuration.count() << " seconds to run execute!" << std::endl;

  std::cout << "\nControl output:" << std::endl;
  std::cout << robot->u << std::endl;

  return 0;
}