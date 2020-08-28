#include <iostream>
#include <chrono>

#include "controller.hpp"
#include "control/oneLinkControl.hpp"
#include "filter/filter.hpp"
#include "types/oneLinkRobot.hpp"

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
  robot::OneLinkRobot robot;
  
  // Initialize the controller with the desired classes, the config file, and the robot //
  robot::Controller<robot::OneLinkRobot, robot::OneLinkControl, robot::Filter<robot::OneLinkRobot>> controller(configFile1, robot);
  
  // If the user desires they can grab the current state controller and filter //
  robot::OneLinkControl control = controller.control();
  robot::Filter<robot::OneLinkRobot> filter = controller.filter();
  
  // To run the controller all the user needs to do is populate the robot with measured and desired states and then execute the controller //
  robot.theta(0) = 0.1223f; // measured rad
  robot.dtheta(0) = -0.3431f; // measured rad/sec

  robot.theta_d(0) = 0.12f; // desired rad
  robot.dtheta_d(0) = -0.2901f; // desired rad/sec
  robot.ddtheta_d(0) = 5.6113f; // desired rad/sec^2
  
  auto t1 = std::chrono::high_resolution_clock::now();
  controller.execute(robot);
  auto t2 = std::chrono::high_resolution_clock::now();

  std::chrono::duration<float> timeDuration = t2 - t1;
  std::cout << "It took " << timeDuration.count() << " seconds to run execute!" << std::endl;

  std::cout << "\nControl output:" << std::endl;
  std::cout << robot.u << std::endl;

  return 0;
}