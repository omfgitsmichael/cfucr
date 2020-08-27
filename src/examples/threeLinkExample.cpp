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
  robot->theta(0) = 0.3021f; // measured rad
  robot->theta(1) = -0.7063f; // measured rad
  robot->theta(2) = 0.7140f; // measured rad

  robot->dtheta(0) = -0.3404f; // measured rad/sec
  robot->dtheta(1) = 0.8760f; // measured rad/sec
  robot->dtheta(2) = -1.0325f; // measured rad/sec

  robot->theta_d(0) = 0.3438f; // desired rad
  robot->dtheta_d(0) = -0.6876f; // desired rad/sec
  robot->ddtheta_d(0) = 0.6876f; // desired rad/sec^2

  robot->theta_d(1) = -0.4718f; // desired rad
  robot->dtheta_d(1) = 0.9436f; // desired rad/sec
  robot->ddtheta_d(1) = -0.9436f; // desired rad/sec^2

  robot->theta_d(2) = -1.8057f; // desired rad
  robot->dtheta_d(2) = 3.6115f; // desired rad/sec
  robot->ddtheta_d(2) = -3.6115f; // desired rad/sec^2
  
  auto t1 = std::chrono::high_resolution_clock::now();
  controller.execute(robot);
  auto t2 = std::chrono::high_resolution_clock::now();

  std::chrono::duration<float> timeDuration = t2 - t1;
  std::cout << "It took " << timeDuration.count() << " seconds to run execute!" << std::endl;

  std::cout << "\nControl output:" << std::endl;
  std::cout << robot->u << std::endl;

  return 0;
}