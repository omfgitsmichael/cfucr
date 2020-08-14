#ifndef ONE_LINK_ROBOT_HPP_
#define ONE_LINK_ROBOT_HPP_

#include <memory>
#include <vector>

#include "mathUtilities/matrixUtilities.hpp"

namespace robot
{

class OneLinkRobot
{
public:
  // Constructor //
  OneLinkRobot() = default;

  // Destructor //
  ~OneLinkRobot() = default;

  // Measured Robot States //
  float theta;
  float dtheta;

  // Filtered Robot States //
  float thetaF;
  float dthetaF;

  // State Error //
  float e;
  float de;

  // Desired States //
  float theta_d;
  float dtheta_d;
  float ddtheta_d;

  // Dynamic Parameters Estimates //
  Vector2x1F parameters;

  // Control //
  float u;

  // General Robot //
  unsigned int numberLinks;
  unsigned int motorGearRatio;
};

// Shared Pointer to the Robot //
using sharedOneLinkRobot = std::shared_ptr<OneLinkRobot>;

} // namespace robot

#endif // ONE_LINK_ROBOT_HPP_
