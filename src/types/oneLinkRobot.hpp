#ifndef ONELINKROBOT_HPP_
#define ONELINKROBOT_HPP_

#include <memory>
#include <vector>

#include "math-utilities/include/matrix_utilities.hpp"

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

#endif // ONELINKROBOT_HPP_
