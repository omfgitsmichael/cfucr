#ifndef ONE_LINK_ROBOT_HPP_
#define ONE_LINK_ROBOT_HPP_

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
  ScalarF theta;
  ScalarF dtheta;

  // Filtered Robot States //
  ScalarF thetaF;
  ScalarF dthetaF;

  // State Error //
  ScalarF e;
  ScalarF de;

  // Desired States //
  ScalarF theta_d;
  ScalarF dtheta_d;
  ScalarF ddtheta_d;

  // Dynamic Parameters Estimates //
  Vector2x1F parameters;

  // Control //
  ScalarF u;

  // General Robot //
  unsigned int numberLinks;
  ScalarF motorGearRatio; // Diagonal Matrix of the Gear Ratios //
};

} // namespace robot

#endif // ONE_LINK_ROBOT_HPP_
