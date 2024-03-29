#ifndef TWO_LINK_ROBOT_HPP_
#define TWO_LINK_ROBOT_HPP_

#include "mathUtilities/matrixUtilities.hpp"

namespace robot
{

class TwoLinkRobot
{
public:
  // Constructor //
  TwoLinkRobot() = default;

  // Destructor //
  ~TwoLinkRobot() = default;

  // Measured Robot States //
  Vector2x1F theta;
  Vector2x1F dtheta;

  // Filtered Robot States //
  Vector2x1F thetaF;
  Vector2x1F dthetaF;

  // State Error //
  Vector2x1F e;
  Vector2x1F de;

  // Desired States //
  Vector2x1F theta_d;
  Vector2x1F dtheta_d;
  Vector2x1F ddtheta_d;

  // Dynamic Parameters Estimates //
  Vector5x1F parameters;

  // Control //
  Vector2x1F u;

  // General Robot //
  unsigned int numberLinks;
  Matrix2x2F motorGearRatio; // Diagonal Matrix of the Gear Ratios //
};

} // namespace robot

#endif // TWO_LINK_ROBOT_HPP_
