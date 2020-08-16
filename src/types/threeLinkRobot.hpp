#ifndef THREE_LINK_ROBOT_HPP_
#define THREE_LINK_ROBOT_HPP_

#include <memory>
#include <vector>

#include "mathUtilities/matrixUtilities.hpp"

namespace robot
{

class ThreeLinkRobot
{
public:
  // Constructor //
  ThreeLinkRobot() = default;

  // Destructor //
  ~ThreeLinkRobot() = default;

  // Measured Robot States //
  Vector3x1F theta;
  Vector3x1F dtheta;

  // Filtered Robot States //
  Vector3x1F thetaF;
  Vector3x1F dthetaF;

  // State Error //
  Vector3x1F e;
  Vector3x1F de;

  // Desired States //
  Vector3x1F theta_d;
  Vector3x1F dtheta_d;
  Vector3x1F ddtheta_d;

  // Dynamic Parameters Estimates //
  Vector9x1F parameters;

  // Control //
  Vector3x1F u;

  // General Robot //
  unsigned int numberLinks;
  Matrix3x3F motorGearRatio; // Diagonal Matrix of the Gear Ratios //
};

// Shared Pointer to the Robot //
using sharedThreeLinkRobot = std::shared_ptr<ThreeLinkRobot>;

} // namespace robot

#endif // THREE_LINK_ROBOT_HPP_
