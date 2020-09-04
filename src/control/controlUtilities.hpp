#ifndef CONTROL_UTILITIES_HPP_
#define CONTROL_UTILITIES_HPP_

#include <cmath>
#include <string>

#include "mathUtilities/matrixUtilities.hpp"
#include "types/oneLinkRobot.hpp"
#include "types/twoLinkRobot.hpp"
#include "types/threeLinkRobot.hpp"

namespace robot
{
// Functions to calculate passivity terms //
template <typename Robot, typename Vector, typename Matrix>
inline Vector calculateV(Robot& robot, Matrix& lambda)
{
  Vector e = robot.e;
  Vector dtheta_d = robot.dtheta_d;
  Vector v = dtheta_d - lambda*e;

  return v;
}

template <typename Robot, typename Vector, typename Matrix>
inline Vector calculateA(Robot& robot, Matrix& lambda)
{

  Vector de = robot.de;
  Vector ddtheta_d = robot.ddtheta_d;
  Vector a = ddtheta_d - lambda*de;

  return a;
}

template <typename Robot, typename Vector, typename Matrix>
inline Vector calculateR(Robot& robot, Matrix& lambda)
{
  Vector e = robot.e;
  Vector de = robot.de;
  Vector r = de + lambda*e;

  return r;
}

// One link regressor matrix //
inline Matrix1x2F oneLinkRegressor(OneLinkRobot& robot, ScalarF& v, ScalarF& a)
{
  ScalarF q = robot.thetaF;

  Matrix1x2F regressor;
  regressor(0,0) = a(0);
  regressor(0,1) = std::cos(q(0));

  return regressor;
}

// Two link regressor matrix //
inline Matrix2x5F twoLinkRegressor(TwoLinkRobot& robot, Vector2x1F& v, Vector2x1F& a)
{
  Vector2x1F q = robot.thetaF;
  Vector2x1F qdot = robot.dthetaF;

  Matrix2x5F regressor;
  regressor(0,0) = a(0);
  regressor(0,1) = a(0) + a(1);
  regressor(0,2) = std::cos(q(1))*(2*a(0)+a(1)) - std::sin(q(1))*(qdot(1)*v(0)+qdot(0)*v(1)+qdot(1)*v(1));
  regressor(0,3) = std::cos(q(0));
  regressor(0,4) = std::cos(q(0)+q(1));

  regressor(1,0) = 0.0f;
  regressor(1,1) = a(0)+a(1);
  regressor(1,2) = std::cos(q(1))*a(0) + std::sin(q(1))*qdot(0)*v(0);
  regressor(1,3) = 0.0f;
  regressor(1,4) = std::cos(q(0)+q(1));

  return regressor;
}

// Three link regressor matrix //
inline Matrix3x9F threeLinkRegressor(ThreeLinkRobot& robot, Vector3x1F& v, Vector3x1F& a)
{
  Vector3x1F q = robot.thetaF;
  Vector3x1F qdot = robot.dthetaF;

  Matrix3x9F regressor;
  regressor(0,0) = a(0);
  regressor(0,1) = a(0) + a(1);
  regressor(0,2) = a(0) + a(1) + a(2);
  regressor(0,3) = std::cos(q(1))*(2*a(0)+a(1)) - std::sin(q(1))*(qdot(1)*v(0)+(qdot(0)+qdot(1))*v(1));
  regressor(0,4) = std::cos(q(1)+q(2))*(2*a(0)+a(1)+a(2)) - std::sin(q(1)+q(2))*((qdot(1)+qdot(2))*v(0)+(qdot(0)+qdot(1)+qdot(2))*v(1)+(qdot(0)+qdot(1)+qdot(2))*v(2));
  regressor(0,5) = std::cos(q(2))*(2*a(0)+2*a(1)+a(2)) - std::sin(q(2))*(qdot(2)*v(0)+qdot(2)*v(1)+(qdot(0)+qdot(1)+qdot(2))*v(2));
  regressor(0,6) = std::cos(q(0));
  regressor(0,7) = std::cos(q(0)+q(1));
  regressor(0,8) = std::cos(q(0)+q(1)+q(2));

  regressor(1,0) = 0.0f;
  regressor(1,1) = a(0) + a(1);
  regressor(1,2) = a(0) + a(1) + a(2);
  regressor(1,3) = std::cos(q(1))*a(0) + std::sin(q(1))*qdot(0)*v(0);
  regressor(1,4) = std::cos(q(1)+q(2))*a(0) + std::sin(q(1)+q(2))*qdot(0)*v(0);
  regressor(1,5) = std::cos(q(2))*(2*a(0)+2*a(1)+a(2)) - std::sin(q(2))*(qdot(2)*v(0)+qdot(2)*v(1)+(qdot(0)+qdot(1)+qdot(2))*v(2));
  regressor(1,6) = 0.0f;
  regressor(1,7) = std::cos(q(0)+q(1));
  regressor(1,8) = std::cos(q(0)+q(1)+q(2));

  regressor(2,0) = 0.0f;
  regressor(2,1) = 0.0f;
  regressor(2,2) = a(0) + a(1) + a(2);
  regressor(2,3) = 0.0f;
  regressor(2,4) = std::cos(q(1)+q(2))*a(0) + std::sin(q(1)+q(2))*qdot(0)*v(0);
  regressor(2,5) = std::cos(q(2))*(a(0)+a(1)) + std::sin(q(2))*(qdot(0)+qdot(1))*(v(0)+v(1));
  regressor(2,6) = 0.0f;
  regressor(2,7) = 0.0f;
  regressor(2,8) = std::cos(q(0)+q(1)+q(2));

  return regressor;
}

inline ScalarF oneLinkGavityTerms(OneLinkRobot& robot)
{
  ScalarF g;
  
  g(0) = robot.parameters(1)*std::cos(robot.thetaF(0));
  return g;
}

inline Vector2x1F twoLinkGavityTerms(TwoLinkRobot& robot)
{
  Vector2x1F g;

  g(0) = robot.parameters(3)*std::cos(robot.thetaF(0)) + robot.parameters(4)*std::cos(robot.thetaF(0)+robot.thetaF(1));
  g(1) = robot.parameters(4)*std::cos(robot.thetaF(0)+robot.thetaF(1));
  return g;
}

inline Vector3x1F threeLinkGavityTerms(ThreeLinkRobot& robot)
{
  Vector3x1F g;

  g(0) = robot.parameters(6)*std::cos(robot.thetaF(0)) + robot.parameters(7)*std::cos(robot.thetaF(0)+robot.thetaF(1)) + robot.parameters(8)*std::cos(robot.thetaF(0)+robot.thetaF(1)+robot.thetaF(2));
  g(1) = robot.parameters(7)*std::cos(robot.thetaF(0)+robot.thetaF(1)) + robot.parameters(8)*std::cos(robot.thetaF(0)+robot.thetaF(1)+robot.thetaF(2));
  g(2) = robot.parameters(8)*std::cos(robot.thetaF(0)+robot.thetaF(1)+robot.thetaF(2));
  return g;
}

} // namespace robot
#endif // CONTROL_UTILITIES_HPP_
