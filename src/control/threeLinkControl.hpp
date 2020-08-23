#ifndef THREE_LINK_CONTROL_HPP_
#define THREE_LINK_CONTROL_HPP_

#include "control/controlUtilities.hpp"

namespace robot
{

class ThreeLinkControl
{
public:
  
  ThreeLinkControl()
  {
  }

  ~ThreeLinkControl()
  {
  }

  void executeAdaptiveControl(sharedThreeLinkRobot& robot)
  {
    // Calculate and save the error //
    robot->e = robot->thetaF - robot->theta_d;
    robot->de = robot->dthetaF - robot->dtheta_d;

    // Calculate passivity terms //
    Vector3x1F v = calculateV<sharedThreeLinkRobot, Vector3x1F, Matrix3x3F>(robot, mLambda);
    Vector3x1F a = calculateA<sharedThreeLinkRobot, Vector3x1F, Matrix3x3F>(robot, mLambda);
    Vector3x1F r = calculateR<sharedThreeLinkRobot, Vector3x1F, Matrix3x3F>(robot, mLambda);

    // Calculate regressor matrix //
    Matrix3x9F Y = threeLinkRegressor(robot, v, a);

    // Update the estimated parameters //
    Vector9x1F F = -mGamma.inverse()*Y.transpose()*r;
    Vector9x1F p = robot->parameters + F*mDelt;
    robot->parameters = p;

    // Calculate the motor control torque for each link //
    robot->u = robot->motorGearRatio.inverse()*(Y*p - mK*r);
  }

  // Three link robust control //
  void executeRobustControl(sharedThreeLinkRobot& robot)
  {
    // Calculate and save the error //
    robot->e = robot->thetaF - robot->theta_d;
    robot->de = robot->dthetaF - robot->dtheta_d;

    // Calculate passivity terms //
    Vector3x1F v = calculateV<sharedThreeLinkRobot, Vector3x1F, Matrix3x3F>(robot, mLambda);
    Vector3x1F a = calculateA<sharedThreeLinkRobot, Vector3x1F, Matrix3x3F>(robot, mLambda);
    Vector3x1F r = calculateR<sharedThreeLinkRobot, Vector3x1F, Matrix3x3F>(robot, mLambda);

    // Calculate regressor matrix //
    Matrix3x9F Y = threeLinkRegressor(robot, v, a);

    // Update the estimated parameters //
    Vector9x1F F = Y.transpose()*r;
    float normF = std::sqrt(F(0)*F(0) + F(1)*F(1) + F(2)*F(2) + F(3)*F(3) + F(4)*F(4) + F(5)*F(5) + F(6)*F(6) + F(7)*F(7) + F(8)*F(8));

    Vector9x1F delp = normF > mEpsilon ? (-mRho/normF)*F : (-mRho/mEpsilon)*F;

    // Calculate the motor control torque for each link //
    robot->u = robot->motorGearRatio.inverse()*(Y*(robot->parameters + delp) - mK*r);
  }

  // Three link robust control //
  void executePDControl(sharedThreeLinkRobot& robot)
  {
    // Calculate and save the error //
    robot->e = robot->thetaF - robot->theta_d;
    robot->de = robot->dthetaF - robot->dtheta_d;

    // Calculate the motor control torque for each link //
    robot->u = robot->motorGearRatio.inverse()*(mKp*robot->e + mKd*robot->de);
  }

  // Public member variables //
  std::string controlType;
  
  // Adaptive and robust control specific //
  Matrix3x3F mK;
  Matrix3x3F mLambda;

  // Adaptive control specific //
  Matrix9x9F mGamma;
  float mDelt;

  // Robust control specific //
  float mRho;
  float mEpsilon;

  // PD control specific //
  Matrix3x3F mKp;
  Matrix3x3F mKd;
};

} // namespace robot
#endif // THREE_LINK_CONTROL_HPP_
