#ifndef ONE_LINK_CONTROL_HPP_
#define ONE_LINK_CONTROL_HPP_

#include "control/controlUtilities.hpp"

namespace robot
{

class OneLinkControl
{
public:
  
  OneLinkControl()
  {
  }

  ~OneLinkControl()
  {
  }

  void executeAdaptiveControl(OneLinkRobot& robot)
  {
    // Calculate and save the error //
    robot.e = robot.thetaF - robot.theta_d;
    robot.de = robot.dthetaF - robot.dtheta_d;

    // Calculate passivity terms //
    ScalarF v = calculateV<OneLinkRobot, ScalarF, ScalarF>(robot, mLambda);
    ScalarF a = calculateA<OneLinkRobot, ScalarF, ScalarF>(robot, mLambda);
    ScalarF r = calculateR<OneLinkRobot, ScalarF, ScalarF>(robot, mLambda);

    // Calculate regressor matrix //
    Matrix1x2F Y = oneLinkRegressor(robot, v, a);

    // Update the estimated parameters //
    Vector2x1F F = -mGamma.inverse()*Y.transpose()*r;
    Vector2x1F p = robot.parameters + F*mDelt;
    robot.parameters = p;

    // Calculate the motor control torque for each link //
    robot.u = robot.motorGearRatio.inverse()*(Y*p - mK*r);
  }

  // One link robust control //
  void executeRobustControl(OneLinkRobot& robot)
  {
    // Calculate and save the error //
    robot.e = robot.thetaF - robot.theta_d;
    robot.de = robot.dthetaF - robot.dtheta_d;

    // Calculate passivity terms //
    ScalarF v = calculateV<OneLinkRobot, ScalarF, ScalarF>(robot, mLambda);
    ScalarF a = calculateA<OneLinkRobot, ScalarF, ScalarF>(robot, mLambda);
    ScalarF r = calculateR<OneLinkRobot, ScalarF, ScalarF>(robot, mLambda);

    // Calculate regressor matrix //
    Matrix1x2F Y = oneLinkRegressor(robot, v, a);

    // Update the estimated parameters //
    Vector2x1F F = Y.transpose()*r;
    float normF = F.norm();

    Vector2x1F delp = normF > mEpsilon ? (-mRho/normF)*F : (-mRho/mEpsilon)*F;

    // Calculate the motor control torque for each link //
    robot.u = robot.motorGearRatio.inverse()*(Y*(robot.parameters + delp) - mK*r);
  }

  // One link robust adaptive control //
  void executeRobustAdaptiveControl(OneLinkRobot& robot)
  {
    // Calculate and save the error //
    robot.e = robot.thetaF - robot.theta_d;
    robot.de = robot.dthetaF - robot.dtheta_d;

    // Calculate passivity terms //
    ScalarF v = calculateV<OneLinkRobot, ScalarF, ScalarF>(robot, mLambda);
    ScalarF a = calculateA<OneLinkRobot, ScalarF, ScalarF>(robot, mLambda);
    ScalarF r = calculateR<OneLinkRobot, ScalarF, ScalarF>(robot, mLambda);

    // Calculate regressor matrix //
    Matrix1x2F Y = oneLinkRegressor(robot, v, a);

    // Claculate the robust term //
    Vector2x1F e;
    e(0) = robot.e(0);
    e(1) = robot.de(0);

    float f = (e.norm()-mDel*mRho)/((1-mDel)*mRho);
    float mu = std::max(0.0f,std::min(1.0f,f));
    
    // Update the estimated parameters //
    Vector2x1F F = -mu*mGamma.inverse()*Y.transpose()*r;
    Vector2x1F p = robot.parameters + F*mDelt;
    robot.parameters = p;

    // Calculate the motor control torque for each link //
    robot.u = robot.motorGearRatio.inverse()*(Y*p - mK*r);
  }

  // One link robust control //
  void executePDControl(OneLinkRobot& robot)
  {
    // Calculate and save the error //
    robot.e = robot.thetaF - robot.theta_d;
    robot.de = robot.dthetaF - robot.dtheta_d;

    // Calculate the motor control torque for each link //
    robot.u = robot.motorGearRatio.inverse()*(mKp*robot.e + mKd*robot.de);
  }

  // Public member variables //
  std::string controlType;
  
  // Adaptive and robust control specific //
  ScalarF mK;
  ScalarF mLambda;

  // Adaptive control specific //
  Matrix2x2F mGamma;
  float mDelt;

  // Robust control specific //
  float mRho;
  float mEpsilon;
  float mDel;

  // PD control specific //
  ScalarF mKp;
  ScalarF mKd;
};

} // namespace robot
#endif // ONE_LINK_CONTROL_HPP_
