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

  void executeAdaptiveControl(ThreeLinkRobot& robot)
  {
    // Calculate and save the error //
    robot.e = robot.thetaF - robot.theta_d;
    robot.de = robot.dthetaF - robot.dtheta_d;

    // Calculate passivity terms //
    Vector3x1F v = calculateV<ThreeLinkRobot, Vector3x1F, Matrix3x3F>(robot, mLambda);
    Vector3x1F a = calculateA<ThreeLinkRobot, Vector3x1F, Matrix3x3F>(robot, mLambda);
    Vector3x1F r = calculateR<ThreeLinkRobot, Vector3x1F, Matrix3x3F>(robot, mLambda);

    // Calculate regressor matrix //
    Matrix3x9F Y = threeLinkRegressor(robot, v, a);

    // Update the estimated parameters //
    Vector9x1F F = -mGamma.inverse()*Y.transpose()*r;
    Vector9x1F p = robot.parameters + F*mDelt;
    robot.parameters = p;

    // Calculate the motor control torque for each link //
    robot.u = robot.motorGearRatio.inverse()*(Y*p - mK*r);
  }

  // Three link robust control //
  void executeRobustControl(ThreeLinkRobot& robot)
  {
    // Calculate and save the error //
    robot.e = robot.thetaF - robot.theta_d;
    robot.de = robot.dthetaF - robot.dtheta_d;

    // Calculate passivity terms //
    Vector3x1F v = calculateV<ThreeLinkRobot, Vector3x1F, Matrix3x3F>(robot, mLambda);
    Vector3x1F a = calculateA<ThreeLinkRobot, Vector3x1F, Matrix3x3F>(robot, mLambda);
    Vector3x1F r = calculateR<ThreeLinkRobot, Vector3x1F, Matrix3x3F>(robot, mLambda);

    // Calculate regressor matrix //
    Matrix3x9F Y = threeLinkRegressor(robot, v, a);

    // Update the estimated parameters //
    Vector9x1F F = Y.transpose()*r;
    float normF = F.norm();

    Vector9x1F delp = normF > mEpsilon ? (-mRho/normF)*F : (-mRho/mEpsilon)*F;

    // Calculate the motor control torque for each link //
    robot.u = robot.motorGearRatio.inverse()*(Y*(robot.parameters + delp) - mK*r);
  }
  
  // Three link robust adaptive control //
  void executeRobustAdaptiveControl(ThreeLinkRobot& robot)
  {
    // Calculate and save the error //
    robot.e = robot.thetaF - robot.theta_d;
    robot.de = robot.dthetaF - robot.dtheta_d;

    // Calculate passivity terms //
    Vector3x1F v = calculateV<ThreeLinkRobot, Vector3x1F, Matrix3x3F>(robot, mLambda);
    Vector3x1F a = calculateA<ThreeLinkRobot, Vector3x1F, Matrix3x3F>(robot, mLambda);
    Vector3x1F r = calculateR<ThreeLinkRobot, Vector3x1F, Matrix3x3F>(robot, mLambda);

    // Calculate regressor matrix //
    Matrix3x9F Y = threeLinkRegressor(robot, v, a);

    // Claculate the robust term //
    Vector6x1F e;
    e(0) = robot.e(0);
    e(1) = robot.e(1);
    e(2) = robot.e(2);
    e(3) = robot.de(0);
    e(4) = robot.de(1);
    e(5) = robot.de(2);

    float f = (e.norm()-mDel*mRho)/((1-mDel)*mRho);
    float mu = std::max(0.0f,std::min(1.0f,f));

    // Update the estimated parameters //
    Vector9x1F F = -mu*mGamma.inverse()*Y.transpose()*r;
    Vector9x1F p = robot.parameters + F*mDelt;
    robot.parameters = p;

    // Calculate the motor control torque for each link //
    robot.u = robot.motorGearRatio.inverse()*(Y*p - mK*r);
  }

  // Three link robust control //
  void executePDControl(ThreeLinkRobot& robot)
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
  Matrix3x3F mK;
  Matrix3x3F mLambda;

  // Adaptive control specific //
  Matrix9x9F mGamma;
  float mDelt;

  // Robust control specific //
  float mRho;
  float mEpsilon;
  float mDel;

  // PD control specific //
  Matrix3x3F mKp;
  Matrix3x3F mKd;
};

} // namespace robot
#endif // THREE_LINK_CONTROL_HPP_
