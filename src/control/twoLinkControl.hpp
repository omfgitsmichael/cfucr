#ifndef TWO_LINK_CONTROL_HPP_
#define TWO_LINK_CONTROL_HPP_

#include "control/controlUtilities.hpp"

namespace robot
{

class TwoLinkControl
{
public:
  
  TwoLinkControl()
  {
  }

  ~TwoLinkControl()
  {
  }

  void executeAdaptiveControl(sharedTwoLinkRobot& robot)
  {
    // Calculate and save the error //
    robot->e = robot->thetaF - robot->theta_d;
    robot->de = robot->dthetaF - robot->dtheta_d;

    // Calculate passivity terms //
    Vector2x1F v = calculateV<sharedTwoLinkRobot, Vector2x1F, Matrix2x2F>(robot, mLambda);
    Vector2x1F a = calculateA<sharedTwoLinkRobot, Vector2x1F, Matrix2x2F>(robot, mLambda);
    Vector2x1F r = calculateR<sharedTwoLinkRobot, Vector2x1F, Matrix2x2F>(robot, mLambda);

    // Calculate regressor matrix //
    Matrix2x5F Y = twoLinkRegressor(robot, v, a);

    // Update the estimated parameters //
    Vector5x1F F = -mGamma.inverse()*Y.transpose()*r;
    Vector5x1F p = robot->parameters + F*mDelt;
    robot->parameters = p;

    // Calculate the motor control torque for each link //
    robot->u = robot->motorGearRatio.inverse()*(Y*p - mK*r);
  }

  // Three link robust control //
  void executeRobustControl(sharedTwoLinkRobot& robot)
  {
    // Calculate and save the error //
    robot->e = robot->thetaF - robot->theta_d;
    robot->de = robot->dthetaF - robot->dtheta_d;

    // Calculate passivity terms //
    Vector2x1F v = calculateV<sharedTwoLinkRobot, Vector2x1F, Matrix2x2F>(robot, mLambda);
    Vector2x1F a = calculateA<sharedTwoLinkRobot, Vector2x1F, Matrix2x2F>(robot, mLambda);
    Vector2x1F r = calculateR<sharedTwoLinkRobot, Vector2x1F, Matrix2x2F>(robot, mLambda);

    // Calculate regressor matrix //
    Matrix2x5F Y = twoLinkRegressor(robot, v, a);

    // Update the estimated parameters //
    Vector5x1F F = Y.transpose()*r;
    float normF = std::sqrt(F(0)*F(0) + F(1)*F(1) + F(2)*F(2) + F(3)*F(3) + F(4)*F(4));

    Vector5x1F delp = normF > mEpsilon ? (-mRho/normF)*F : (-mRho/mEpsilon)*F;

    // Calculate the motor control torque for each link //
    robot->u = robot->motorGearRatio.inverse()*(Y*(robot->parameters + delp) - mK*r);
  }

  // Two link robust control //
  void executePDControl(sharedTwoLinkRobot& robot)
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
  Matrix2x2F mK;
  Matrix2x2F mLambda;

  // Adaptive control specific //
  Matrix5x5F mGamma;
  float mDelt;

  // Robust control specific //
  float mRho;
  float mEpsilon;

  // PD control specific //
  Matrix2x2F mKp;
  Matrix2x2F mKd;
};

} // namespace robot
#endif // TWO_LINK_CONTROL_HPP_