#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "configurator/configurator.hpp"

namespace robot
{

static const float GRAVITY = 9.81f;

template <typename Robot, typename Control, typename Filter>
class Controller
{
public:
  Controller(const char* configFile, Robot& robot)
  {
    // Initialize the control and filter algotithms from the config file //
    ParamsR paramsRobot;
    ParamsC paramsControl;
    ParamsF paramsFilter;

    std::tie(paramsRobot, paramsFilter, paramsControl) = configurator::initializeParams(configFile);

    initializeRobot(robot, paramsRobot);
    initializeFilter(paramsFilter);
    initializeControl(paramsControl);
  }

  ~Controller()
  {
  }

  void execute(Robot& robot);

  void initializeRobot(Robot& robot, ParamsR& params)
  {
    robot->numberLinks = params.numberLinks;

    std::vector<float> I;

    for (unsigned int i = 0; i < robot->numberLinks; i++)
    {
      robot->theta(i) = 0.0f;
      robot->dtheta(i) = 0.0f;

      robot->thetaF(i) = 0.0f;
      robot->dthetaF(i) = 0.0f;

      robot->e(i) = 0.0f;
      robot->de(i) = 0.0f;

      robot->theta_d(i) = 0.0f;
      robot->dtheta_d(i) = 0.0f;
      robot->ddtheta_d(i) = 0.0f;

      robot->u(i) = 0.0f;
      
      for (unsigned int j = 0; j < robot->numberLinks; j++)
      {
        robot->motorGearRatio(i,j) = (i == j) ? params.gearRatio[i] : 0.0f;
      }
      
      // Total inertia for link i = r^2*Jm + (1/3)*m*l^2 //
      I.push_back(params.gearRatio[i]*params.gearRatio[i]*params.motorInertia[i] + (1.0f/3.0f)*params.m[i]*params.l[i]*params.l[i]);
    }

    if (robot->numberLinks == 1)
    {
      initializeOneLink(robot, params, I);
    }
    else if (robot->numberLinks == 2)
    {
      initializeTwoLink(robot, params, I);
    }
    else if (robot->numberLinks == 3)
    {
      initializeThreeLink(robot, params, I);
    }
  }

  void initializeOneLink(Robot& robot, ParamsR& params, std::vector<float> I)
  {  
    robot->parameters(0) = params.m[0]*params.lc[0]*params.lc[0] + I[0];

    if (params.enableGravityTerms)
    {
      robot->parameters(1) = params.m[0]*params.lc[0]*GRAVITY;
    }
    else
    {
      robot->parameters(1) = 0.0f;
    }
  }

  void initializeTwoLink(Robot& robot, ParamsR& params, std::vector<float> I)
  {
    robot->parameters(0) = params.m[0]*params.lc[0]*params.lc[0] + params.m[1]*params.l[0]*params.l[0] + I[0];
    robot->parameters(1) = params.m[1]*params.lc[1]*params.lc[1] + I[1];
    robot->parameters(2) = params.m[1]*params.l[0]*params.lc[1];

    if (params.enableGravityTerms)
    {
      robot->parameters(3) = (params.m[0]*params.lc[0] + params.m[1]*params.l[0])*GRAVITY;
      robot->parameters(4) = params.m[1]*params.lc[1]*GRAVITY;
    }
    else
    {
      robot->parameters(3) = 0.0f;
      robot->parameters(4) = 0.0f;
    }
  }

  void initializeThreeLink(Robot& robot, ParamsR& params, std::vector<float> I)
  {
    robot->parameters(0) = params.m[0]*params.lc[0]*params.lc[0] + params.m[1]*params.l[0]*params.l[0] + params.m[2]*params.l[0]*params.l[0] + I[0];
    robot->parameters(1) = params.m[1]*params.lc[1]*params.lc[1] + params.m[2]*params.l[1]*params.l[1] + I[1];
    robot->parameters(2) = params.m[2]*params.lc[2]*params.lc[2] + I[2];
    robot->parameters(3) = params.m[1]*params.l[0]*params.lc[1] + params.m[2]*params.l[0]*params.l[1];
    robot->parameters(4) = params.m[2]*params.l[0]*params.lc[2];
    robot->parameters(5) = params.m[2]*params.l[1]*params.lc[2];

    if (params.enableGravityTerms)
    {
      robot->parameters(6) = (params.m[0]*params.lc[0] + params.m[1]*params.l[0] + params.m[2]*params.l[0])*GRAVITY;
      robot->parameters(7) = (params.m[1]*params.lc[1] + params.m[2]*params.l[1])*GRAVITY;
      robot->parameters(8) = params.m[2]*params.lc[2]*GRAVITY;
    }
    else
    {
      robot->parameters(6) = 0.0f;
      robot->parameters(7) = 0.0f;
      robot->parameters(8) = 0.0f;
    }
  }

  void initializeControl(ParamsC& params)
  {
    mControl.controlType = params.controlType;
    if (mControl.controlType.compare("adpativeControl") > 0)
    {
      initializeAdaptiveControl(params);
    }
    else if (mControl.controlType.compare("robustControl") > 0)
    {
      initializeRobustControl(params);
    }
    else if (mControl.controlType.compare("pdControl") > 0)
    {
      initializePDControl(params);
    }
  }

  void initializeAdaptiveControl(ParamsC& params)
  {
    mControl.mDelt = params.delt;

    for (unsigned int i = 0; i < params.numberLinks; i++)
    {
      for (unsigned int j = 0; j < params.numberLinks; j++)
      {
        mControl.mK(i,j) = (i == j) ? params.k[i] : 0.0f;
        mControl.mLambda(i,j) = (i == j) ? params.lambda[i] : 0.0f;
      }
    }

    for (unsigned int i = 0; i < params.gamma.size(); i++)
    {
      for (unsigned int j = 0; j < params.gamma.size(); j++)
      {
        mControl.mGamma(i,j) = (i == j) ? params.gamma[i] : 0.0f;
      }
    }
  }

  void initializeRobustControl(ParamsC& params)
  {
    mControl.mRho = params.rho;
    mControl.mEpsilon = params.epsilon;

    for (unsigned int i = 0; i < params.numberLinks; i++)
    {
      for (unsigned int j = 0; j < params.numberLinks; j++)
      {
        mControl.mK(i,j) = (i == j) ? params.k[i] : 0.0f;
        mControl.mLambda(i,j) = (i == j) ? params.lambda[i] : 0.0f;
      }
    }
  }

  void initializePDControl(ParamsC& params)
  {
    for (unsigned int i = 0; i < params.numberLinks; i++)
    {
      for (unsigned int j = 0; j < params.numberLinks; j++)
      {
        mControl.mKp(i,j) = (i == j) ? params.kp[i] : 0.0f;
        mControl.mKd(i,j) = (i == j) ? params.kd[i] : 0.0f;
      }
    }
  }

  void initializeFilter(ParamsF& params)
  {
    mFilter.filterType = params.filterType;

    if (mFilter.filterType.compare("lowPassFilter") > 0)
    {
      initializeLowPassFilter(params);
    }
  }

  void initializeLowPassFilter(ParamsF& params)
  {
    mFilter.mFilterOrder = params.filterOrder;

    // Nested for loop between the number of robot links and filter order and initialize //
    for (unsigned int i = 0; i < params.numberLinks; i++)
    {
      mFilter.mPreviousIntegralOutputQ.emplace_back();
      for (unsigned int j = 0; j < mFilter.mFilterOrder; j++)
      {
        mFilter.mPreviousIntegralOutputQ[i].push_back(0.0f);
        mFilter.mPreviousIntegralOutputdQ[i].push_back(0.0f);
      }
    }

    for (unsigned int i = 0; i < mFilter.mFilterOrder; i++)
    {
      mFilter.mAlphaQ.push_back(params.alphaQ[i]);
      mFilter.mAlphadQ.push_back(params.alphadQ[i]);
    }
  }

private:
  // Controller and Filter Class Variables //
  Control mControl;
  Filter mFilter;
};

} // namespace robot

#endif // CONTROLLER_HPP_
