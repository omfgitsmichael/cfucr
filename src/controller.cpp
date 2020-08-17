#include "controller.hpp"

namespace robot
{

template<typename Robot>
void Controller::execute(Robot& robot)
{
  // Execute the filter and control algorithms //
  mFilter->execute(robot);
  mControl->execute(robot);
}

template <typename Robot>
void Controller::initializeRobot(Robot& robot, ParamsR& params)
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
    
    for (j = 0; j < robot->numberLinks; j++)
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

template <typename Robot>
void Controller::initializeOneLink(Robot& robot, ParamsR& params, std::vector<float> I)
{  
  robot->p(0) = params.m[0]*params.lc[0]*params.lc[0] + I[0];

  if (params.enableGravityTerms)
  {
    robot->p(1) = params.m[0]*params.lc[0]*GRAVITY;
  }
  else
  {
    robot->p(1) = 0.0f;
  }
}

template <typename Robot>
void Controller::initializeTwoLink(Robot& robot, ParamsR& params, std::vector<float> I)
{
  robot->p(0) = params.m[0]*params.lc[0]*params.lc[0] + params.m[1]*params.l[0]*params.l[0] + I[0];
  robot->p(1) = params.m[1]*params.lc[1]*params.lc[1] + I[1];
  robot->p(2) = params.m[1]*params.l[0]*params.lc[1];

  if (params.enableGravityTerms)
  {
    robot->p(3) = (params.m[0]*params.lc[0] + params.m[1]*params.l[0])*GRAVITY;
    robot->p(4) = params.m[1]*params.lc[1]*GRAVITY;
  }
  else
  {
    robot->p(3) = 0.0f;
    robot->p(4) = 0.0f;
  }
}

template <typename Robot>
void Controller::initializeThreeLink(Robot& robot, ParamsR& params, std::vector<float> I)
{
  robot->p(0) = params.m[0]*params.lc[0]*params.lc[0] + params.m[1]*params.l[0]*params.l[0] + params.m[2]*params.l[0]*params.l[0] + I[0];
  robot->p(1) = params.m[1]*params.lc[1]*params.lc[1] + params.m[2]*params.l[1]*params.l[1] + I[1];
  robot->p(2) = params.m[2]*params.lc[2]*params.lc[2] + I[2];
  robot->p(3) = params.m[1]*params.l[0]*params.lc[1] + params.m[2]*params.l[0]*params.l[1];
  robot->p(4) = params.m[2]*params.l[0]*params.lc[2];
  robot->p(5) = params.m[2]*params.l[1]*params.lc[2];

  if (params.enableGravityTerms)
  {
    robot->p(6) = (params.m[0]*params.lc[0] + params.m[1]*params.l[0] + params.m[2]*params.l[0])*GRAVITY;
    robot->p(7) = (params.m[1]*params.lc[1] + params.m[2]*params.l[1])*GRAVITY;
    robot->p(8) = params.m[2]*params.lc[2]*GRAVITY;
  }
  else
  {
    robot->p(6) = 0.0f;
    robot->p(7) = 0.0f;
    robot->p(8) = 0.0f;
  }
}

void Controller::initializeControl(ParamsC& params)
{
  if (params.controlType.compare("adpativeControl") > 0)
  {
    initializeAdaptiveControl(params);
  }
}

void Controller::initializeAdaptiveControl(ParamsC& params)
{
  mControl->mDelt = params.delt;

  for (unsigned int i = 0; i < params.numberLinks; i++)
  {
    for (unsigned int j = 0; j < params.numberLinks; j++)
    {
      mControl->mK(i,j) = (i == j) ? params.kAdaptive[i] : 0.0f;
      mControl->mLambda(i,j) = (i == j) ? params.lambda[i] : 0.0f;
    }
  }

  for (unsigned int i = 0; i < params.gamma.size(); i++)
  {
    for (unsigned int j = 0; j < params.gamma.size(); j++)
    {
      mControl->mGamma(i,j) = (i == j) ? params.gamma[i] : 0.0f;
    }
  }
}

void Controller::initializeFilter(ParamsF& params)
{
  if (params.filterType.compare("lowPassFilter") > 0)
  {
    initializeLowPassFilter(params);
  }
}

void Controller::initializeLowPassFilter(ParamsF& params)
{
  mFilter->mFilterOrder = params.filterOrder;

  // Nested for loop between the number of robot links and filter order and initialize //
  for (unsigned int i = 0; i < params.numberLinks; i++)
  {
    mFilter->mPreviousIntegralOutputQ.emplace_back();
    for (unsigned int j = 0; j < mFilter->mFilterOrder; j++)
    {
      mFilter->mPreviousIntegralOutputQ[i].push_back(0.0f);
      mFilter->mPreviousIntegralOutputdQ[i].push_back(0.0f);
    }
  }

  for (unsigned int i = 0; i < mFilter->mFilterOrder; i++)
  {
    mFilter->mAlphaQ.push_back(params.alphaQ[i]);
    mFilter->mAlphadQ.push_back(params.alphadQ[i]);
  }
}

} // namespace robot