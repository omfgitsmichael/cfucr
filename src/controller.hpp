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
    mControl = std::make_shared<Control>();
    mFilter = std::make_shared<Filter>();

    // Initialize the control and filter algotithms from the config file //
    ParamsR paramsRobot;
    ParamsC paramsControl;
    ParamsF paramsFilter;

    std::tie(paramsRobot, paramsFilter, paramsControl) = configurator::initializeParams(configFile);

    initializeRobot(robot, paramsRobot);
    initializeControl(paramsControl);
    initializeFilter(paramsFilter);
  }

  ~Controller()
  {
  }

  void execute(Robot& robot);

  void initializeRobot(Robot& robot, ParamsR& params);
  void initializeOneLink(Robot& robot, ParamsR& params, std::vector<float> I);
  void initializeTwoLink(Robot& robot, ParamsR& params, std::vector<float> I);
  void initializeThreeLink(Robot& robot, ParamsR& params, std::vector<float> I);

  void initializeControl(ParamsC& params);
  void initializeAdaptiveControl(ParamsC& params);
  void initializeRobustControl(ParamsC& params);
  void initializePDControl(ParamsC& params);

  void initializeFilter(ParamsF& params);
  void initializeLowPassFilter(ParamsF& params);

private:
  // Controller and Filter Class Variables //
  std::shared_ptr<Control> mControl;
  std::shared_ptr<Filter> mFilter;
};

} // namespace robot

#endif // CONTROLLER_HPP_
