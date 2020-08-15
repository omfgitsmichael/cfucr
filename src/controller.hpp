#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

#include <string>
#include <vector>

#include "configurator/configurator.hpp"

namespace robot
{

template <typename Robot, typename Control, typename Filter>
class Controller
{
public:
  Controller(const std::string configFile, Robot& robot, Control& c, Filter& f)
  : mControl(std::move(c)), mFilter(std::move(f))
  {
    // Initialize the control and filter algotithms from the config file //
    ParamsR paramsRobot;
    ParamsC paramsControl;
    ParamsF paramsFilter;

    std::tie(paramsRobot, paramsFilter, paramsControl) = Configurator::initializeParams(configFile);

    initializeRobot(robot, paramsR);
    initializeControl(paramsC);
    initializeFilter(paramsF);
  }

  ~Controller()
  {
  }

  void initializeRobot(Robot& robot, ParamsR params);
  void initializeControl(ParamsC params);
  void initializeFilter(ParamsF params);

  void execute(Robot robot);

private:
  // Controller and Filter Class Variables //
  Control mControl;
  Filter mFilter;
};

} // namespace robot

#endif // CONTROLLER_HPP_
