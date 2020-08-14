#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

#include <string>

namespace robot
{

template <typename Robot, typename Control, typename Filter>
class Controller
{
public:
  Controller(const std::string configFile, Robot& robot, Control& c, Filter& f)
  : control(std::move(C)), filter(std::move(f))
  {
    // Initialize the control and filter algotithms from the config file //
    ParamsR paramsR;
    ParamsC paramsC;
    ParamsF paramsF;

    std::tie(ParamsR, ParamsC, ParamsF) = Configurator::initializeParams(configFile);

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
  Control control;
  Filter filter;
};

} // namespace robot

#endif // CONTROLLER_HPP_
