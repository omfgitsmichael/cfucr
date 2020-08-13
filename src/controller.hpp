#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

namespace robot
{

template <typename Robot, typename Controller, typename Filter>
class Controller
{
public:
  Controller()
  {
   // Initialize the Filter and Controller //
  }

  ~Controller()
  {
  }

  void execute(Robot robot);

private:
  Controller controller;
  Filter filter;
};

} // namespace robot

#endif // CONTROLLER_HPP_
