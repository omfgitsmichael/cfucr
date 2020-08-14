#ifndef CONFIGURATOR_HPP_
#define CONFIGURATOR_HPP_

#include <string>

namespace robot
{

namespace configurator
{

std::tuple<ParamsR, ParamsC, ParamsF> initializeParams(const std::string configFile);

} // namespace configurator
} // namespace robot

#endif // CONFIGURATOR_HPP_