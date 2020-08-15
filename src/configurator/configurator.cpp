#include "configurator/configurator.hpp"

namespace robot
{
namespace configurator
{

std::tuple<ParamsR, ParamsC, ParamsF> initializeParams(const std::string configFile)
{
  // Load and Parse the Config File //
  tinyxml2::XMLDocument config;
  config.LoadFile(configFile);
  
  // Get the Root Element of the Config File //
  tinyxml::XMLElement* controllerConfig = config.FirstChildElement("Controller");

  const unsigned int robotLinks = controllerConfig->FirstChildElement("numberLinks");

  tinyxml2::XMLElement* robotConfig = controllerConfig->FirstChildElement("robot");
  tinyxml2::XMLElement* controlConfig = controllerConfig->FirstChildElement("control");
  tinyxml2::XMLElement* filterConfig = controllerConfig->FirstChildElement("filter");
}

} // namespace configurator
} // namespace robot