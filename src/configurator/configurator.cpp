#include "configurator/configurator.hpp"

namespace robot
{
namespace configurator
{

// tinyxml2 helper functions while creating this:
// FirstChildElement("name") grabs the first element of that name. returns a pointer to an XMLElement
// NextSiblingElement("name") grabs the next element of that name. returns a pointer to an XMLElement
// GetText() gets the text value of the element.

// Need to create a function that'll convert text of `true` or 'false` into a boolean true or false.
// Need to create a function that'll loop through sibling elements of the same name.
// Need to create functions to configure the robot, filter, and control.

std::tuple<ParamsR, ParamsF, ParamsC> initializeParams(const std::string configFile)
{
  // Load and Parse the Config File //
  tinyxml2::XMLDocument config;
  config.LoadFile(configFile);

  // Get the Root Element of the Config File //
  tinyxml::XMLElement* controllerConfig = config.FirstChildElement("Controller");

  std::string robotLinksString = controllerConfig->FirstChildElement("numberLinks")->GetText();
  const unsigned int robotLinks = std::stoi(robotLinksString);

  tinyxml2::XMLElement* robotConfig = controllerConfig->FirstChildElement("robot");
  tinyxml2::XMLElement* controlConfig = controllerConfig->FirstChildElement("control");
  tinyxml2::XMLElement* filterConfig = controllerConfig->FirstChildElement("filter");

  ParamsR paramsRobot = configureRobot(robotConfig, robotLinks);
  ParamsF paramsFilter = configureFilter(filterConfig, robotLinks);
  ParamsC paramsControl = configureControl(controlConfig, robotLinks);

  return std::make_tuple(paramsRobot, paramsFilter, paramsControl);
}

ParamsR configureRobot(tinyxml2::XMLElement* robotConfig, const unsigned int& robotLinks)
{

}

ParamsF configureFilter(tinyxml2::XMLElement* filterConfig, const unsigned int& robotLinks)
{
  ParamsF paramsFilter;

  paramsFilter.numberLinks = robotLinks;

  std::string filterType = filterConfig->FirstChildElement("filterType")->GetText();
  paramsFilter.filterType = filterType;

  if (filterType.compare("lowPassFilter") > 0)
  {
    configureLowPassFilter(filterConfig, paramsFilter);
  }

  return paramsFilter;
}

configureLowPassFilter(tinyxml2::XMLElement* filterConfig, ParamsF& paramsFilter)
{
  std::string filterOrderString = filterConfig->GetText();
  unsigned int filterOrder = std::stoi(filterOrderString);
  paramsFilter.filterOrder = filterOrder;
  
  tinyxml2::XMLElement alphaQElement = filterConfig->FirstChildElement("alphaQ");
  tinyxml2::XMLElement alphadQElement = filterConfig->FirstChildElement("alphadQ");

  for (int i = 0; i < filterOrder; i++)
  {
    std::string alphaQString = alphaQElement->GetText();
    float alphaQ = std::stof(alphaQString);
    paramsFilter.alphaQ.push_back(alphaQ);
    alphaQElement = alphaQElement->NextSiblingElement("alphaQ");

    std::string alphadQString = alphadQElement->GetText();
    float alphadQ = std::stof(alphadQString);
    paramsFilter.alphadQ.push_back(alphadQ);
    alphadQElement = alphadQElement->NextSiblingElement("alphadQ");
  }
}

ParamsC configureControl(tinyxml2::XMLElement* controlConfig, const unsigned int& robotLinks)
{

}

} // namespace configurator
} // namespace robot