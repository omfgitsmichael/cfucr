#include "configurator/configurator.hpp"

namespace robot
{
namespace configurator
{

// tinyxml2 helper functions while creating this:
// FirstChildElement("name") grabs the first element of that name. returns a pointer to an XMLElement
// NextSiblingElement("name") grabs the next element of that name. returns a pointer to an XMLElement
// GetText() gets the text value of the element.

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
  ParamsR paramsRobot;

  paramsRobot.numberLinks = robotLinks;

  std::string enableGravityTermsString = robotConfig->FirstChildElement("enableGravityTerms")->GetText();
  paramsRobot.enableGravityTerms = stringToBool(enableGravityTermsString);

  tinyxml2::XMLElement* massElement = filterConfig->FirstChildElement("mass");
  tinyxml2::XMLElement* lengthLinkElement = filterConfig->FirstChildElement("lengthLink");
  tinyxml2::XMLElement* lengthToMassElement = filterConfig->FirstChildElement("lengthToMass");
  tinyxml2::XMLElement* motorInertiaElement = filterConfig->FirstChildElement("motorInertia");
  tinyxml2::XMLElement* gearRatioElement = filterConfig->FirstChildElement("gearRatio");

  for (int i = 0; i < robotLinks; i++)
  {
    paramsFilter.m.push_back(xmlToFloat(massElement, "mass"));
    paramsFilter.l.push_back(xmlToFloat(lengthLinkElement, "lengthLink"));
    paramsFilter.lc.push_back(xmlToFloat(lengthToMassElement, "lengthToMass"));
    paramsFilter.motorInertia.push_back(xmlToFloat(motorInertiaElement, "motorInertia"));
    paramsFilter.gearRatio.push_back(xmlToInt(gearRatioElement, "gearRatio"));
  }

  return paramsRobot;
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

void configureLowPassFilter(tinyxml2::XMLElement* filterConfig, ParamsF& paramsFilter)
{
  std::string filterOrderString = filterConfig->GetText();
  unsigned int filterOrder = std::stoi(filterOrderString);
  paramsFilter.filterOrder = filterOrder;
  
  tinyxml2::XMLElement* alphaQElement = filterConfig->FirstChildElement("alphaQ");
  tinyxml2::XMLElement* alphadQElement = filterConfig->FirstChildElement("alphadQ");

  for (int i = 0; i < filterOrder; i++)
  {
    paramsFilter.alphaQ.push_back(xmlToFloat(alphaQElement, "alphaQ"));
    paramsFilter.alphadQ.push_back(xmlToFloat(alphadQElement, "alphadQ"));
  }
}

ParamsC configureControl(tinyxml2::XMLElement* controlConfig, const unsigned int& robotLinks)
{
  ParamsC paramsControl;

  paramsControl.numberLinks = robotLinks;

  std::string controlType = controlConfig->FirstChildElement("controlType")->GetText();
  paramsControl.controlType = controlType;

  if (controlType.compare("adpativeControl") > 0)
  {
    configureAdaptiveControl(controlConfig, paramsControl);
  }
  else if (controlType.compare("robustControl") > 0)
  {
    // Populate Robust Control Configs //
  }
  else if (controlType.compare("pdControl") > 0)
  {
    // Populate PD Control Configs //
  }

  return paramsControl;
}

void configureAdaptiveControl(tinyxml2::XMLElement* controlConfig, ParamsC& paramsControl)
{
  tinyxml2::XMLElement* samplingRateElement = controlConfig->FirstChildElement("samplingRate");
  std::string deltString = samplingRateElement->FirstChildElement("delt")->GetText();
  paramsControl.delt = std::stof(deltString);

  tinyxml2::XMLElement* linearGainsElement = controlConfig->FirstChildElement("linearGains");
  tinyxml2::XMLElement* kAdaptiveElement = linearGainsElement->FirstChildElement("k");
  tinyxml2::XMLElement* lambdaElement = linearGainsElement->FirstChildElement("lambda");
  
  int n = 0;
  // Place the Linear Gains Inside of the Params //
  for (int i = 0; i < paramsControl.numberLinks; i++)
  {
    paramsControl.kAdaptive.push_back(xmlToFloat(kAdaptiveElement, "k"));
    paramsControl.lambda.push_back(xmlToFloat(lambdaElement, "lambda"));
    n = n+i+1;
  }

  tinyxml2::XMLElement* rateOfAdaptivityElement = controlConfig->FirstChildElement("rateOfAdaptivity");
  tinyxml2::XMLElement* gammaElement = rateOfAdaptivityElement->FirstChildElement("gamma");
  for (int i = 0; i < n; i++)
  {
    paramsControl.gamma.push_back(xmlToFloat(gammaElement, "gamma"));
  }
}

bool stringToBool(std::string text)
{
  bool result = false;
  if (text.compare("true") || text.compare("1"))
  {
    result = true;
  }

  return result;
}

int xmlToInt(tinyxml2::XMLElement* xmlElement, std::string elementString)
{
  std::string elementText = xmlElement->GetText();
  int value = std::stoi(elementText);
  xmlElement = xmlElement->NextSiblingElement(elementString);

  return value;
}

float xmlToFloat(tinyxml2::XMLElement* xmlElement, std::string elementString)
{
  std::string elementText = xmlElement->GetText();
  float value = std::stof(elementText);
  xmlElement = xmlElement->NextSiblingElement(elementString);

  return value;
}

} // namespace configurator
} // namespace robot