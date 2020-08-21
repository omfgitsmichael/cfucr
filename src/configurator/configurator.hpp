#ifndef CONFIGURATOR_HPP_
#define CONFIGURATOR_HPP_

#include <string>
#include <tuple>
#include <vector>

#include "thirdParty/tinyxml2/tinyxml2.h"

namespace robot
{

struct ParamsR
{
  unsigned int numberLinks = 0;
  
  // Robot Parameters //
  bool enableGravityTerms = false;

  std::vector<float> m;
  std::vector<float> l;
  std::vector<float> lc;
  
  // Motor Parameters //
  std::vector<float> motorInertia;
  std::vector<unsigned int> gearRatio;
};

struct ParamsC
{
  unsigned int numberLinks = 0;

  // Control Parameters //
  std::string controlType;

  // Linear Control Gain Adaptive/Robust //
  std::vector<float> k;
  
  // Robust Control //
  float rho = 0.0f;
  float epsilon = 0.0f;
  
  // Adaptive Control //
  float delt = 0.0f;
  std::vector<float> lambda;
  std::vector<float> gamma;

  // PD Control //
  std::vector<float> kp;
  std::vector<float> kd;
};

struct ParamsF
{
  std::string filterType;

  unsigned int numberLinks = 0;

  // Filter Parameters //
  std::vector<float> alphaQ;
  std::vector<float> alphadQ;
  
  unsigned int filterOrder;
};

namespace configurator
{

inline bool stringToBool(std::string text)
{
  bool result = false;
  if (text.compare("true") || text.compare("1"))
  {
    result = true;
  }

  return result;
}

inline int xmlToInt(tinyxml2::XMLElement* xmlElement, const char* elementString)
{
  std::string elementText = xmlElement->GetText();
  int value = std::stoi(elementText);
  xmlElement = xmlElement->NextSiblingElement(elementString);

  return value;
}

inline float xmlToFloat(tinyxml2::XMLElement* xmlElement, const char* elementString)
{
  std::string elementText = xmlElement->GetText();
  float value = std::stof(elementText);
  xmlElement = xmlElement->NextSiblingElement(elementString);

  return value;
}

inline ParamsR configureRobot(tinyxml2::XMLElement* robotConfig, unsigned int& robotLinks)
{
  ParamsR paramsRobot;

  paramsRobot.numberLinks = robotLinks;

  std::string enableGravityTermsString = robotConfig->FirstChildElement("enableGravityTerms")->GetText();
  paramsRobot.enableGravityTerms = stringToBool(enableGravityTermsString);

  tinyxml2::XMLElement* massElement = robotConfig->FirstChildElement("mass");
  tinyxml2::XMLElement* lengthLinkElement = robotConfig->FirstChildElement("lengthLink");
  tinyxml2::XMLElement* lengthToMassElement = robotConfig->FirstChildElement("lengthToMass");
  tinyxml2::XMLElement* motorInertiaElement = robotConfig->FirstChildElement("motorInertia");
  tinyxml2::XMLElement* gearRatioElement = robotConfig->FirstChildElement("gearRatio");

  for (int i = 0; i < robotLinks; i++)
  {
    paramsRobot.m.push_back(xmlToFloat(massElement, "mass"));
    paramsRobot.l.push_back(xmlToFloat(lengthLinkElement, "lengthLink"));
    paramsRobot.lc.push_back(xmlToFloat(lengthToMassElement, "lengthToMass"));
    paramsRobot.motorInertia.push_back(xmlToFloat(motorInertiaElement, "motorInertia"));
    paramsRobot.gearRatio.push_back(xmlToInt(gearRatioElement, "gearRatio"));
  }

  return paramsRobot;
}

inline void configureLowPassFilter(tinyxml2::XMLElement* filterConfig, ParamsF& paramsFilter)
{
  std::string filterOrderString = filterConfig->GetText();
  unsigned int filterOrder = std::stoi(filterOrderString);
  paramsFilter.filterOrder = filterOrder;
  
  tinyxml2::XMLElement* alphaQElement = filterConfig->FirstChildElement("alphaQ");
  tinyxml2::XMLElement* alphadQElement = filterConfig->FirstChildElement("alphadQ");
  
  // Place the Alpha Terms Inside of the Params //
  for (int i = 0; i < filterOrder; i++)
  {
    paramsFilter.alphaQ.push_back(xmlToFloat(alphaQElement, "alphaQ"));
    paramsFilter.alphadQ.push_back(xmlToFloat(alphadQElement, "alphadQ"));
  }
}

inline ParamsF configureFilter(tinyxml2::XMLElement* filterConfig, unsigned int& robotLinks)
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

inline void configureAdaptiveControl(tinyxml2::XMLElement* controlConfig, ParamsC& paramsControl)
{
  tinyxml2::XMLElement* samplingRateElement = controlConfig->FirstChildElement("samplingRate");
  std::string deltString = samplingRateElement->FirstChildElement("delt")->GetText();
  paramsControl.delt = std::stof(deltString);

  tinyxml2::XMLElement* linearGainsElement = controlConfig->FirstChildElement("linearGains");
  tinyxml2::XMLElement* kElement = linearGainsElement->FirstChildElement("k");
  tinyxml2::XMLElement* lambdaElement = linearGainsElement->FirstChildElement("lambda");
  
  unsigned int n = 0;
  // Place the Linear Gains Inside of the Params //
  for (int i = 0; i < paramsControl.numberLinks; i++)
  {
    paramsControl.k.push_back(xmlToFloat(kElement, "k"));
    paramsControl.lambda.push_back(xmlToFloat(lambdaElement, "lambda"));
    n = n+i+1;
  }

  tinyxml2::XMLElement* rateOfAdaptivityElement = controlConfig->FirstChildElement("rateOfAdaptivity");
  tinyxml2::XMLElement* gammaElement = rateOfAdaptivityElement->FirstChildElement("gamma");
  // Place Rate of Adaptivity Inside the Params //
  for (int i = 0; i < n; i++)
  {
    paramsControl.gamma.push_back(xmlToFloat(gammaElement, "gamma"));
  }
}

inline void configureRobustControl(tinyxml2::XMLElement* controlConfig, ParamsC& paramsControl)
{
  tinyxml2::XMLElement* linearGainsElement = controlConfig->FirstChildElement("linearGains");
  tinyxml2::XMLElement* kElement = linearGainsElement->FirstChildElement("k");
  tinyxml2::XMLElement* lambdaElement = linearGainsElement->FirstChildElement("lambda");
  
  // Place the Linear Gains Inside of the Params //
  for (int i = 0; i < paramsControl.numberLinks; i++)
  {
    paramsControl.k.push_back(xmlToFloat(kElement, "k"));
    paramsControl.lambda.push_back(xmlToFloat(lambdaElement, "lambda"));
  }

  tinyxml2::XMLElement* parameterNoiseElement = controlConfig->FirstChildElement("parameterNoise");
  tinyxml2::XMLElement* rhoElement = parameterNoiseElement->FirstChildElement("rho");
  tinyxml2::XMLElement* epsilonElement = parameterNoiseElement->FirstChildElement("epsilon");
  
  // Grab the parameter noise terms //
  paramsControl.rho = xmlToFloat(rhoElement, "rho");
  paramsControl.epsilon = xmlToFloat(epsilonElement, "epsilon");
}

inline void configurePDControl(tinyxml2::XMLElement* controlConfig, ParamsC& paramsControl)
{
  tinyxml2::XMLElement* linearGainsElement = controlConfig->FirstChildElement("linearGains");
  tinyxml2::XMLElement* kpElement = linearGainsElement->FirstChildElement("kp");
  tinyxml2::XMLElement* kdElement = linearGainsElement->FirstChildElement("kd");
  
  // Place the Linear Gains Inside of the Params //
  for (int i = 0; i < paramsControl.numberLinks; i++)
  {
    paramsControl.kp.push_back(xmlToFloat(kpElement, "kp"));
    paramsControl.kd.push_back(xmlToFloat(kdElement, "kd"));
  }
}

inline ParamsC configureControl(tinyxml2::XMLElement* controlConfig, unsigned int& robotLinks)
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
    configureRobustControl(controlConfig, paramsControl);
  }
  else if (controlType.compare("pdControl") > 0)
  {
    configurePDControl(controlConfig, paramsControl);
  }

  return paramsControl;
}

inline std::tuple<ParamsR, ParamsF, ParamsC> initializeParams(const char* configFile)
{
  // Load and Parse the Config File //
  tinyxml2::XMLDocument config;
  tinyxml2::XMLError error = config.LoadFile(configFile);

  // Get the Root Element of the Config File //
  tinyxml2::XMLElement* controllerConfig = config.FirstChildElement("Controller");

  std::string robotLinksString = controllerConfig->FirstChildElement("numberLinks")->GetText();
  unsigned int robotLinks = std::stoi(robotLinksString);

  tinyxml2::XMLElement* robotConfig = controllerConfig->FirstChildElement("robot");
  tinyxml2::XMLElement* controlConfig = controllerConfig->FirstChildElement("control");
  tinyxml2::XMLElement* filterConfig = controllerConfig->FirstChildElement("filter");

  ParamsR paramsRobot = configureRobot(robotConfig, robotLinks);
  ParamsF paramsFilter = configureFilter(filterConfig, robotLinks);
  ParamsC paramsControl = configureControl(controlConfig, robotLinks);

  return std::make_tuple(paramsRobot, paramsFilter, paramsControl);
}

} // namespace configurator
} // namespace robot

#endif // CONFIGURATOR_HPP_