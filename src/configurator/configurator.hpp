#ifndef CONFIGURATOR_HPP_
#define CONFIGURATOR_HPP_

#include <exception>
#include <iostream>
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
  float del = 0.0f;
  
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

inline void babyLogger(tinyxml2::XMLElement*& xmlElement, std::string string)
{
  if (xmlElement == nullptr)
  {
    std::cout << string << std::endl;
    throw std::exception();
  }
}

inline void babyLogger(tinyxml2::XMLError& xmlError, std::string string)
{
  if (xmlError != tinyxml2::XMLError::XML_SUCCESS)
  {
    std::cout << string << std::endl;
    throw std::exception();
  }
}

inline bool stringToBool(std::string text)
{
  bool result = false;
  if (text.compare("true") == 0 || text.compare("1") == 0)
  {
    result = true;
  }

  return result;
}

inline int xmlToInt(tinyxml2::XMLElement*& xmlElement, const char* elementChar)
{
  std::string elementString = elementChar;
  std::string stringWord = "configurator::xmlToInt - Not enough " + elementString + " elements!";
  babyLogger(xmlElement, stringWord);

  std::string elementText = xmlElement->GetText();
  int value = std::stoi(elementText);
  xmlElement = xmlElement->NextSiblingElement(elementChar);

  return value;
}

inline float xmlToFloat(tinyxml2::XMLElement*& xmlElement, const char* elementChar)
{
  std::string elementString = elementChar;
  std::string stringWord = "configurator::xmlToFloat - Not enough " + elementString + " elements!";
  babyLogger(xmlElement, stringWord);

  std::string elementText = xmlElement->GetText();
  float value = std::stof(elementText);
  xmlElement = xmlElement->NextSiblingElement(elementChar);

  return value;
}

inline ParamsR configureRobot(tinyxml2::XMLElement*& robotConfig, unsigned int& robotLinks)
{
  ParamsR paramsRobot;

  paramsRobot.numberLinks = robotLinks;
  
  tinyxml2::XMLElement* gravityElement = robotConfig->FirstChildElement("enableGravityTerms");
  babyLogger(gravityElement, "configurator::configureRobot - Failed to find gravity element!");

  std::string enableGravityTermsString = gravityElement->GetText();
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

inline void configureLowPassFilter(tinyxml2::XMLElement*& filterConfig, ParamsF& paramsFilter)
{
  tinyxml2::XMLElement* filterOrderElement = filterConfig->FirstChildElement("filterOrder");
  babyLogger(filterOrderElement, "configurator::configureLowPassFilter - Failed to find filter order element!");

  std::string filterOrderString = filterOrderElement->GetText();
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

inline ParamsF configureFilter(tinyxml2::XMLElement*& filterConfig, unsigned int& robotLinks)
{
  ParamsF paramsFilter;

  paramsFilter.numberLinks = robotLinks;

  tinyxml2::XMLElement* filterTypeElement = filterConfig->FirstChildElement("filterType");
  babyLogger(filterTypeElement, "configurator::configureFilter - Failed to find filter type element!");

  std::string filterType = filterTypeElement->GetText();
  paramsFilter.filterType = filterType;

  if (filterType.compare("lowPassFilter") == 0)
  {
    configureLowPassFilter(filterConfig, paramsFilter);
  }

  return paramsFilter;
}

inline void configureAdaptiveControl(tinyxml2::XMLElement*& controlConfig, ParamsC& paramsControl)
{
  tinyxml2::XMLElement* samplingRateElement = controlConfig->FirstChildElement("samplingRate");
  babyLogger(samplingRateElement, "configurator::configureAdaptiveControl - Failed to find sampling rate element!");
  
  tinyxml2::XMLElement* deltElement =samplingRateElement->FirstChildElement("delt");
  babyLogger(deltElement, "configurator::configureAdaptiveControl - Failed to find delt element!");

  std::string deltString = deltElement->GetText();
  paramsControl.delt = std::stof(deltString);

  tinyxml2::XMLElement* linearGainsElement = controlConfig->FirstChildElement("linearGains");
  babyLogger(linearGainsElement, "configurator::configureAdaptiveControl - Failed to find linear gains element!");

  tinyxml2::XMLElement* kElement = linearGainsElement->FirstChildElement("k");
  tinyxml2::XMLElement* lambdaElement = linearGainsElement->FirstChildElement("lambda");
  
  unsigned int n = 0;
  // Place the Linear Gains Inside of the Params //
  for (int i = 0; i < paramsControl.numberLinks; i++)
  {
    paramsControl.k.push_back(xmlToFloat(kElement, "k"));
    paramsControl.lambda.push_back(xmlToFloat(lambdaElement, "lambda"));
    n = n+i+2;
  }

  tinyxml2::XMLElement* rateOfAdaptivityElement = controlConfig->FirstChildElement("rateOfAdaptivity");
  babyLogger(rateOfAdaptivityElement, "configurator::configureAdaptiveControl - Failed to find rate of adaptivity element!");

  tinyxml2::XMLElement* gammaElement = rateOfAdaptivityElement->FirstChildElement("gamma");

  // Place Rate of Adaptivity Inside the Params //
  for (int i = 0; i < n; i++)
  {
    paramsControl.gamma.push_back(xmlToFloat(gammaElement, "gamma"));
  }
}

inline void configureRobustControl(tinyxml2::XMLElement*& controlConfig, ParamsC& paramsControl)
{
  tinyxml2::XMLElement* linearGainsElement = controlConfig->FirstChildElement("linearGains");
  babyLogger(linearGainsElement, "configurator::configureRobustControl - Failed to find linear gains element!");

  tinyxml2::XMLElement* kElement = linearGainsElement->FirstChildElement("k");
  tinyxml2::XMLElement* lambdaElement = linearGainsElement->FirstChildElement("lambda");
  
  // Place the Linear Gains Inside of the Params //
  for (int i = 0; i < paramsControl.numberLinks; i++)
  {
    paramsControl.k.push_back(xmlToFloat(kElement, "k"));
    paramsControl.lambda.push_back(xmlToFloat(lambdaElement, "lambda"));
  }

  tinyxml2::XMLElement* parameterNoiseElement = controlConfig->FirstChildElement("parameterNoise");
  babyLogger(parameterNoiseElement, "configurator::configureRobustControl - Failed to find parameter noise element!");

  tinyxml2::XMLElement* rhoElement = parameterNoiseElement->FirstChildElement("rho");
  tinyxml2::XMLElement* epsilonElement = parameterNoiseElement->FirstChildElement("epsilon");
  
  // Grab the parameter noise terms //
  paramsControl.rho = xmlToFloat(rhoElement, "rho");
  paramsControl.epsilon = xmlToFloat(epsilonElement, "epsilon");
}

inline void configureRobustAdaptiveControl(tinyxml2::XMLElement*& controlConfig, ParamsC& paramsControl)
{
  // Configure adaptive terms //
  tinyxml2::XMLElement* samplingRateElement = controlConfig->FirstChildElement("samplingRate");
  babyLogger(samplingRateElement, "configurator::configureRobustAdaptiveControl - Failed to find sampling rate element!");
  
  tinyxml2::XMLElement* deltElement = samplingRateElement->FirstChildElement("delt");
  babyLogger(deltElement, "configurator::configureRobustAdaptiveControl - Failed to find delt element!");

  std::string deltString = deltElement->GetText();
  paramsControl.delt = std::stof(deltString);

  tinyxml2::XMLElement* linearGainsElement = controlConfig->FirstChildElement("linearGains");
  babyLogger(linearGainsElement, "configurator::configureRobustAdaptiveControl - Failed to find linear gains element!");

  tinyxml2::XMLElement* kElement = linearGainsElement->FirstChildElement("k");
  tinyxml2::XMLElement* lambdaElement = linearGainsElement->FirstChildElement("lambda");
  
  unsigned int n = 0;
  // Place the Linear Gains Inside of the Params //
  for (int i = 0; i < paramsControl.numberLinks; i++)
  {
    paramsControl.k.push_back(xmlToFloat(kElement, "k"));
    paramsControl.lambda.push_back(xmlToFloat(lambdaElement, "lambda"));
    n = n+i+2;
  }

  tinyxml2::XMLElement* rateOfAdaptivityElement = controlConfig->FirstChildElement("rateOfAdaptivity");
  babyLogger(rateOfAdaptivityElement, "configurator::configureRobustAdaptiveControl - Failed to find rate of adaptivity element!");

  tinyxml2::XMLElement* gammaElement = rateOfAdaptivityElement->FirstChildElement("gamma");

  // Place Rate of Adaptivity Inside the Params //
  for (int i = 0; i < n; i++)
  {
    paramsControl.gamma.push_back(xmlToFloat(gammaElement, "gamma"));
  }
  
  // Configure Robust terms //
  tinyxml2::XMLElement* parameterNoiseElement = controlConfig->FirstChildElement("parameterNoise");
  babyLogger(parameterNoiseElement, "configurator::configureRobustAdaptiveControl - Failed to find parameter noise element!");

  tinyxml2::XMLElement* rhoElement = parameterNoiseElement->FirstChildElement("rho");
  tinyxml2::XMLElement* delElement = parameterNoiseElement->FirstChildElement("del");
  
  // Grab the parameter noise terms //
  paramsControl.rho = xmlToFloat(rhoElement, "rho");
  paramsControl.del = xmlToFloat(delElement, "del");
}

inline void configurePDControl(tinyxml2::XMLElement*& controlConfig, ParamsC& paramsControl)
{
  tinyxml2::XMLElement* linearGainsElement = controlConfig->FirstChildElement("linearGains");
  babyLogger(linearGainsElement, "configurator::configurePDControl - Failed to find linear gains element!");

  tinyxml2::XMLElement* kpElement = linearGainsElement->FirstChildElement("kp");
  tinyxml2::XMLElement* kdElement = linearGainsElement->FirstChildElement("kd");
  
  // Place the Linear Gains Inside of the Params //
  for (int i = 0; i < paramsControl.numberLinks; i++)
  {
    paramsControl.kp.push_back(xmlToFloat(kpElement, "kp"));
    paramsControl.kd.push_back(xmlToFloat(kdElement, "kd"));
  }
}

inline ParamsC configureControl(tinyxml2::XMLElement*& controlConfig, unsigned int& robotLinks)
{
  ParamsC paramsControl;

  paramsControl.numberLinks = robotLinks;
  
  tinyxml2::XMLElement* controlTypeElement = controlConfig->FirstChildElement("controlType");
  babyLogger(controlTypeElement, "configurator::configureControl - Failed to find control type element!");

  std::string controlType =controlTypeElement->GetText();
  paramsControl.controlType = controlType;

  if (controlType.compare("adaptiveControl") == 0)
  {
    configureAdaptiveControl(controlConfig, paramsControl);
  }
  else if (controlType.compare("robustControl") == 0)
  {
    configureRobustControl(controlConfig, paramsControl);
  }
  else if (controlType.compare("robustAdaptiveControl") == 0)
  {
    configureRobustAdaptiveControl(controlConfig, paramsControl);
  }
  else if (controlType.compare("pdControl") == 0)
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
  babyLogger(error, "configurator::initializeParams - XML failed to parse!");

  // Get the Root Element of the Config File //
  tinyxml2::XMLElement* controllerConfig = config.FirstChildElement("Controller");
  
  tinyxml2::XMLElement* robotLinksElement = controllerConfig->FirstChildElement("numberLinks");
  babyLogger(robotLinksElement, "configurator::initializeParams - Failed to find robot links element!");

  std::string robotLinksString = robotLinksElement->GetText();
  unsigned int robotLinks = std::stoi(robotLinksString);

  tinyxml2::XMLElement* robotConfig = controllerConfig->FirstChildElement("robot");
  babyLogger(robotConfig, "configurator::initializeParams - Failed to find robot element!");

  tinyxml2::XMLElement* controlConfig = controllerConfig->FirstChildElement("control");
  babyLogger(controlConfig, "configurator::initializeParams - Failed to find control element!");

  tinyxml2::XMLElement* filterConfig = controllerConfig->FirstChildElement("filter");
  babyLogger(filterConfig, "configurator::initializeParams - Failed to find filter element!");

  ParamsR paramsRobot = configureRobot(robotConfig, robotLinks);
  ParamsF paramsFilter = configureFilter(filterConfig, robotLinks);
  ParamsC paramsControl = configureControl(controlConfig, robotLinks);

  return std::make_tuple(paramsRobot, paramsFilter, paramsControl);
}

} // namespace configurator
} // namespace robot

#endif // CONFIGURATOR_HPP_