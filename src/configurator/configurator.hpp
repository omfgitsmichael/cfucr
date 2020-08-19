#ifndef CONFIGURATOR_HPP_
#define CONFIGURATOR_HPP_

#include <string>

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
  float delt;

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

std::tuple<ParamsR, ParamsF, ParamsC> initializeParams(const std::string configFile);

ParamsR configureRobot(tinyxml2::XMLElement* robotConfig, unsigned int& robotLinks);

ParamsF configureFilter(tinyxml2::XMLElement* filterConfig, unsigned int& robotLinks);
void configureLowPassFilter(tinyxml2::XMLElement* filterConfig, ParamsF& paramsFilter);

ParamsC configureControl(tinyxml2::XMLElement* controlConfig, unsigned int& robotLinks);
void configureAdaptiveControl(tinyxml2::XMLElement* controlConfig, ParamsC& paramsControl);
void configureRobustControl(tinyxml2::XMLElement* controlConfig, ParamsC& paramsControl);
void configurePDControl(tinyxml2::XMLElement* controlConfig, ParamsC& paramsControl);

// Helper Functions //
bool stringToBool(std::string text);
int xmlToInt(tinyxml2::XMLElement* xmlElement, std::string elementString);
float xmlToFloat(tinyxml2::XMLElement* xmlElement, std::string elementString);

} // namespace configurator
} // namespace robot

#endif // CONFIGURATOR_HPP_