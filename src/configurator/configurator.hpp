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
  bool enable_adaptive = false;
  bool enable_robust = false;
  bool enable_pd = true;

  float delt = 0.0f;
  float rho = 0.0f;
  float epsilon = 0.0f;

  std::vector<float> K;
  std::vector<float> mLambda;
  std::vector<float> mGamma;
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
configureLowPassFilter(tinyxml2::XMLElement* filterConfig, ParamsF& paramsFilter);

ParamsC configureControl(tinyxml2::XMLElement* controlConfig, unsigned int& robotLinks);

} // namespace configurator
} // namespace robot

#endif // CONFIGURATOR_HPP_