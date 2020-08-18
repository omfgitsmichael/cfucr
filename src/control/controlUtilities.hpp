#ifndef CONTROL_UTILITIES_HPP_
#define CONTROL_UTILITIES_HPP_

#include <cmath>

#include "mathUtilities/matrixUtilities.hpp"
#include "types/oneLinkRobot.hpp"

namespace robot
{
// Functions to calculate passivity terms //
template <typename Robot, typename Vector, typename Matrix>
Vector calculateV(Robot& robot, Matrix& lambda);
template <typename Robot, typename Vector, typename Matrix>
Vector calculateA(Robot& robot, Matrix& lambda);
template <typename Robot, typename Vector, typename Matrix>
Vector calculateR(Robot& robot, Matrix& lambda);

// One link regressor matrix //
Matrix1x2F oneLinkRegressor(sharedOneLinkRobot& robot, ScalarF& v, ScalarF& a);

} // namespace robot
#endif // CONTROL_UTILITIES_HPP_
