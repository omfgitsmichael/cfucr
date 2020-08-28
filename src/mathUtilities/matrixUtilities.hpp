#ifndef MATRIX_UTILITIES_HPP_
#define MATRIX_UTILITIES_HPP_

#include "thirdParty/eigen/Eigen/Dense"
namespace robot
{
// Matrices and Vectors Used by the Robotic Manipulator //
using ScalarF = Eigen::Matrix<float,1,1>;
using Matrix2x2F = Eigen::Matrix<float,2,2>;
using Vector2x1F = Eigen::Matrix<float,2,1>;
using Matrix1x2F = Eigen::Matrix<float,1,2>;
using Matrix3x3F = Eigen::Matrix<float,3,3>;
using Matrix5x5F = Eigen::Matrix<float,5,5>;
using Vector3x1F = Eigen::Matrix<float,3,1>;
using Vector4x1F = Eigen::Matrix<float,4,1>;
using Vector5x1F = Eigen::Matrix<float,5,1>;
using Matrix2x5F = Eigen::Matrix<float,2,5>;
using Matrix9x9F = Eigen::Matrix<float,9,9>;
using Vector6x1F = Eigen::Matrix<float,6,1>;
using Vector9x1F = Eigen::Matrix<float,9,1>;
using Matrix3x9F = Eigen::Matrix<float,3,9>;
}

#endif // MATRIX_UTILITIES_HPP_
