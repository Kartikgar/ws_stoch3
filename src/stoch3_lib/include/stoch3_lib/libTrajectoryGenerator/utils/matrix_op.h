/**
 * matrix_op.h
 *
 * Created : 3 May, 2021
 * Author  : Chandravaran Kunjeti, Aditya Shirwatkar
 */

#ifndef __MATRIX_OP_H__
#define __MATRIX_OP_H__

#include <Eigen/Dense>

#include "utils/vector_op.h"

namespace utils
{
  template<typename Type, int Rows, int Cols>
  using Matrix = Eigen::Matrix <Type, Rows, Cols>;

  using Matrix2d = Eigen::Matrix2d;
  using Matrix3d = Eigen::Matrix3d;

  template<typename Type, int Rows>
  using MatrixRX = Eigen::Matrix<Type, Rows, Eigen::Dynamic>;

  template<typename Type, int Cols>
  using MatrixXC = Eigen::Matrix<Type, Eigen::Dynamic, Cols>;

  template<typename Type>
  using MatrixXX = Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic>;

}
#endif // __MATRIX_OP_H__
