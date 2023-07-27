/**
 * vector_op.h
 *
 * Created : 3 May, 2021
 * Author  : Chandravaran Kunjeti, Aditya Shirwatkar
 */

#ifndef __VECTOR_OP_H__
#define __VECTOR_OP_H__

#include <iostream>
#include <math.h>
#include <Eigen/Dense>

namespace utils
{
  template<typename Type, int Rows>
  using Vector = Eigen::Matrix <Type, Rows, 1>; // A column vector, currently does not support user derived data types

  template<typename Type>
  using VectorX = Eigen::Matrix<Type, Eigen::Dynamic, 1>; // A Dynamic column vector, currently does not support user derived data types

  using Vector2d = Eigen::Vector2d;
  using Vector3d = Eigen::Vector3d;
  using Vector4d = Eigen::Vector4d;

  template<typename T, std::size_t S>
  std::ostream& operator<<(std::ostream& os, const std::array<T, S>& vec)
  {
    for (int i = 0;i < S;i++)
      os << vec[i] << " ";
    os << std::endl;
    return os;
  }

}
#endif // __VECTOR_OP_H__
