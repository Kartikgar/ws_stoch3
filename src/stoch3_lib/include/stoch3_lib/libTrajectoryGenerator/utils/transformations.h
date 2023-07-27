/**
 * transformation.h
 *
 * Created : 3 May, 2021
 * Author  : Chandravaran Kunjeti, Aditya Shirwatkar
 */

#ifndef __TRANSFORMATIONS_H__
#define __TRANSFORMATIONS_H__

#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "utils/matrix_op.h"

#define FL 0
#define FR 1
#define BL 2
#define BR 3

#define ABD 0
#define HIP 1
#define KNE 2

namespace utils
{
  // For printing with full precision
  // Eigen::IOFormat CommaIntFmt(Eigen::FullPrecision, Eigen::DontAlignCols, ", ");

  // TODO:: Note: Commenting the above function to prevent errors (of multiple definitions) when 
  // this file is included from multiple cpp files. This might break some dependencies. 
  // If so, then an alternate method to set the format should be figured out.

  template<typename Type>
  using Rot2D = Eigen::Rotation2D<Type>;

  template<typename Type>
  using AngleAxis = Eigen::AngleAxis<Type>; // vector must be normalized

  template<typename Type>
  using Quat = Eigen::Quaternion<Type>;

  /*
    For scaling, directly use Eigen::Scaling() function in N Dimensions or .scale() method of transform
  */

  /**
    @brief Get Rotation Matrix from Euler angles

    @param[in] roll: roll angle in radians
    @param[in] pitch: pitch angle in radians
    @param[in] yaw: yaw angle in radians
    @return A rotation matrix of order RPY
  */
  template<typename Type>
  Matrix<Type, 3, 3> eulerToRotation(Type roll, Type pitch, Type yaw)
  {
    Quat<Type> q(0, 0, 0, 0);

    q = AngleAxis<Type>(roll, Vector<Type, 3>::UnitX())
      * AngleAxis<Type>(pitch, Vector<Type, 3>::UnitY())
      * AngleAxis<Type>(yaw, Vector<Type, 3>::UnitZ());

    Matrix<Type, 3, 3> R = q.toRotationMatrix();

    assert(R.hasNaN() != true && "Invalid! eulerToRotation output Matrix has NaN element");

    return R;
  }

  /**
    @brief Get Euler angles (RPY) from Rotation Matrix

    @param[in] R: Rotation matrix (Rows = 3, Cols = 3)
    @return A vector of RPY values
  */
  template<typename Type>
  Vector<Type, 3> rotationToEuler(Matrix<Type, 3, 3> R)
  {
    assert(R.hasNaN() != true && "Invalid! rotationToEuler input Matrix has NaN element");

    // Returns angles in range [-pi, pi]
    // Vector<Type, 3> v = R.eulerAngles(0,1,2);

    double sy = ((R.col(0)).head(2)).norm();
    Vector<Type, 3> v;
    int singular = sy < 1e-6 ? 1 : 0;

    if (!singular)
    {
      v(0) = atan2(R(2, 1), R(2, 2));
      v(1) = atan2(-R(2, 0), sy);
      v(2) = atan2(R(1, 0), R(0, 0));
    }
    else
    {
      v(0) = atan2(R(0, 1), R(1, 1));
      v(1) = M_PI / 2.0;
      v(2) = 0.0;
    }

    return v;
  }

  /**
    @brief Get Euler angles (RPY) from Quaternion Vector

    @param[in] q: Quaternion
    @return A vector of RPY values
  */
  template<typename Type>
  Vector<Type, 3> quaternionToEuler(Quat<Type> q)
  {
    return rotationToEuler(q.toRotationMatrix());
  }

  /**
    @brief Get Euler angles (RPY) from Quaternion Vector

    @param[in] q: Quaternion
    @return A vector of RPY values
  */
  template<typename Type>
  Quat<Type> eulerToQuaternion(Type roll, Type pitch, Type yaw)
  {
    Matrix<Type, 3, 3> rotation_matrix = eulerToRotation(roll, pitch, yaw);
    Quat<Type> q(rotation_matrix);
    return q;
  }

 /**
    @brief Apply a transformation -> T(x) = R*x + t, to an input vector x

    @param roll: roll angle in radians
    @param pitch: pitch angle in radians
    @param yaw: yaw angle in radians
    @param trans: translation vector
    @param x: input vector to transform
    @return A vector after the transformation is applied
  */
  template<typename Type>
  Vector<Type, 3> rotAndTrans(Type roll, Type pitch, Type yaw, Vector<Type, 3> trans, Vector<Type, 3> x)
  {
    assert(trans.hasNaN() != true && "Invalid! rotAndTrans input translational vector has NaN element");
    assert(x.hasNaN() != true && "Invalid! rotAndTrans input x vector has NaN element");

    Vector<Type, 3> x_out(0, 0, 0);

    Quat<Type> q(0, 0, 0, 0);

    q = AngleAxis<Type>(roll, Vector<Type, 3>::UnitX())
      * AngleAxis<Type>(pitch, Vector<Type, 3>::UnitY())
      * AngleAxis<Type>(yaw, Vector<Type, 3>::UnitZ());

    x_out = q * x + trans;

    assert(x_out.hasNaN() != true && "Invalid! rotAndTrans output x_out vector has NaN element");
    return x_out;
  }

  /**
    @brief Get RPY values wrt to world frame of a surface,
    from a normal vector of a surface and RPY in body frame

    @param roll: roll angle in radians of body frame
    @param pitch: pitch angle in radians of body frame
    @param yaw: yaw angle in radians of body frame
    @param surface_normal: a surface normal vector, need not be a unit vector
    @return A vector or RPY values wrt world frame
  */
  template<typename Type>
  Vector<Type, 3> surfNormVecToEuler(Type roll, Type pitch, Type yaw, Vector<Type, 3> surface_normal)
  {
    assert(surface_normal.hasNaN() != true && "Invalid! surfNormVecToEuler input surface normal vector has NaN element");

    Vector<Type, 3> rpy_angles;

    if (surface_normal.norm() < 1e-6)
    {
      rpy_angles.setZero(3, 1);
      return rpy_angles;
    }

    surface_normal.normalize();

    Vector<Type, 3> vx(1, 0, 0);
    Vector<Type, 3> vx_world(0, 0, 0);
    Vector<Type, 3> surface_normal_world(0, 0, 0);

    Quat<Type> q(0, 0, 0, 0);

    q = AngleAxis<Type>(roll, Vector<Type, 3>::UnitX())
      * AngleAxis<Type>(pitch, Vector<Type, 3>::UnitY())
      * AngleAxis<Type>(yaw, Vector<Type, 3>::UnitZ());

    vx_world = q * vx;
    surface_normal_world = q * surface_normal;

    Vector<Type, 3> y_est; y_est = surface_normal_world.cross(vx_world);
    Vector<Type, 3> x_est; x_est = y_est.cross(surface_normal_world);

    Matrix<Type, 3, 3> rot_mat_support_plane;
    rot_mat_support_plane << x_est, y_est, surface_normal_world; // Horizontal concatenation
    rpy_angles = rotationToEuler(rot_mat_support_plane);

    assert(rpy_angles.hasNaN() != true && "Invalid! surfNormVecToEuler output rpy_angles vector has NaN element");

    return rpy_angles;
  }

  /** @brief Get normal to the plane of 3 points
  *
  * \param[in] a: Vector A
  * \param[in] b: Vector B
  * \param[in] c: Vector C
  */
  template<typename Type>
  utils::Vector<Type, 3> planeNormalThreePoint(utils::Vector<Type, 3> a, utils::Vector<Type, 3> b, utils::Vector<Type, 3> c)
  {
    assert(a.hasNaN() != true && "Invalid! planeNormalThreePoint input a vector has NaN element");
    assert(b.hasNaN() != true && "Invalid! planeNormalThreePoint input b vector has NaN element");
    assert(c.hasNaN() != true && "Invalid! planeNormalThreePoint input c vector has NaN element");

    utils::Vector<Type, 3> cross_p;
    cross_p = (b - a).cross(c - a);
    cross_p.normalize();

    assert(cross_p.hasNaN() != true && "Invalid! planeNormalThreePoint output vector has NaN element");

    return cross_p;
  }

  /** @brief Shifts a matrix/vector row-wise.
  *
  * \param[in] in: Matrix/vector who's type can be either a fixed or dynamically-sized matrix.
  * \param[in] down: A negative value is taken to mean shifting up.
  *                 When passed zero, the input matrix is returned unchanged.
  */
  template <typename M> M shiftByRows(const M& in, int down)
  {
    assert(in.size() != 0 && "Invalid! Matrix/Vector size is zero");
    assert(down != 0 && "Invalid! shiftByRows is useless for shift value of 0");

    if (!down) return in;
    M out(in.rows(), in.cols());
    if (down > 0) down = down % in.rows();
    else down = in.rows() - (-down % in.rows());
    // We avoid the implementation-defined sign of modulus with negative arg. 
    int rest = in.rows() - down;
    out.topRows(rest) = in.bottomRows(rest);
    out.bottomRows(down) = in.topRows(down);

    assert(out.hasNaN() != true && "Invalid! shiftByRows output Matrix/Vector has NaN element");

    return out;
  }

}

#endif // __TRANSFORMATIONS_H__
