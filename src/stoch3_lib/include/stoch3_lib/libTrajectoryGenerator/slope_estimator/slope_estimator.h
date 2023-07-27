/**
 * slope_estimator.h
 *
 * Created : 3 May, 2021
 * Author  : Chandravaran Kunjeti, Sayak Nandi, Aditya Shirwatkar
 */
#ifndef __SLOPE_ESTIMATOR_H__
#define __SLOPE_ESTIMATOR_H__

#include <iostream>
#include <math.h>
#include <string.h>

#include "utils/transformations.h"
#include "utils/filters.h"
#include "slope_estimator/tof_coordinates.h"

 // X-axis pointing forward
 // Y-axis pointing to the left
 // Z-axis pointing upward

namespace slope_esti
{
  /**
   * @brief This function gives a slope estimate of a planar surface using IMU and ToF sensors
   *
   * @param[in] tof_z: A Matrix of 6 ToF readings. The rows represent each ToF sensor,
   *               with col-0 being id of ToF sensor and col-1 being the reading
   * @param[in] imu_rpy: A Vector of 1 IMU reading (RPY).
   */
  template<typename Type>
  utils::Vector3d slopeEstimator(utils::Matrix<Type, 6, 2> tof_z, utils::Vector<Type, 3> imu_rpy)
  {
    assert(tof_z.hasNaN() != true && "Invalid! slopeEstimator input tof_z Matrix has NaN element");
    assert(imu_rpy.hasNaN() != true && "Invalid! slopeEstimator input imu_rpy Matrix has NaN element");

    utils::Vector3d plane_normal1, plane_normal2, plane_normal3, plane_normal4, plane_normal5, plane_normal6;
    utils::Vector3d diag52, diag62, diag42, diag53, diag51, diag31;
    utils::Vector3d rpy1, rpy2, rpy3, rpy4, rpy5, rpy6, rpy_angles;

    auto tof_xy = getTofCoordinates();
    utils::Vector<Type, 3> tof1(tof_xy(0, 0), tof_xy(0, 1), tof_z(0, 1));
    utils::Vector<Type, 3> tof2(tof_xy(1, 0), tof_xy(1, 1), tof_z(1, 1));
    utils::Vector<Type, 3> tof3(tof_xy(2, 0), tof_xy(2, 1), tof_z(2, 1));
    utils::Vector<Type, 3> tof4(tof_xy(3, 0), tof_xy(3, 1), tof_z(3, 1));
    utils::Vector<Type, 3> tof5(tof_xy(4, 0), tof_xy(4, 1), tof_z(4, 1));
    utils::Vector<Type, 3> tof6(tof_xy(5, 0), tof_xy(5, 1), tof_z(5, 1));

    // Vectors along the diagonals
    diag52 = tof5 - tof2;
    diag62 = tof6 - tof2;
    diag42 = tof4 - tof2;
    diag53 = tof5 - tof3;
    diag51 = tof5 - tof1;
    diag31 = tof3 - tof1;

    // Normal to the plane
    plane_normal1 = diag52.cross(diag62);
    plane_normal2 = diag42.cross(diag62);
    plane_normal3 = diag52.cross(diag53);
    plane_normal4 = diag51.cross(diag52);
    plane_normal5 = diag31.cross(diag42);
    plane_normal6 = diag42.cross(diag62);

    rpy1 = utils::surfNormVecToEuler(imu_rpy.x(), imu_rpy.y(), 0.0, plane_normal1);
    rpy2 = utils::surfNormVecToEuler(imu_rpy.x(), imu_rpy.y(), 0.0, plane_normal2);
    rpy3 = utils::surfNormVecToEuler(imu_rpy.x(), imu_rpy.y(), 0.0, plane_normal3);
    rpy4 = utils::surfNormVecToEuler(imu_rpy.x(), imu_rpy.y(), 0.0, plane_normal4);
    rpy5 = utils::surfNormVecToEuler(imu_rpy.x(), imu_rpy.y(), 0.0, plane_normal5);
    rpy6 = utils::surfNormVecToEuler(imu_rpy.x(), imu_rpy.y(), 0.0, plane_normal6);

    utils::Matrix<Type, 3, 6> Mat_rpy; Mat_rpy << rpy1, rpy2, rpy3, rpy4, rpy5, rpy6;
    utils::VectorX<Type> rolls;  rolls = Mat_rpy.row(0);
    utils::VectorX<Type> pitchs; pitchs = Mat_rpy.row(1);

    rpy_angles = { filters::giveMedian(rolls), filters::giveMedian(pitchs), 0 };

    assert(rpy_angles.hasNaN() != true && "Invalid! slopeEstimator output rpy_angles Vector has NaN element");

    return rpy_angles;
  }


  ///////////////////////////////// Experimental functions ///////////////////////////////////////////////

  // /** 
  //  * @brief This function gives a normal vector to plane using foot contact points. 
  //  * 
  //  * Note: This function adds additional 7-8 secs of compile time 
  //  * 
  //  * @param[in] foot_contact_points: A Matrix of foot position vectors which are in contact.
  //  *                                 Each col represents a foot position vector 3x1
  //  */
  // template<typename Type>
  // utils::Vector<Type, 3> normalFromFootContacts(utils::MatrixRX<Type, 3> foot_contact_points)
  // {
  //   assert(foot_contact_points.cols() != 0 && "Invalid! normalFromFootContacts input foot_contact_points Vector has cols = 0");
  //   assert(foot_contact_points.cols() <= 4 && "Invalid! normalFromFootContacts input foot_contact_points Vector has cols > 4");
  //   assert(foot_contact_points.hasNaN() != true && "Invalid! normalFromFootContacts input foot_contact_points Vector has NaN element");

  //   // calculate centroid
  //   utils::Vector<Type, 3> centroid(foot_contact_points.row(0).mean(), 
  //                                   foot_contact_points.row(1).mean(), 
  //                                   foot_contact_points.row(2).mean());

  //   // subtract centroid
  //   foot_contact_points -= centroid;

  //   // we only need the left-singular matrix here
  //   //  http://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
  //   auto svd = foot_contact_points.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
  //   utils::Vector<Type, 3> plane_normal = (svd.matrixU()).rightCols(1);

  //   assert(plane_normal.hasNaN() != true && "Invalid! normalFromFootContacts output plane_normal Vector has NaN element");

  //   return plane_normal;
  // }

}
#endif // __SLOPE_ESTIMATOR_H__
