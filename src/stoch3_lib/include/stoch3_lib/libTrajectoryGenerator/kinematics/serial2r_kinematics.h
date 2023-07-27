/*
 * Copyright 2021, StochLab
 *
 * Author: Aditya Sagi, Aditya Shirwatkar
 * Date:   Feb, 2021
 */

#ifndef __SERIAL2R_KINEMATICS_H__
#define __SERIAL2R_KINEMATICS_H__

#include <iostream>
#include <assert.h>
#include <math.h>

#include "utils/transformations.h"

namespace kine
{

  /**
   * @brief Class for the kinematics of a serial-2R chain.
   *
   * This class implements the kinematics (forward and inverse) of a planar serial-2R chain.
   * It assumes that the chain lies in the x-z plane with x pointing forward and z pointing
   * upward. The joints rotate about the y-axis.
   */
  class Serial2RKinematics
  {

  private:
    utils::Vector2d link_lengths_;
    bool  safety_; // safety check turned on

    /**
     * @brief Give the three sides of a triangle, find the angle opposite to side A
     * i.e., angle between links B and C
     *
     */
    template<typename Type>
    double cosineRule(Type a, Type b, Type c)
    {
      assert(a >= 0 && "Invalid! For cosineRule length a of the triangle must be >= 0");
      assert(b >= 0 && "Invalid! For cosineRule length b of the triangle must be >= 0");
      assert(c >= 0 && "Invalid! For cosineRule length c of the triangle must be >= 0");

      assert(a + b >= c && "Invalid! In cosineRule triangle inequality, a + b >= c not satisfied");
      assert(c + a >= b && "Invalid! In cosineRule triangle inequality, c + a >= b not satisfied");
      assert(b + c >= a && "Invalid! In cosineRule triangle inequality, b + c >= a not satisfied");

      return acosf((c * c + b * b - a * a) / (2 * b * c));
    }


  public:
    Serial2RKinematics(utils::Vector2d link_lengths) :
      link_lengths_(link_lengths), safety_(true)
    {}

    Serial2RKinematics() : link_lengths_({ 1.0, 1.0 }), safety_(true) {}

    void turnOnSafety()
    {
      safety_ = true;
      return;
    }

    void turnOffSafety()
    {
      safety_ = false;
      return;
    }

    bool getSafety()
    {
      return safety_;
    }


    /**
     * @brief Invserse kinematics for a serial-2R chain.
     *
     * This function implements the inverse kinematics for a planar serial-2R chain.
     * It is assumed that the chain lies in the x-z plane with the first joint angle
     * measured w.r.t. the negative z axis (vertical axis) and the second joint
     * angle measured w.r.t the first link. This function can provide one of two
     * solutions to the inverse kinematics problem based on the branch that is chosen.
     *
     * The inverse kinematics is solved in two stages. First, the angle made by the radial
     * line (line joining origin to the end-effector) w.r.t the reference is determined. Second,
     * the angle made by the link l1 w.r.t. the radial line is determined. The hip angle then
     * becomes the sum of the two angles.
     *
     * \param[in] ee_pos : End-effector position in the Cartesian space (x, z).
     *
     * \param[in] branch : Specifies branch of the inverse kinematics solution.
     *                     It can take values '<' or '>'.
     *
     * \param[out] joint_state : The joint angle obtained as a solution are provided
     *                           in this vector.
     *
     *
     * \return int : Specifies if the solution exists for the given input.
     *                Returns 0 if solution exists for given input.
     *                Returns 1 if a new point is found within the workspace and the solution exists for it.
     *                Returns -1 if no solution exists.
     */
    int inverseKinematics(utils::Vector2d ee_pos, char branch, utils::Vector2d& joint_angles)
    {
      assert(ee_pos.size() == 2 && "Invalid! In serial 2r Inverse Kinematics, input ee_pos size != 2");
      assert(ee_pos.hasNaN() != true && "Invalid! In serial 2r Inverse Kinematics, input ee_pos has NaN element");

      double  l1, l2;
      double  theta1, theta2;
      double  t1, t2;
      double  r_theta;
      bool    valid_pt = false;
      int     valid_ik = 0;
      utils::Vector2d pos(0, 0);

      // Zero position is invalid
      if (ee_pos.norm() < 0.0001)
        return -1;

      // If not in workspace, then find the point in workspace
      // closest to the desired end-effector position. Return
      // false if such a point is not found.
      if (!inWorkspace(ee_pos))
      {
        if (safety_)
        {
          pos = searchSafePosition(ee_pos);
          valid_ik = 1;
        }
        else
        {
          return -1;
        }
      }
      else
      {
        pos = ee_pos;
      }

      l1 = link_lengths_[0];
      l2 = link_lengths_[1];

      r_theta = atan2(-pos[0], -pos[1]); // angle made by radial line w.r.t the reference

      // Ensure the output lies in the range [-PI , PI]
      if (r_theta > M_PI)
        r_theta = r_theta - 2 * M_PI;
      else if (r_theta < -M_PI)
        r_theta = r_theta + 2 * M_PI;

      t1 = cosineRule(l2, pos.norm(), l1);  // internal angle opposite to l2
      t2 = cosineRule(pos.norm(), l1, l2);  // internal angle opposite to radial line

      theta2 = -(M_PI - t2);

      if (branch == '<')
      {
        t1 = -t1;
        theta2 = -theta2;
      }

      theta1 = r_theta + t1;

      joint_angles.resize(2);
      joint_angles[0] = theta1;
      joint_angles[1] = theta2;

      assert(joint_angles.hasNaN() != true && "Invalid! In serial 2r Inverse Kinematics, output joint_angles has NaN element");

      return valid_ik;
    }

    /**
     * @brief Forward kinematics of a serial-2R chain.
     *
     * This function provides the forward kinematics soultion for a planar serial-2R chain.
     * It is assumed that the chain lies in the x-z plane. The first joint angle is measured
     * w.r.t the negative z-axis and the second joint angle is measured w.r.t the first link.
     * The axis of rotation is the positive y-axis and the direction is determined using
     * the right-hand rule with the thumb pointing in the positive y direction.
     *
     * \param[in] joint_angles : A two element vector of the joint angles.
     *
     * \param[out] ee_pos : End-effector position (x, z).
     */
    void forwardKinematics(utils::Vector2d joint_angles, utils::Vector2d& ee_pos)
    {
      assert(joint_angles.size() == 2 && "Invalid! In serial 2r forward kinematics, input joint_angles size != 2");
      assert(joint_angles.hasNaN() != true && "Invalid! In serial 2r Inverse Kinematics, input joint_angles has NaN element");

      double theta1, theta2;
      double l1, l2;

      l1 = link_lengths_[0];
      l2 = link_lengths_[1];

      theta1 = joint_angles[0];
      theta2 = joint_angles[1];

      ee_pos[0] = -(l1 * sin(theta1) + l2 * sin(theta1 + theta2));
      ee_pos[1] = -(l1 * cos(theta1) + l2 * cos(theta1 + theta2));

      assert(ee_pos.hasNaN() != true && "Invalid! In serial 2r Inverse Kinematics, output ee_pos has NaN element");
      return;
    }


    /*
     * @brief Funtion to provide the Jacobian Matrix.
     *
     * The forward Jacobian matrix is calculated for the foot position w.r.t the hip in the
     * frame of reference located at the hip.
     *
     * \param[in] joint_angles: Joint angles
     *
     * \reutrn 2x2 Jacobian matirix
     */
    utils::Matrix2d jacobianMatrix(utils::Vector2d joint_angles)
    {
       assert(joint_angles.hasNaN() != true && "Invalid! In serial 2r Inverse Kinematics, input joint_angles has NaN element");
    
      double theta1, theta2;
      double l1, l2;
      utils::Matrix2d jacobian_matrix;

      l1 = link_lengths_[0];
      l2 = link_lengths_[1];

      theta1 = joint_angles[0];
      theta2 = joint_angles[1];

      jacobian_matrix(0,0) = -l1*cos(theta1) - l2*cos(theta1 + theta2);
      jacobian_matrix(0,1) = -l2*cos(theta1 + theta2);
      jacobian_matrix(1,0) = l1*sin(theta1) + l2*sin(theta1 + theta2);
      jacobian_matrix(1,1) = l2*sin(theta1 + theta2);;
          
      return jacobian_matrix;
    }


    /**
     * @brief Function to check if the provided point is within the workspace of the serial-2R.
     *
     * \param[in] ee_pos: end-effector position
     *
     * \return bool: true if the point lies in the workspace, false otherwise
     */
    bool inWorkspace(utils::Vector2d ee_pos)
    {
      if (ee_pos.squaredNorm() > pow(link_lengths_.sum(), 2))
        return false;

      if (ee_pos.squaredNorm() < pow(link_lengths_[0] - link_lengths_[1], 2))
        return false;

      return true;
    }

    /**
     * @brief Function to search for the closest end-effector point within the workspace.
     *
     * This uses the bisection method to search for a feasible point on the boundary
     * of the workspace.
     *
     * \param[in] desired_pos: desired position of the end-effector
     *
     * \return array: Valid position inside the workspace
     */
    utils::Vector2d searchSafePosition(utils::Vector2d desired_pos)
    {
      const double  RADIAL_DISTANCE = 0.2; // radial distance for a point in workspace
      const double  DELTA_MAX = 0.001;
      const int     MAX_ITERATIONS = 20;

      utils::Vector2d p_in(0, 0), p_out(0, 0), p(0, 0);
      utils::Vector2d unit_vector(0, 0);
      int n = 0;

      // If the input is valid, then there is no need to search
      if (inWorkspace(desired_pos))
      {
        return desired_pos;
      }

      // p_out is always an invalid point (lies outside the workspace)
      p_out = desired_pos;

      unit_vector = p_out.normalized();

      // p_in is always a valid point (lies inside the workspace)
      p_in = RADIAL_DISTANCE * unit_vector;

      while (((p_in - p_out).norm() > DELTA_MAX) && (n < MAX_ITERATIONS))
      {
        p = (p_in + p_out) / 2;
        if (inWorkspace(p))
          p_in = p;
        else
          p_out = p;
        n++;
      }

      std::cout << "WARNING: Serial 2r Inverse Kinematics safety is being applied" << std::endl;

      return p_in;
    }

  };

} // end namespace

#endif //__SERIAL2R_KINEMATICS_H__
