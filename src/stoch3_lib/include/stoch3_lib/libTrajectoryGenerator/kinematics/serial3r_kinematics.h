/*
 * Copyright 2021, StochLab
 *
 * Author: Aditya Sagi, Aditya Shirwatkar
 * Date:   Feb, 2021
 */

#ifndef __SERIAL3R_KINEMATICS_H__
#define __SERIAL3R_KINEMATICS_H__

#include <iostream>
#include <assert.h>
#include <math.h>

#include "utils/transformations.h"
#include "kinematics/serial2r_kinematics.h"

namespace kine
{

  /**
   * @brief Class for the kinematics of a serial-3R chain.
   *
   * This class implements the kinematics (forward and inverse) of a spatial serial-3R chain.
   */
  class Serial3RKinematics
  {

  private:
    utils::Vector3d link_lengths_;
    bool safety_; // safety check turned on

    Serial2RKinematics* serial_2r_;

  public:
    Serial3RKinematics(utils::Vector3d link_lengths) :
      link_lengths_(link_lengths), safety_(true)
    {
      serial_2r_ = new Serial2RKinematics({ link_lengths[1], link_lengths[2] });
    }

    Serial3RKinematics() : link_lengths_({ 1.0, 1.0, 1.0 }), safety_(true)
    {
      serial_2r_ = new Serial2RKinematics({ link_lengths_[1], link_lengths_[2] });
    }

    ~Serial3RKinematics()
    {
      if (serial_2r_)
        delete serial_2r_;
    }

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
     * @brief Inverse kinematics for a serial-3R chain.
     *
     * This function implements the inverse kinematics for a planar serial-3R chain.
     * It is assumed that the chain lies in the x-z plane with the first joint angle
     * measured w.r.t. the negative z axis (vertical axis) and the second joint
     * angle measured w.r.t the first link. This function can provide one of two
     * solutions to the inverse kinematics problem based on the branch that is chosen.
     *
     * \param[in] leg_name: Name of the leg, "fl", "fr", "bl", "br"
     *
     * \param[in] ee_pos : End-effector position in the Cartesian space (x, y, z).
     *
     * \param[in] branch : Specifies branch of the inverse kinematics solution.
     *                     It can take values '<' or '>' for the branch of the
     *                     serial-2R chain formed by the last two links.
     *
     * \param[out] joint_state : The joint angle obtained as a solution are provided
     *                           in this vector.
     *
     *
     * \return int : Specifies if the solution exists for the given input.
     *                Returns 0 if solution exists.
     *                Returns 1 if the input is modified for safety (and solution exists)
     *                Returns -1 if solution does not exis.
     */
    int inverseKinematics(std::string leg_name, utils::Vector3d ee_pos, char branch, utils::Vector3d& joint_angles)
    {
      assert(ee_pos.size() == 3 && "Invalid! In serial 3r Inverse Kinematics, input ee_pos size != 3");
      assert(ee_pos.hasNaN() != true && "Invalid! In serial 3r Inverse Kinematics, input ee_pos has NaN element");

      bool right_leg;

      double  t1, t2;
      double  abd_angle, hip_angle, knee_angle;
      double  h, r;
      double  x, y, z;
      int     valid_ik = 0;
      double  l1;

      utils::Vector2d foot_pos_2r(0, 0);
      utils::Vector2d joint_angles_2r(0, 0);
      utils::Vector3d pos(0, 0, 0);

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

      if ((leg_name == "fr") || (leg_name == "br"))
        right_leg = true;
      else
        right_leg = false;

      x = pos.x();
      y = pos.y();
      z = pos.z();

      l1 = link_lengths_[0];

      r = pos.tail<2>().norm();

      h = sqrt(r * r - l1 * l1);

      t1 = atan2(h, l1);
      t2 = atan2(y, -z);

      if (right_leg)
        abd_angle = M_PI / 2 - t1 + t2;
      else
        abd_angle = t1 + t2 - M_PI / 2;

      foot_pos_2r[0] = x;
      foot_pos_2r[1] = -h;

      serial_2r_->inverseKinematics(foot_pos_2r, branch, joint_angles_2r);

      hip_angle = joint_angles_2r[0];
      knee_angle = joint_angles_2r[1];

      joint_angles[0] = abd_angle;
      joint_angles[1] = hip_angle;
      joint_angles[2] = knee_angle;

      assert(joint_angles.hasNaN() != true && "Invalid! In serial 3r Inverse Kinematics, output joint_angles has NaN element");

      return valid_ik;
    }

    /**
     * @brief Forward kinematics of a serial-3R chain.
     *
     * \param[in] leg_name: Name of the leg, "fl", "fr", "bl", "br".
     *
     * \param[in] joint_angles : A three element vector of the joint angles.
     *
     * \param[out] ee_pos : End-effector position (x, y, z).
     *
     */
    void forwardKinematics(std::string leg_name, utils::Vector3d joint_angles, utils::Vector3d& ee_pos)
    {
      assert(joint_angles.size() == 3 && "Invalid! In serial 3r forward kinematics, input joint_angles size != 3");
      assert(joint_angles.hasNaN() != true && "Invalid! In serial 3r forward kinematics, input joint_angles has NaN element");

      bool right_leg;

      double abd_angle, hip_angle, knee_angle;
      double l1, l2, l3;
      utils::Vector2d foot_pos_2r(0, 0), joint_angles_2r(0, 0);
      utils::Vector3d foot_pos_3r(0, 0, 0);


      if ((leg_name == "fr") || (leg_name == "br"))
        right_leg = true;
      else
        right_leg = false;

      l1 = link_lengths_[0];
      l2 = link_lengths_[1];
      l3 = link_lengths_[2];

      abd_angle = joint_angles[0];
      hip_angle = joint_angles[1];
      knee_angle = joint_angles[2];

      joint_angles_2r[0] = hip_angle;
      joint_angles_2r[1] = knee_angle;

      serial_2r_->forwardKinematics(joint_angles_2r, foot_pos_2r);

      // Foot position without considering abduction movement
      foot_pos_3r.x() = foot_pos_2r[0];          // x
      foot_pos_3r.y() = right_leg ? -l1 : l1;   // y
      foot_pos_3r.z() = foot_pos_2r[1];          // z

      // Rotate about x-axis to account for the abduction angle.
      ee_pos = utils::eulerToRotation(abd_angle, 0.0, 0.0) * foot_pos_3r;


      // Forward kinematics
      // x = -l2*sin(th2) -l3*sin(th2+th3);
      // y = l1*cos(th1) + l2*sin(th1)*cos(th2) + l3*sin(th1)*cos(th2+th3);
      // z = l1*sin(th1) + l2*cos(th1)*cos(th2) + l3*cos(th1)*cos(th2+th3);


      assert(ee_pos.hasNaN() != true && "Invalid! In serial 3r forward kinematics, output ee_pos has NaN element");
      return;
    }

    /**
     * @brief Function to provide foraward Jacobian matrix for the serial-3R manipulator
     *
     * This function provides the forward Jacobian matrix of the end-effector w.r.t the hip
     * joint in the frame located at the hip.
     *
     * \param[in] leg_name: ID of the leg, "fl", "fr", "bl", "br"
     *
     * \param[in] joint_angles: 3 dimensional vector of joint angles
     *
     * \return 3x3 forward Jacobian matrix
     */
    utils::Matrix3d jacobianMatrix(std::string leg_name, utils::Vector3d joint_angles)
    {
      assert(joint_angles.size() == 3 && "Invalid! In serial 3r forward kinematics, input joint_angles size != 3");
      assert(joint_angles.hasNaN() != true && "Invalid! In serial 3r forward kinematics, input joint_angles has NaN element");

      double th1, th2, th3;
      double l1, l2, l3;
      utils::Matrix3d jacobian_matrix;

      l1 = link_lengths_[0];
      l2 = link_lengths_[1];
      l3 = link_lengths_[2];

      if ((leg_name == "fr") || (leg_name == "br"))
        l1 = -l1;

      th1 = joint_angles[0];
      th2 = joint_angles[1];
      th3 = joint_angles[2];

      // Forward kinematics
      // x = -l2*sin(th2) -l3*sin(th2+th3);
      // y = l1*cos(th1) + l2*sin(th1)*cos(th2) + l3*sin(th1)*cos(th2+th3);
      // z = l1*sin(th1) + l2*cos(th1)*cos(th2) + l3*cos(th1)*cos(th2+th3);

      jacobian_matrix(0,0) = 0;
      jacobian_matrix(0,1) = -l2*cos(th2) - l3*cos(th2+th3);
      jacobian_matrix(0,2) = -l3*cos(th2+th3);
      jacobian_matrix(1,0) = -l1*sin(th1) + l2*cos(th1)*cos(th2) + l3*cos(th1)*cos(th2+th3);
      jacobian_matrix(1,1) = -l2*sin(th1)*sin(th2) - l3*sin(th1)*sin(th2+th3);
      jacobian_matrix(1,2) = -l3*sin(th1)*sin(th2+th3);
      jacobian_matrix(2,0) = l1*cos(th1) + l2*sin(th1)*cos(th2) + l3*sin(th1)*cos(th2+th3);
      jacobian_matrix(2,1) = +l2*cos(th1)*sin(th2) +l3*cos(th1)*sin(th2+th3);
      jacobian_matrix(2,2) = +l3*cos(th1)*sin(th2+th3);

      return jacobian_matrix;
    }

    /**
     * @brief Inverse velocity kinematics for a serial-3R chain.
     *
     * \param[in] leg_name: Name of the leg, "fl", "fr", "bl", "br"
     *
     * \param[in] joint_angles : Joint angles of the chain
     *
     * \param[in] ee_velocity : Velocity of the end-effector
     *
     * \param[out] joint_velocity : The joint velocity obtained as a solution are provided
     *                           in this vector.
     *
     * \return int : Specifies if the solution exists for the given input.
     *                Returns 0 if solution exists.
     *                Returns -1 if solution does not exis.
     */
    int inverseVelocityKinematics(
        std::string leg_name,
        utils::Vector3d joint_angles,
        utils::Vector3d ee_velocity,
        utils::Vector3d& joint_velocity
        )
    {
      assert(joint_angles.hasNaN() != true &&
          "Invalid! In serial 3r Inverse Velocity Kinematics, input joint_angles has NaN element");

      int valid_ik=0;

      utils::Matrix3d jacobian_matrix;
      utils::Matrix3d jacobian_matrix_inverse;
      double determinant;

      jacobian_matrix = jacobianMatrix(leg_name, joint_angles);

      determinant = jacobian_matrix.determinant();
      if(fabs(determinant) < 1e-4)
      {
        valid_ik = -1;
      }
      else
      {
        jacobian_matrix_inverse = jacobian_matrix.inverse();
        joint_velocity = jacobian_matrix_inverse * ee_velocity;
      }

      return valid_ik;
    }

    /**
     * @brief Forward velocity kinematics for a serial-3R chain.
     *
     * \param[in] leg_name: Name of the leg, "fl", "fr", "bl", "br"
     *
     * \param[in] joint_angles : Joint angles of the chain
     *
     * \param[in] joint_velocity : Joint velocities
     *
     * \param[out] ee_velocity : Velocity of the end-effector

     * \return int : Specifies if the solution exists for the given input.
     *                Returns 0 if solution exists.
     *                Returns -1 if solution does not exis.
     */
    int forwardVelocityKinematics(
        std::string leg_name,
        utils::Vector3d joint_angles,
        utils::Vector3d joint_velocity,
        utils::Vector3d& ee_velocity
        )
    {
      assert(joint_angles.hasNaN() != true &&
          "Invalid! In serial 3r Forward Velocity Kinematics, input joint_angles has NaN element");

      int valid_fk=0;

      utils::Matrix3d jacobian_matrix;

      jacobian_matrix = jacobianMatrix(leg_name, joint_angles);

      ee_velocity = jacobian_matrix * joint_velocity;

      return valid_fk;
    }

    /**
     * @brief Function to check if the specified point lies in the workspace.
     *
     * \param[in] ee_pos: End-effector position (vector of dimension 3)
     *
     * \return bool: true if the point lies in the workspace, false otherwise.
     */
    bool inWorkspace(utils::Vector3d ee_pos)
    {
      double x, y, z;
      double l1;
      bool   in_workspace;
      double h;
      double r;
      utils::Vector2d foot_pos_2r(0, 0);

      x = ee_pos.x();
      y = ee_pos.y();
      z = ee_pos.z();

      l1 = link_lengths_[0];

      r = ee_pos.tail<2>().norm(); // <y, z>

      // Out of workspace
      if (r < l1)
        return false;

      h = sqrt(r * r - l1 * l1);

      foot_pos_2r[0] = x;
      foot_pos_2r[1] = -h;

      in_workspace = serial_2r_->inWorkspace(foot_pos_2r);

      return in_workspace;
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
    utils::Vector3d searchSafePosition(utils::Vector3d desired_pos)
    {
      assert(desired_pos.hasNaN() != true && "Invalid! In serial 3r safety function input has NaN elements");

      const double  RADIAL_DISTANCE = 0.2; // radial distance for a point in workspace
      const double  DELTA_MAX = 0.001;
      const int     MAX_ITERATIONS = 20;

      utils::Vector3d p_in(0, 0, 0), p_out(0, 0, 0), p(0, 0, 0);
      utils::Vector3d unit_vector(0, 0, 0);
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

      std::cout << "WARNING: Serial 3r Inverse Kinematics safety is being applied" << std::endl;
      assert(p_in.hasNaN() != true && "Invalid! In serial 3r safety function ouput has NaN elements");
      return p_in;
    }

    utils::Vector3d forwardKinematics(std::string leg_name, utils::Vector3d joint_angles)
    {
      utils::Vector3d ee_pos;
      forwardKinematics(leg_name, joint_angles, ee_pos);
      return ee_pos;
    }

    utils::Vector3d inverseKinematics(std::string leg_name, utils::Vector3d ee_pos, char branch)
    {
      utils::Vector3d joint_angles;
      int validity;
      validity = inverseKinematics(leg_name, ee_pos, branch, joint_angles);
      return joint_angles;
    }
  };

} // end namespace

#endif //__SERIAL3R_KINEMATICS_H__
