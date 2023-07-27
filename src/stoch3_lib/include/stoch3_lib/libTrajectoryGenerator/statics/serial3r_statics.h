/*
 * Copyright 2021, StochLab
 *
 * Author: Shashank R
 * Date:   Jan, 2022
 */

#ifndef __SERIAL3R_STATICS_H__
#define __SERIAL3R_STATICS_H__

#include <iostream>
#include <assert.h>
#include <math.h>

#include "utils/transformations.h"
#include "kinematics/serial3r_kinematics.h"

namespace statics
{

  /**
   * @brief Class for the statics of a serial-3R chain.
   *
   * This class implements the statics of a planar serial-3R chain.
   * The task is to find the joint torques given the force vector and vice-versa.
   *
   */
  class Serial3RStatics
  {

  private:
    utils::Vector3d link_lengths_;
    kine::Serial3RKinematics *serial_3r_;
    const double ep_tol = 1e-10;

  public:
    Serial3RStatics(utils::Vector3d link_lengths) :
      link_lengths_(link_lengths)
    {
      serial_3r_ = new kine::Serial3RKinematics(link_lengths);
    }

    Serial3RStatics() : link_lengths_({ 1.0, 1.0, 1.0 }) {}

    /**
     * @brief Computation of joint torques of the serial 3-R chain required to apply
     *        the desired force on the environment.
     *
     * This function computes the joint torques given the force applied by the end-effector
     * on the environment.The assumptions made in the serial 3r kinematics holds here as well.
     *
     * \param[in] leg_name : The leg under consideration - "fl", "fr", "bl", "br"
     *
     * \param[in] joint_angles : Given joint angles of the 3r chain.
     *
     * \param[in] foot_force : Force (applied by the end-effector on the environment)
     *                         vector at the end-effector of the 3-r chain
     *
     * \param[out] joint_torques : Torques to be exerted at the joints to generate the
     *                             the desired force vector
     *
     * \return int : Specifies if the input configuration is singular/non-singular.
     *                Returns 1 if the configuration is singular.
     *                Returns 0 if the configuration is non-singular.
     */
    int computeJointTorques(std::string leg_name, utils::Vector3d joint_angles,
        utils::Vector3d foot_force, utils::Vector3d& joint_torques)
    {
      // Compute the jacobian matrix
      utils::Matrix3d jacobian = serial_3r_->jacobianMatrix(leg_name, joint_angles);
      // Compute the determinant of the jacobian matrix
      double jacobian_det = jacobian.determinant(); //TODO: Check if this function exists

      // Joint torques are computed in any case, i.e., if the configuration is singular
      // or non-singular
      utils::Matrix3d jacobian_transpose = jacobian.transpose();
      joint_torques = jacobian_transpose * foot_force;

      // Check for singularity of the jacobian
      if(abs(jacobian_det) <= ep_tol)
      {
        return 1;
      }
      else
      {
        return 0;
      }
    }

    /**
     * @brief Computation of end-effector forces of a serial-3r chain given the joint torques.
     *
     * This function computes the end-effector forces generated for the given joint-torques
     * The assumptions made in the serial 3r kinematics holds here as well.
     *
     * \param[in] leg_name : The leg under consideration - "fl", "fr", "bl", "br"
     *
     * \param[in] joint_angles : Given joint angles of the 3r chain.
     *
     * \param[in] joint_torques : Torques exerted by the joint of the 3-r chain
     *
     * \param[out] foot_force : Computed end-effector forces exerted when the given
     *                          joint-torques are applied
     *
     * \return int : Specifies if the input configuration is singular/non-singular.
     *                Returns 1 if the configuration is singular.
     *                Returns 0 if the configuration is non-singular.
     */
    int computeFootForce(std::string leg_name, utils::Vector3d joint_angles,
        utils::Vector3d joint_torques, utils::Vector3d& foot_force)
    {

      // Compute the jacobian matrix
      utils::Matrix3d jacobian = serial_3r_->jacobianMatrix(leg_name, joint_angles);
      // Compute the determinant of the jacobian matrix
      double jacobian_det = jacobian.determinant(); //TODO: Check if this function exists

      // Check for singularity of the jacobian
      if(abs(jacobian_det) <= ep_tol)
      {
        return 1;
      }
      else
      {
        // Foot forces are computed if the configuration is non-singular
        utils::Matrix3d jacobian_transpose = jacobian.transpose();
        foot_force = jacobian_transpose.inverse() * joint_torques;
        return 0;
      }
    }

  };

} // end namespace

#endif //__SERIAL3R_STATICS_H_
