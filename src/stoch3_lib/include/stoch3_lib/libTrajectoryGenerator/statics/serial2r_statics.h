/*
 * Copyright 2021, StochLab
 *
 * Author: Shashank R
 * Date:   Dec, 2021
 */

#ifndef __SERIAL2R_STATICS_H__
#define __SERIAL2R_STATICS_H__

#include <iostream>
#include <assert.h>
#include <math.h>
#include <iostream>

#include "utils/transformations.h"
#include "kinematics/serial2r_kinematics.h"

namespace statics
{

  /**
   * @brief Class for the statics of a serial-2R chain.
   *
   * This class implements the statics of a planar serial-2R chain.
   * The task is to find the joint torques given the force vector and vice-versa.
   * The force is applied by the end-effector on the environment.
   * It assumes that the chain lies in the x-z plane with x pointing forward and z pointing
   * upward. The joints rotate about the y-axis.
   */
  class Serial2RStatics
  {

  private:
    utils::Vector2d link_lengths_;
    kine::Serial2RKinematics *serial_2r_;
    const double ep_tol = 1e-10;

  public:
    Serial2RStatics(utils::Vector2d link_lengths) :
      link_lengths_(link_lengths)
    {
      serial_2r_ = new kine::Serial2RKinematics(link_lengths);
    }

    Serial2RStatics() : link_lengths_({ 1.0, 1.0 }) {}

    /**
     * @brief Computation of joint torques of the serial 2-R chain given the force
     *        applied by the end-effector on the environment.
     *
     * The assumptions made in the serial 2r kinematics holds here as well.
     *
     * \param[in] joint_angles : Given joint angles of the 2r chain.
     *
     * \param[in] foot_force : Force applied by the end-effector of the 2-r chain
     *
     * \param[out] joint_torques : Torques to be exerted at the joints to generate the
     *                             the desired force vector
     *
     * \return int : Specifies if the input configuration is singular/non-singular.
     *                Returns 1 if the configuration is singular.
     *                Returns 0 if the configuration is non-singular.
     */
    int computeJointTorques(utils::Vector2d joint_angles, utils::Vector2d foot_force,
        utils::Vector2d& joint_torques)
    {

      // Compute the jacobian matrix
      utils::Matrix2d jacobian = serial_2r_->jacobianMatrix(joint_angles);
      // Compute the determinant of the jacobian matrix
      double jacobian_det = jacobian(0, 0)*jacobian(1, 1) - jacobian(0, 1)*jacobian(1, 0);

      // Joint torques are computed in any case, i.e., if the configuration is singular
      // or non-singular
      utils::Matrix2d jacobian_transpose = jacobian.transpose();
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
     * @brief Computation of force applied by end-effector on the environment give the joint torques
     *        of the serial-2r chain.
     *
     * The assumptions made in the serial 2r kinematics holds here as well.
     *
     * \param[in] joint_angles : Given joint angles of the 2r chain.
     *
     * \param[in] joint_torques : Torques exerted by the joint of the 2-r chain
     *
     * \param[out] foot_force : Force exerted by the end-effector of the 2r chain for the given
     *                          joint torques.
     *
     * \return int : Specifies if the input configuration is singular/non-singular.
     *                Returns 1 if the configuration is singular.
     *                Returns 0 if the configuration is non-singular.
     */
    int computeFootForce(utils::Vector2d joint_angles, utils::Vector2d joint_torques,
        utils::Vector2d& foot_force)
    {

      // Compute the jacobian matrix
      utils::Matrix2d jacobian = serial_2r_->jacobianMatrix(joint_angles);
      // Compute the determinant of the jacobian matrix
      double jacobian_det = jacobian(0, 0)*jacobian(1, 1) - jacobian(0, 1)*jacobian(1, 0);

      // Check for singularity of the jacobian
      if(abs(jacobian_det) <= ep_tol)
      {
        return 1;
      }
      else
      {
        // Foot forces are computed if the configuration is non-singular
        utils::Matrix2d jacobian_transpose = jacobian.transpose();
        foot_force = jacobian_transpose.inverse() * joint_torques;
        return 0;
      }
    }

  };

} // end namespace

#endif //__SERIAL2R_STATICS_H_
