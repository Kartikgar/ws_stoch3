///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2012, hiDOF INC.
// Copyright (C) 2021, IISc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of hiDOF, Inc. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/// \author Igor Kalevatykh
/// \author Aditya Sagi

#pragma once


#include <cassert>
#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/posvel_command_interface.h>

namespace hardware_interface
{

/** \brief A handle used to read and command a single joint. */
class PosVelEffJointHandle : public PosVelJointHandle
{
public:
  PosVelEffJointHandle() = default;

  /**
   * \param js This joint's state handle
   * \param cmd_pos       : A pointer to the storage for this joint's output command position
   * \param cmd_vel       : A pointer to the storage for this joint's output command velocity
   * \param cmd_eff       : A pointer to the storage for this joint's output command effort
   * \param kp_gain_scale : A pointer to the storage for this joint's kp gain scale
   * \param kd_gain_scale : A pointer to the storage for this joint's kd gain scale
   */
  PosVelEffJointHandle(
      const JointStateHandle& js,
      double* cmd_pos,
      double* cmd_vel,
      double* cmd_eff,
      double* kp_gain_scale,
      double* kd_gain_scale
      )
    : PosVelJointHandle(js, cmd_pos, cmd_vel),
    cmd_eff_(cmd_eff),
    kp_gain_scale_(kp_gain_scale),
    kd_gain_scale_(kd_gain_scale)
  {
    if (!cmd_eff)
    {
      throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command effort data pointer is null.");
    }
    if (!kp_gain_scale)
    {
      throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Kp gain scale data pointer is null.");
    }
    if (!kd_gain_scale)
    {
      throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Kd gain scale data pointer is null.");
    }
  }

  void setCommand(
      double cmd_pos,
      double cmd_vel,
      double cmd_eff
      )
  {
    setCommandPosition(cmd_pos);
    setCommandVelocity(cmd_vel);
    setCommandEffort(cmd_eff);
    setCommandKpScale(1.0);
    setCommandKdScale(1.0);
  }

  void setCommand(
      double cmd_pos,
      double cmd_vel,
      double cmd_eff,
      double kp_gain_scale,
      double kd_gain_scale
      )
  {
    setCommandPosition(cmd_pos);
    setCommandVelocity(cmd_vel);
    setCommandEffort(cmd_eff);
    setCommandKpScale(kp_gain_scale);
    setCommandKdScale(kd_gain_scale);
  }

  void setCommandEffort(double cmd_eff)
  {
    assert(cmd_eff_);
    *cmd_eff_ = cmd_eff;
  }

  double getCommandEffort() const
  {
    assert(cmd_eff_);
    return *cmd_eff_;
  }

  void setCommandKpScale(double kp_gain_scale)
  {
    assert(kp_gain_scale_);
    *kp_gain_scale_ = kp_gain_scale;
  }

  double getCommandKpScale() const
  {
    assert(kp_gain_scale_);
    return *kp_gain_scale_;
  }

  void setCommandKdScale(double kd_gain_scale)
  {
    assert(kd_gain_scale_);
    *kd_gain_scale_ = kd_gain_scale;
  }

  double getCommandKdScale() const
  {
    assert(kd_gain_scale_);
    return *kd_gain_scale_;
  }

private:
  double* cmd_eff_       = {nullptr};
  double* kp_gain_scale_ = {nullptr};
  double* kd_gain_scale_ = {nullptr};
};


/** \brief Hardware interface to support commanding an array of joints.
 *
 * This \ref HardwareInterface supports commanding joints by position, velocity &
 * effort together in one command.
 *
 * \note Getting a joint handle through the getHandle() method \e will claim that resource.
 *
 */
class PosVelEffJointInterface : public HardwareResourceManager<PosVelEffJointHandle, ClaimResources> {};

}
