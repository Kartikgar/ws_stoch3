///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2012, hiDOF INC.
// Copyright (C) 2021, IISc
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

/// \author Wim Meeussen
/// \author Aditya Sagi

#pragma once

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <cassert>
#include <string>

namespace hardware_interface
{

/**
 * \brief A handle used to read the state of a single motor.
 */
class MotorStateHandle
{
public:
  MotorStateHandle() = default;

  /**
   * \param name The name of the motor
   * \param mode        A pointer to the storage for this motor's mode
   * \param fault       A pointer to the storage for this motor's fault
   * \param voltage     A pointer to the storage for this motor's voltage
   * \param temperature A pointer to the storage for this motor's temperature
   */
  MotorStateHandle(const std::string& name, 
      const int16_t* mode, 
      const int8_t* fault, 
      const double* voltage, 
      const double* temperature)
    : name_(name), mode_(mode), fault_(fault), voltage_(voltage), temperature_(temperature)
  {
    if (!mode)
    {
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. Mode data pointer is null.");
    }
    if (!fault)
    {
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. Fault data pointer is null.");
    }
    if (!voltage)
    {
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. Voltage data pointer is null.");
    }
    if (!temperature)
    {
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. Temperature data pointer is null.");
    }
  }

  std::string getName() const {return name_;}
  int16_t getMode()        const {assert(mode_); return *mode_;}
  int8_t  getFault()       const {assert(fault_); return *fault_;}
  double  getVoltage()     const {assert(voltage_); return *voltage_;}
  double  getTemperature() const {assert(temperature_); return *temperature_;}

  const int16_t* getModePtr() const {return mode_;}
  const int8_t* getFaultPtr() const {return fault_;}
  const double* getVoltagePtr()   const {return voltage_;}
  const double* getTemperaturePtr()   const {return temperature_;}

private:
  std::string name_;
  const int16_t* mode_          = {nullptr};
  const int8_t*  fault_         = {nullptr};
  const double*  voltage_       = {nullptr};
  const double*  temperature_   = {nullptr};
};

/** \brief Hardware interface to support reading the state of an array of motors
 *
 * This \ref HardwareInterface supports reading the state of an array of named
 * motors, each of which has some mode, fault, voltage and temperature.
 *
 */
class MotorStateInterface : public HardwareResourceManager<MotorStateHandle> {};

}
