/*
 *  file: estimator_state_interface.h
 *
 *  Created: 9 Feb, 2022
 *  Author : Aditya Sagi
 */

#pragma once
#include <cassert>
#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>

namespace hardware_interface
{
  class EstimatorStateHandle
  {
    public:
      EstimatorStateHandle() = default;


      EstimatorStateHandle(const std::string& name, double* valptr): name_(name), valptr_(valptr)
      {
        if(!valptr)
        {
          throw HardwareInterfaceException("Cannot create handle '" + name + "'. Data pointer is null.");
        }
      }

      std::string getName() const 
      {
        return name_;
      }
      
      double getValue() const
      { 
        assert(valptr_);
        return *valptr_;
      }

      void setValue(const double val)
      {
        assert(valptr_);
        *valptr_ = val;
        return;
      }

      const double* getEstimatePtr() const
      {
        return valptr_;
      }

    private:
      std::string name_;
      double* valptr_ = {nullptr};
  };


class EstimatorStateInterface : public HardwareResourceManager<EstimatorStateHandle> {};
}
