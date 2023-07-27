/* file: imu_interface.cpp
 *
 * Created: 23 Sep, 2021
 * Author : Aditya Sagi
 */

#include <iostream>
#include <stdio.h>
#include <string>

#include <xscontroller/xscontrol_def.h>
#include <xscontroller/xsdevice_def.h>
#include <xscontroller/xsscanner.h>
#include <xstypes/xsoutputconfigurationarray.h>
#include <xstypes/xsdatapacket.h>
#include <xstypes/xstime.h>
#include <xscommon/xsens_mutex.h>

#include "stoch3_hardware_interface/imu_interface.h"

Journaller *gJournal = 0;



IMUInterface::IMUInterface() : initialized_(false) {}

IMUInterface::~IMUInterface()
{
  if(initialized_)
  {
    std::cout << "Closing port..." << std::endl;
    control_->closePort(mtPort_.portName().toStdString());

    std::cout << "Freeing XsControl object..." << std::endl;
    control_->destruct();
  }
};

bool IMUInterface::initialize(void)
{
  control_ = XsControl::construct();
  assert(control_ != 0);

  // Lambda function for error handling
  auto handleError = [=](std::string errorString)
  {
    control_->destruct();
    std::cout << errorString << std::endl;
    return false;
  };

  std::cout << "Scanning for devices..." << std::endl;
  XsPortInfoArray portInfoArray = XsScanner::scanPorts();

  // Find an MTi device
  for (auto const &portInfo : portInfoArray)
  {
    if (portInfo.deviceId().isMti() || portInfo.deviceId().isMtig())
    {
      mtPort_ = portInfo;
      break;
    }
  }

  std::cout << "Found a device with ID: " 
    << mtPort_.deviceId().toString().toStdString() 
    << " @ port: " << mtPort_.portName().toStdString() 
    << ", baudrate: " << mtPort_.baudrate() << std::endl;

  std::cout << "Opening port..." << std::endl;
  if (!control_->openPort(mtPort_.portName().toStdString(), mtPort_.baudrate()))
    return handleError("Could not open port. Aborting.");

  // Get the device object
  device_ = control_->device(mtPort_.deviceId());
  assert(device_ != 0);

  std::cout << "Device: " << device_->productCode().toStdString()
    << ", with ID: " << device_->deviceId().toString() 
    << " opened." << std::endl;

  // Create and attach callback handler to device
  device_->addCallbackHandler(&callback_);

  // Put the device into configuration mode before configuring the device
  std::cout << "Putting device into configuration mode..." << std::endl;
  if (!device_->gotoConfig())
    return handleError("Could not put device into configuration mode. Aborting.");

  std::cout << "Configuring the device..." << std::endl;

  // Important for Public XDA!
  // Call this function if you want to record a mtb file:
  XsOutputConfigurationArray configArray;
  configArray.push_back(XsOutputConfiguration(XDI_PacketCounter, 0));
  configArray.push_back(XsOutputConfiguration(XDI_SampleTimeFine, 0));
  configArray.push_back(XsOutputConfiguration(XDI_Quaternion, 400));
  configArray.push_back(XsOutputConfiguration(XDI_Acceleration, 400));
  configArray.push_back(XsOutputConfiguration(XDI_RateOfTurn, 400));
  configArray.push_back(XsOutputConfiguration(XDI_MagneticField, 100));

  if (!device_->setOutputConfiguration(configArray))
    return handleError("Could not configure MTi device. Aborting.");

  std::cout << "Putting device into measurement mode..." << std::endl;
  if (!device_->gotoMeasurement())
    return handleError("Could not put device into measurement mode. Aborting.");

  initialized_ = true;
  return true;
}


bool IMUInterface::read(IMUData& data)
{
  bool ret = false;

  data.ax = std::numeric_limits<double>::quiet_NaN();
  data.ay = std::numeric_limits<double>::quiet_NaN();
  data.az = std::numeric_limits<double>::quiet_NaN();
  data.gx = std::numeric_limits<double>::quiet_NaN();
  data.gy = std::numeric_limits<double>::quiet_NaN();
  data.gz = std::numeric_limits<double>::quiet_NaN();
  data.qw = std::numeric_limits<double>::quiet_NaN();
  data.qx = std::numeric_limits<double>::quiet_NaN();
  data.qy = std::numeric_limits<double>::quiet_NaN();
  data.qz = std::numeric_limits<double>::quiet_NaN();

  if (callback_.packetAvailable())
  {
    // Retrieve a packet
    XsDataPacket packet = callback_.getNextPacket();

    if (packet.containsCalibratedAcceleration() &&
        packet.containsCalibratedGyroscopeData() &&
        packet.containsOrientation())
    {
      XsVector acc = packet.calibratedAcceleration();
      XsVector gyr = packet.calibratedGyroscopeData();
      XsQuaternion quaternion = packet.orientationQuaternion();
     
      // Accelerometer 
      data.ax = (double)acc[0];
      data.ay = (double)acc[1];
      data.az = (double)acc[2];
      
      // Gyroscope
      data.gx = (double)gyr[0];
      data.gy = (double)gyr[1];
      data.gz = (double)gyr[2];
      
      // Quaternion
      data.qw = (double) quaternion.w();
      data.qx = (double) quaternion.x();
      data.qy = (double) quaternion.y();
      data.qz = (double) quaternion.z();
    
      ret = true;
    }
  }
  return ret;
}
