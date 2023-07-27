set(CMAKE_CXX_STANDARD 11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -include xscontroller/xscontrollerconfig.h -DHAVE_JOURNALLER")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -include xscontroller/xscontrollerconfig.h")

option(HAVE_LIBUSB "Use libusb" OFF)
if (HAVE_LIBUSB)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHAVE_LIBUSB")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DHAVE_LIBUSB")
endif()

include_directories(../)

add_library(xscontroller STATIC
  broadcastdevice.cpp
  callbackmanagerxda.cpp
  communicator.cpp
  communicatorfactory.cpp
  datalogger.cpp
  datapacketcache.cpp
  dataparser.cpp
  datapoller.cpp
  devicecommunicator.cpp
  devicefactory.cpp
  deviceredetector.cpp
  dotdevice.cpp
  dummy.cpp
  enumerateusbdevices.cpp
  enumexpanders.cpp
  iointerface.cpp
  iointerfacefile.cpp
  messageextractor.cpp
  messageserializer.cpp
  mtbdatalogger.cpp
  mtbfilecommunicator.cpp
  mtdevice.cpp
  mti3x0device.cpp
  mti6x0device.cpp
  mti7device.cpp
  mti8x0device.cpp
  mtibasedevice.cpp
  mtigdevice.cpp
  mtix00device.cpp
  mtix0device.cpp
  mtixdevice.cpp
  mtthread.cpp
  nmea_protocolhandler.cpp
  packeterrorrateestimator.cpp
  packetstamper.cpp
  protocolhandler.cpp
  protocolmanager.cpp
  proxycommunicator.cpp
  replymonitor.cpp
  replyobject.cpp
  restorecommunication.cpp
  scanner.cpp
  serialcommunicator.cpp
  serialinterface.cpp
  serialportcommunicator.cpp
  streaminterface.cpp
  supportedsyncsettings.cpp
  udev.cpp
  usbcommunicator.cpp
  usbinterface.cpp
  xdacommunicatorfactory.cpp
  xscontrol_def.cpp
  xsdef.cpp
  xsdevice_def.cpp
  xslibusb.cpp
  xsscanner.cpp
  xswinusb.cpp
  synclinegmt.c
  synclinemk4.c
  xsconnectivitystate.c
  xsdeviceconfiguration.c
  xsdeviceptrarray.c
  xsdevicestate.c
  xsrejectreason.c
  xsusbhubinfo.c
  )
