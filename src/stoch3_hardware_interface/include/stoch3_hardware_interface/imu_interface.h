/*
 * file: imu_interface.h
 *
 * Created: 23 Sep, 2021
 * Author : Aditya Sagi
 */

#ifndef __IMU_INTERFACE_H__
#define __IMU_INTERFACE_H__

#include <list>

#include <xscontroller/xscontrol_def.h>
#include <xscontroller/xsdevice_def.h>
#include <xscontroller/xsscanner.h>
#include <xstypes/xsoutputconfigurationarray.h>
#include <xstypes/xsdatapacket.h>
#include <xstypes/xstime.h>
#include <xscommon/xsens_mutex.h>

class CallbackHandler : public XsCallback
{
  public:
    CallbackHandler(size_t maxBufferSize = 5)
      : m_maxNumberOfPacketsInBuffer(maxBufferSize), m_numberOfPacketsInBuffer(0)
    {
    }

    virtual ~CallbackHandler() throw()
    {
    }

    bool packetAvailable() const
    {
      xsens::Lock locky(&m_mutex);
      return m_numberOfPacketsInBuffer > 0;
    }

    XsDataPacket getNextPacket()
    {
      assert(packetAvailable());
      xsens::Lock locky(&m_mutex);
      XsDataPacket oldestPacket(m_packetBuffer.front());
      m_packetBuffer.pop_front();
      --m_numberOfPacketsInBuffer;
      return oldestPacket;
    }

  protected:
    void onLiveDataAvailable(XsDevice *, const XsDataPacket *packet) override
    {
      xsens::Lock locky(&m_mutex);
      assert(packet != 0);
      while (m_numberOfPacketsInBuffer >= m_maxNumberOfPacketsInBuffer)
        (void)getNextPacket();

      m_packetBuffer.push_back(*packet);
      ++m_numberOfPacketsInBuffer;
      assert(m_numberOfPacketsInBuffer <= m_maxNumberOfPacketsInBuffer);
    }

  private:
    mutable xsens::Mutex m_mutex;

    size_t m_maxNumberOfPacketsInBuffer;
    size_t m_numberOfPacketsInBuffer;
    std::list<XsDataPacket> m_packetBuffer;
};


typedef struct {
  double ax, ay, az; // accelerometer
  double gx, gy, gz; // gyroscope
  double qw, qx, qy, qz; // Quaternion
} IMUData;

class IMUInterface {

  public:
    IMUInterface();
    ~IMUInterface();
    bool initialize(void);
    bool read(IMUData&);

  private:
    CallbackHandler callback_;

    XsControl *control_;
    XsDevice *device_;
    XsPortInfo mtPort_;

    bool initialized_;
};



#endif // __IMU_INTERFACE_H__
