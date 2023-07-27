/*
 * file joy_interface.h
 *
 * Created : 1 Nov, 2021
 * Author  : Aditya Sagi
 */

#include <vector>
#include <time.h>
#include <fcntl.h>
#include <math.h>
#include <sys/ioctl.h>
#include <sys/poll.h>

#include <linux/joystick.h>

#include "ros/ros.h"

#include <sensor_msgs/Joy.h>

#define DEVICE_FILE "/dev/input/js0"

using namespace std;

class JoyInterface
{
  protected:
    int js_fd_;
    unsigned char num_axes_;
    unsigned char num_buttons_;
    bool joy_ready_;

    sensor_msgs::Joy joy_;

    ros::Publisher joy_pub_;

    ros::Timer loop_timer_; // A ros timer for calling control loop

    struct control_params
    {
      double loop_rate_ = 100;     // control loop frequency
      double dt_ = 1.0 / loop_rate_; // time step for libTrajectoryGenerator

    } control_params_;

  public:
    JoyInterface(ros::NodeHandle *nh, ros::NodeHandle *pnh)
    {
      // Open the joystick device file
      js_fd_ = open(DEVICE_FILE, O_RDONLY | O_NONBLOCK);
      if (js_fd_ < 0)
      {
        ROS_ERROR("Unable to open joystick device file!");
        return;
      }

      // Determine the number of axes and buttons provided by the joystick
      ioctl(js_fd_, JSIOCGAXES, &num_axes_);
      ioctl(js_fd_, JSIOCGBUTTONS, &num_buttons_);

      joy_pub_ = nh->advertise<sensor_msgs::Joy>("joy", 1);

      joy_.buttons.resize(num_buttons_);
      joy_.axes.resize(num_axes_);

      loop_timer_ = pnh->createTimer(ros::Duration(1 / control_params_.loop_rate_),
          &JoyInterface::controlLoop, this);
    }


    void controlLoop(const ros::TimerEvent &event)
    {
      struct js_event jse;
      int num;
      
      /** 
       * Process all the pending events, based on which axis is activated 
       * The data is mapped.
       */
      while (read(js_fd_, &jse, sizeof(jse)) > 0)
      {
        num = jse.number;
        if (jse.type == JS_EVENT_AXIS)
        {
          joy_.axes[num] = (double) jse.value/32767.0 ;
        }
        else if (jse.type == JS_EVENT_BUTTON)
        {
          joy_.buttons[num] = jse.value;
        }
      }

      joy_pub_.publish(joy_);
    }
};
