/*
 * file: control_state_machine.h
 *
 * Created: 30 Oct, 2021
 * Author : Aditya Sagi
 */

#include <string>
#include <mutex>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>

#include "stoch3_state_machine/idle_state.h"
#include "stoch3_state_machine/hold_state.h"
#include "stoch3_state_machine/sit_state.h"
#include "stoch3_state_machine/stand_state.h"
#include "stoch3_state_machine/pose_control_state.h"
#include "stoch3_state_machine/walking_control_state.h"


class ControlStateMachine
{

  public:
    ControlStateMachine(ros::NodeHandle nh): nh_(nh)
  {
    sm_.addState(&idle_state_);
    sm_.addState(&hold_state_);
    sm_.addState(&sit_state_);
    sm_.addState(&stand_state_);
    sm_.addState(&pc_state_);
    sm_.addState(&wc_state_);

    std::string idle_state_name   = idle_state_.getName();
    std::string hold_state_name   = hold_state_.getName();
    std::string sit_state_name    = sit_state_.getName();
    std::string stand_state_name  = stand_state_.getName();
    std::string pc_state_name     = pc_state_.getName();
    std::string wc_state_name     = wc_state_.getName();


    sm_.addTransition(idle_state_name, "Hold", hold_state_name);
    sm_.addTransition(hold_state_name, "Idle", idle_state_name);
    
    sm_.addTransition(hold_state_name, "Sit", sit_state_name);
    sm_.addTransition(sit_state_name, "Hold", hold_state_name);
    sm_.addTransition(stand_state_name, "Hold", hold_state_name);
    
    sm_.addTransition(sit_state_name, "Stand", stand_state_name);
    sm_.addTransition(stand_state_name, "Sit", sit_state_name);
        
    sm_.addTransition(sit_state_name, "PoseControl", pc_state_name);
    sm_.addTransition(pc_state_name, "Sit", sit_state_name);
    
    sm_.addTransition(stand_state_name, "PoseControl", pc_state_name);
    sm_.addTransition(pc_state_name, "Stand", stand_state_name);
    
    sm_.addTransition(stand_state_name, "WalkingControl", wc_state_name);
    sm_.addTransition(wc_state_name, "Stand", stand_state_name);
    
    sm_.addTransition(wc_state_name, "Sit", sit_state_name);
    
    sm_.addTransition(wc_state_name, "PoseControl", pc_state_name);
    sm_.addTransition(pc_state_name, "WalkingControl", wc_state_name);
    
    cmd_sub = nh.subscribe<sensor_msgs::Joy>("joy", 1, &ControlStateMachine::commandCB, this);
    state_pub = nh.advertise<std_msgs::String>("state", 1);

    pub_timer = nh.createTimer(ros::Duration(1/PUBLISH_RATE), &ControlStateMachine::publishState, this);

    prev_msg.buttons.resize(10);
    for(auto i=0; i<10; i++)
    {
      prev_msg.buttons[i] = 0;
    }
  }

    ~ControlStateMachine()
    {
      std::lock_guard<std::mutex> guard(mtx);
      sm_.stop();
    }

    void start()
    {
      std::lock_guard<std::mutex> guard(mtx);
      sm_.start();
      return;
    }

    void publishState(const ros::TimerEvent& e)
    {
      std::lock_guard<std::mutex> guard(mtx);
      std_msgs::String msg;
      msg.data = sm_.getState();
      state_pub.publish(msg);
    }

    void commandCB(const sensor_msgs::Joy::ConstPtr& msg)
    {
      std::lock_guard<std::mutex> guard(mtx);

      // Mapping for Logitech F710 joystick
      if((msg->buttons[2] == 1) && (prev_msg.buttons[2] == 0))  // X
      {
        sm_.trigger("Sit");
      }
      else if((msg->buttons[3] == 1) && (prev_msg.buttons[3] == 0)) // Y
      {
        sm_.trigger("Stand");
      }
      else if((msg->buttons[0] == 1) && (prev_msg.buttons[0] == 0)) // A
      {
        sm_.trigger("PoseControl");
      }
      else if((msg->buttons[1] == 1) && (prev_msg.buttons[1] == 0)) // B
      {
        sm_.trigger("WalkingControl");
      }
      else if((msg->buttons[7] == 1) && (prev_msg.buttons[7] == 0)) // Start
      {
        sm_.trigger("Hold");
      }
      else if((msg->buttons[6] == 1) && (prev_msg.buttons[6] == 0)) // Back
      {
        sm_.trigger("Idle");
      }
    
      prev_msg = *msg;
    }


  private:
    const double PUBLISH_RATE=25.0;
  
    ros::NodeHandle nh_;
    ros::Subscriber cmd_sub;
    ros::Publisher state_pub;

    ros::Timer pub_timer;

    std::mutex mtx;

    sensor_msgs::Joy prev_msg;

    utils::StateMachine sm_;
    IdleState idle_state_;
    HoldState hold_state_;
    SitState sit_state_;
    StandState stand_state_;    
    PoseControlState pc_state_;
    WalkingControlState wc_state_;
};
