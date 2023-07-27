#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <stoch3_msgs/SitStandAction.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_sit_stand");

  actionlib::SimpleActionClient<stoch3_msgs::SitStandAction> ac("sit_stand", true);

  ac.waitForServer();

  stoch3_msgs::SitStandGoal goal;
  goal.posture = std::string("Stand");
  ac.sendGoal(goal);

  return 0;
}
