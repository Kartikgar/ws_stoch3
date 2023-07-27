#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "gazebo_msgs/SetModelConfiguration.h"
#include "gazebo_msgs/SetModelState.h"
#include "gazebo_msgs/LinkState.h"
#include <cstdlib>
#include <map>
#include <unistd.h>

int pause_physics(ros::NodeHandle& nh)
{
  ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
  std_srvs::Empty srv;
  client.call(srv);
}

int unpause_physics(ros::NodeHandle& nh)
{
  ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
  std_srvs::Empty srv;
  client.call(srv);
}

int set_model_configuration(ros::NodeHandle& nh)
{
  ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");

  gazebo_msgs::SetModelConfiguration srv;
  srv.request.model_name = std::string("stoch3");
  srv.request.urdf_param_name = std::string("robot_description");

  std::map<std::string, double> joint_map = {
    {"fl_abd_joint",    0.0},
    {"fl_hip_joint",    1.56},
    {"fl_knee_joint",   -3.0},
    {"fr_abd_joint",    0.0},
    {"fr_hip_joint",    1.56},
    {"fr_knee_joint",   -3.0},
    {"bl_abd_joint",    0.0},
    {"bl_hip_joint",    1.56},
    {"bl_knee_joint",   -3.0},
    {"br_abd_joint",    0.0},
    {"br_hip_joint",    1.56},
    {"br_knee_joint",   -3.0},
  };

  for(auto& it : joint_map)
  {
    srv.request.joint_names.push_back(it.first);
    srv.request.joint_positions.push_back(it.second);
  }
  
  if (client.call(srv))
  {
    ROS_INFO("Response: %d, %s", srv.response.success, srv.response.status_message.c_str());
    return 0;
  }
  else
  {
    ROS_ERROR("Failed to call service /gazebo/set_model_configuration");
    return -1;
  }

  return 0;
}

int set_model_state(ros::NodeHandle& nh)
{
  ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

  gazebo_msgs::SetModelState srv;


  srv.request.model_state.model_name="stoch3";
  srv.request.model_state.reference_frame="world";
  
  // Position
  srv.request.model_state.pose.position.x = 0;
  srv.request.model_state.pose.position.y = 0;
  srv.request.model_state.pose.position.z = 0.2;
  
  // Orientation
  srv.request.model_state.pose.orientation.x = 0;
  srv.request.model_state.pose.orientation.y = 0;
  srv.request.model_state.pose.orientation.z = 0;
  srv.request.model_state.pose.orientation.w = 1.0;

  // Twist
  srv.request.model_state.twist.linear.x = 0;
  srv.request.model_state.twist.linear.y = 0;
  srv.request.model_state.twist.linear.z = 0;
  srv.request.model_state.twist.angular.x = 0;
  srv.request.model_state.twist.angular.y = 0;
  srv.request.model_state.twist.angular.z = 0;

  if (client.call(srv))
  {
    ROS_INFO("Response: %d, %s", srv.response.success, srv.response.status_message.c_str());
    return 0;
  }
  else
  {
    ROS_ERROR("Failed to call service /gazebo/set_model_configuration");
    return -1;
  }

  return 0;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "set_model_configuration_node");
  ros::NodeHandle nh;

  pause_physics(nh);
  sleep(2.0);

  set_model_configuration(nh);  

  set_model_state(nh);
  
  sleep(2.0);
  unpause_physics(nh);


  return 0;
}
