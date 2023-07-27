/*
 * file joy_interface_node.cpp
 *
 * Created : 1 Nov, 2021
 * Author  : Aditya Sagi
 */

#include "ros/ros.h"
#include "stoch3_teleop/joy_interface.h"

int main(int argc, char** argv )
{
    ros::init(argc, argv, "joy_interface_node");

    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    JoyInterface joy_interface(&nh, &nh_private);
    
    ros::spin();
    return 0;
}
