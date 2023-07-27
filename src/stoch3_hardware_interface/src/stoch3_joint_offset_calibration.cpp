/**
 * @file stoch3_joint_offset_calibration.cpp
 *
 * Created : 17 October, 2022
 * Author  : Aditya Sagi
 */

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Trigger.h>

#define FL_ABD  0
#define FL_HIP  1
#define FL_KNEE 2
#define FR_ABD  3
#define FR_HIP  4
#define FR_KNEE 5
#define BL_ABD  6
#define BL_HIP  7
#define BL_KNEE 8
#define BR_ABD  9
#define BR_HIP  10
#define BR_KNEE 11

// Min joint angle
#define MIN_FL_ABD  -0.784
#define MIN_FL_HIP  -0.745
#define MIN_FL_KNEE -2.900
#define MIN_FR_ABD  -0.784
#define MIN_FR_HIP  -0.745
#define MIN_FR_KNEE -2.900
#define MIN_BL_ABD  -0.784
#define MIN_BL_HIP  -1.007
#define MIN_BL_KNEE -2.900
#define MIN_BR_ABD  -0.784
#define MIN_BR_HIP  -1.007
#define MIN_BR_KNEE -2.900

// Max joint angle
#define MAX_FL_ABD  0.789
#define MAX_FL_HIP  2.064
#define MAX_FL_KNEE 0
#define MAX_FR_ABD  0.789
#define MAX_FR_HIP  2.064
#define MAX_FR_KNEE 0
#define MAX_BL_ABD  0.789
#define MAX_BL_HIP  1.797
#define MAX_BL_KNEE 0
#define MAX_BR_ABD  0.789
#define MAX_BR_HIP  1.797
#define MAX_BR_KNEE 0

class Stoch3JointOffsetCalibration 
{
  private:
    ros::NodeHandle* nh_;
    ros::NodeHandle* pnh_;

    std::vector<double> min_pos;
    std::vector<double> max_pos;

    std::vector<double> expected_min_pos;
    std::vector<double> expected_max_pos;
    
    std::vector<double> zero_offset_;

    ros::Subscriber joint_state_sub_;

    ros::ServiceServer print_srv_;

    void jointStateCB_(const sensor_msgs::JointStateConstPtr& msg)
    {
      for( auto i=0; i<12; i++)
      {
        if(msg->name[i] == "fl_abd_joint")
          update_min_max__(FL_ABD, msg->position[i]);
        else if(msg->name[i] == "fl_hip_joint")
          update_min_max__(FL_HIP, msg->position[i]);
        else if(msg->name[i] == "fl_knee_joint")
          update_min_max__(FL_KNEE, msg->position[i]);

        else if(msg->name[i] == "fr_abd_joint")
          update_min_max__(FR_ABD, msg->position[i]);
        else if(msg->name[i] == "fr_hip_joint")
          update_min_max__(FR_HIP, msg->position[i]);
        else if(msg->name[i] == "fr_knee_joint")
          update_min_max__(FR_KNEE, msg->position[i]);

        else if(msg->name[i] == "bl_abd_joint")
          update_min_max__(BL_ABD, msg->position[i]);
        else if(msg->name[i] == "bl_hip_joint")
          update_min_max__(BL_HIP, msg->position[i]);
        else if(msg->name[i] == "bl_knee_joint")
          update_min_max__(BL_KNEE, msg->position[i]);

        else if(msg->name[i] == "br_abd_joint")
          update_min_max__(BR_ABD, msg->position[i]);
        else if(msg->name[i] == "br_hip_joint")
          update_min_max__(BR_HIP, msg->position[i]);
        else if(msg->name[i] == "br_knee_joint")
          update_min_max__(BR_KNEE, msg->position[i]);
      }
    }

    // Function to update the min and max values of the positions
    void update_min_max__(
        int joint_id,
        double pos
        )
    {
      if (pos < min_pos[joint_id])
        min_pos[joint_id] = pos;
      else if (pos > max_pos[joint_id])
        max_pos[joint_id] = pos;
    }

    // Calculate zero offset for ABD and HIP joints based on the joint limits
    // Angles provided in radians.
    // Min and Max values for the desired joint angles are measured with respect
    // to the correct zero reference.
    double zeroOffset_(
        double min_desired, 
        double max_desired, 
        double min_actual, 
        double max_actual
        )
    {
      double zero_fraction;
      double zero_offset;

      zero_fraction = (0 - min_desired)/ (max_desired - min_desired);
      zero_offset = min_actual + zero_fraction*(max_actual - min_actual);

      return zero_offset;
    }

    // Zero offset calculate for the knee actuator.
    // The offset is calculated only based on teh minimum recorded joint
    // value as this reference is sufficient for referencing a known position.
    // The minimum value is when the knee is fully folded.
    double zeroOffsetKnee_(
        double min_desired, 
        double max_desired, 
        double min_actual, 
        double max_actual
        )
    {
      double zero_offset;

      zero_offset = min_actual - MIN_FL_KNEE; // Minimum joint angle is same for all knee joints

      return zero_offset;
    }

    // Determine the joint offset calibration and print data to screen
    bool printSrvCB_(
        std_srvs::Trigger::Request &req, 
        std_srvs::Trigger::Response &resp
        )
    {
      zero_offset_.resize(12);

      zero_offset_[FL_ABD] = zeroOffset_(
          expected_min_pos[FL_ABD], expected_max_pos[FL_ABD], min_pos[FL_ABD], max_pos[FL_ABD]
          );
      zero_offset_[FL_HIP] = zeroOffset_(
          expected_min_pos[FL_HIP], expected_max_pos[FL_HIP], min_pos[FL_HIP], max_pos[FL_HIP]
          );
      zero_offset_[FL_KNEE] = zeroOffsetKnee_(
          expected_min_pos[FL_KNEE], expected_max_pos[FL_KNEE], min_pos[FL_KNEE], max_pos[FL_KNEE]
          );
     
      zero_offset_[FR_ABD] = zeroOffset_(
          expected_min_pos[FR_ABD], expected_max_pos[FR_ABD], min_pos[FR_ABD], max_pos[FR_ABD]
          );
      zero_offset_[FR_HIP] = zeroOffset_(
          expected_min_pos[FR_HIP], expected_max_pos[FR_HIP], min_pos[FR_HIP], max_pos[FR_HIP]
          );
      zero_offset_[FR_KNEE] = zeroOffsetKnee_(
          expected_min_pos[FR_KNEE], expected_max_pos[FR_KNEE], min_pos[FR_KNEE], max_pos[FR_KNEE]
          );

      zero_offset_[BL_ABD] = zeroOffset_(
          expected_min_pos[BL_ABD], expected_max_pos[BL_ABD], min_pos[BL_ABD], max_pos[BL_ABD]
          );
      zero_offset_[BL_HIP] = zeroOffset_(
          expected_min_pos[BL_HIP], expected_max_pos[BL_HIP], min_pos[BL_HIP], max_pos[BL_HIP]
          );
      zero_offset_[BL_KNEE] = zeroOffsetKnee_(
          expected_min_pos[BL_KNEE], expected_max_pos[BL_KNEE], min_pos[BL_KNEE], max_pos[BL_KNEE]
          );

      zero_offset_[BR_ABD] = zeroOffset_(
          expected_min_pos[BR_ABD], expected_max_pos[BR_ABD], min_pos[BR_ABD], max_pos[BR_ABD]
          );
      zero_offset_[BR_HIP] = zeroOffset_(
          expected_min_pos[BR_HIP], expected_max_pos[BR_HIP], min_pos[BR_HIP], max_pos[BR_HIP]
          );
      zero_offset_[BR_KNEE] = zeroOffsetKnee_(
          expected_min_pos[BR_KNEE], expected_max_pos[BR_KNEE], min_pos[BR_KNEE], max_pos[BR_KNEE]
          );
      
      ROS_INFO("#########################");
      ROS_INFO("Printing calibration data");
      ROS_INFO("#########################");

      ROS_INFO("motor_offsets:");
      ROS_INFO("  fl_abd_joint: %.3lf", zero_offset_[FL_ABD]);
      ROS_INFO("  fl_hip_joint: %.3lf", zero_offset_[FL_HIP]);
      ROS_INFO("  fl_knee_joint: %.3lf", zero_offset_[FL_KNEE]);

      ROS_INFO("  fr_abd_joint: %.3lf", zero_offset_[FR_ABD]);
      ROS_INFO("  fr_hip_joint: %.3lf", zero_offset_[FR_HIP]);
      ROS_INFO("  fr_knee_joint: %.3lf", zero_offset_[FR_KNEE]);

      ROS_INFO("  bl_abd_joint: %.3lf", zero_offset_[BL_ABD]);
      ROS_INFO("  bl_hip_joint: %.3lf", zero_offset_[BL_HIP]);
      ROS_INFO("  bl_knee_joint: %.3lf", zero_offset_[BL_KNEE]);
      
      ROS_INFO("  br_abd_joint: %.3lf", zero_offset_[BR_ABD]);
      ROS_INFO("  br_hip_joint: %.3lf", zero_offset_[BR_HIP]);
      ROS_INFO("  br_knee_joint: %.3lf", zero_offset_[BR_KNEE]);



      ROS_DEBUG("#########################");
      ROS_DEBUG("Printing minimum ");
      ROS_DEBUG("#########################");

      ROS_DEBUG("motor_offsets:");
      ROS_DEBUG("  fl_abd_joint: %.3lf", min_pos[FL_ABD]);
      ROS_DEBUG("  fl_hip_joint: %.3lf", min_pos[FL_HIP]);
      ROS_DEBUG("  fl_knee_joint: %.3lf", min_pos[FL_KNEE]);

      ROS_DEBUG("  fr_abd_joint: %.3lf", min_pos[FR_ABD]);
      ROS_DEBUG("  fr_hip_joint: %.3lf", min_pos[FR_HIP]);
      ROS_DEBUG("  fr_knee_joint: %.3lf", min_pos[FR_KNEE]);

      ROS_DEBUG("  bl_abd_joint: %.3lf", min_pos[BL_ABD]);
      ROS_DEBUG("  bl_hip_joint: %.3lf", min_pos[BL_HIP]);
      ROS_DEBUG("  bl_knee_joint: %.3lf", min_pos[BL_KNEE]);

      ROS_DEBUG("  br_abd_joint: %.3lf", min_pos[BR_ABD]);
      ROS_DEBUG("  br_hip_joint: %.3lf", min_pos[BR_HIP]);
      ROS_DEBUG("  br_knee_joint: %.3lf", min_pos[BR_KNEE]);



      ROS_DEBUG("#########################");
      ROS_DEBUG("Printing maximum ");
      ROS_DEBUG("#########################");

      ROS_DEBUG("motor_offsets:");
      ROS_DEBUG("  fl_abd_joint: %.3lf", max_pos[FL_ABD]);
      ROS_DEBUG("  fl_hip_joint: %.3lf", max_pos[FL_HIP]);
      ROS_DEBUG("  fl_knee_joint: %.3lf", max_pos[FL_KNEE]);

      ROS_DEBUG("  fr_abd_joint: %.3lf", max_pos[FR_ABD]);
      ROS_DEBUG("  fr_hip_joint: %.3lf", max_pos[FR_HIP]);
      ROS_DEBUG("  fr_knee_joint: %.3lf", max_pos[FR_KNEE]);

      ROS_DEBUG("  bl_abd_joint: %.3lf", max_pos[BL_ABD]);
      ROS_DEBUG("  bl_hip_joint: %.3lf", max_pos[BL_HIP]);
      ROS_DEBUG("  bl_knee_joint: %.3lf", max_pos[BL_KNEE]);

      ROS_DEBUG("  br_abd_joint: %.3lf", max_pos[BR_ABD]);
      ROS_DEBUG("  br_hip_joint: %.3lf", max_pos[BR_HIP]);
      ROS_DEBUG("  br_knee_joint: %.3lf", max_pos[BR_KNEE]);

      return true;

    }

  public:

    Stoch3JointOffsetCalibration(
        ros::NodeHandle* nh,
        ros::NodeHandle* pnh
        )
      : nh_(nh), pnh_(pnh)
  {
    joint_state_sub_ = nh->subscribe("/stoch3/joint_states", 1, &Stoch3JointOffsetCalibration::jointStateCB_, this);

    print_srv_ = pnh_->advertiseService("print_offset_calibration", &Stoch3JointOffsetCalibration::printSrvCB_, this);

    min_pos.resize(12);
    max_pos.resize(12);
    for(int i=0; i<12; i++)
    {
      min_pos[i] = 100000.f;
      max_pos[i] = -100000.f;
    }

    expected_min_pos.resize(12);
    expected_max_pos.resize(12);

    // Expected MIN POSITION 
    expected_min_pos[FL_ABD]  = MIN_FL_ABD;
    expected_min_pos[FL_HIP]  = MIN_FL_HIP;
    expected_min_pos[FL_KNEE] = MIN_FL_KNEE;

    expected_min_pos[FR_ABD]  = MIN_FR_ABD;
    expected_min_pos[FR_HIP]  = MIN_FR_HIP;
    expected_min_pos[FR_KNEE] = MIN_FR_KNEE;

    expected_min_pos[BL_ABD]  = MIN_BL_ABD;
    expected_min_pos[BL_HIP]  = MIN_BL_HIP;
    expected_min_pos[BL_KNEE] = MIN_BL_KNEE;

    expected_min_pos[BR_ABD]  = MIN_BR_ABD;
    expected_min_pos[BR_HIP]  = MIN_BR_HIP;
    expected_min_pos[BR_KNEE] = MIN_BR_KNEE;

    // Expected MAX POSITION 
    expected_max_pos[FL_ABD]  = MAX_FL_ABD;
    expected_max_pos[FL_HIP]  = MAX_FL_HIP;
    expected_max_pos[FL_KNEE] = MAX_FL_KNEE;

    expected_max_pos[FR_ABD]  = MAX_FR_ABD;
    expected_max_pos[FR_HIP]  = MAX_FR_HIP;
    expected_max_pos[FR_KNEE] = MAX_FR_KNEE;

    expected_max_pos[BL_ABD]  = MAX_BL_ABD;
    expected_max_pos[BL_HIP]  = MAX_BL_HIP;
    expected_max_pos[BL_KNEE] = MAX_BL_KNEE;

    expected_max_pos[BR_ABD]  = MAX_BR_ABD;
    expected_max_pos[BR_HIP]  = MAX_BR_HIP;
    expected_max_pos[BR_KNEE] = MAX_BR_KNEE;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stoch3_joint_offset_calibration");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  Stoch3JointOffsetCalibration joc(&nh, &pnh);

  ros::spin();

  return 0;
}
