/*
 * file: stoch3_visualizer.cpp
 *
 * Created: 27 Apr, 2022
 * Author : Aditya Sagi
 */

#include <ros/ros.h>

#include <visualization_msgs/MarkerArray.h>
#include <gazebo_msgs/ModelStates.h>
#include "stoch3_msgs/QuadrupedRobotState.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/JointState.h"

#include "urdf/model.h"

#include "stoch3_lib/libTrajectoryGenerator/utils/transformations.h"

// In the URDF model the robot center is on the bottom
// surface of the body. We consider the body center to
// lie in the plane of the axes of rotation of the joints.
// The offset of 0.054m is added to compensate for the difference.
#define URDF_OFFSET_Z 0.054

/*
 *
 * USAGE:
 *
 * To get a visual comparison of the actual robot state (as obtained from Gazebo)
 * and the estimated state obtained from the state estimator, run this node and
 * open the Marker plugin in Rviz to see the visualization.
 * To set-up Rviz follow the instructions:
 * 1) Launch Rviz:
 *    $ rviz
 * 2) Use the add button in the bottom left corner of the Rviz window to open the plugin
 *    Add -> Marker -> Ok
 * 3) Set the fixed_frame in RViz to "world".
 *
 * After Rviz is setup, launch this node to start publishing the marker information
 * to the topic.
 *
 */

class Stoch3Visualizer
{
  private:
    ros::Subscriber gazebo_sub_;
    ros::Subscriber robot_state_sub_;
    ros::Subscriber joint_state_sub_;
    ros::Publisher marker_pub_;

    urdf::Model model_;
    urdf::LinkConstSharedPtr base_link;

    sensor_msgs::JointState joint_state_;

    /*
     * Class to hold the colour information for
     * the markers.
     */
    class Colour{
      public:
        double r;
        double g;
        double b;
        double a;

        Colour(double r, double g, double b, double a) :
          r(r), g(g), b(b), a(a)
      {}
    };

    /*
     * Utility function to transform a vector
     * to the given orientation.
     */
    geometry_msgs::Vector3 reorientVector(
        geometry_msgs::Quaternion q_in,
        geometry_msgs::Vector3 v_in
        )
    {
      utils::Quat<double> q;
      utils::Matrix3d rot_matrix;
      utils::Vector3d vector;
      utils::Vector3d vector_out;
      geometry_msgs::Vector3 v_out;

      q.x() = q_in.x;
      q.y() = q_in.y;
      q.z() = q_in.z;
      q.w() = q_in.w;

      q.normalize();
      rot_matrix = q.toRotationMatrix();

      vector << v_in.x, v_in.y, v_in.z;

      vector_out = rot_matrix * vector;

      v_out.x = vector_out[0];
      v_out.y = vector_out[1];
      v_out.z = vector_out[2];

      return v_out;
    }



    /*
     * Utility function to transform a vector
     * to the given orientation.
     */
    utils::Vector3d reorientVector(
        utils::Quat<double> q_in,
        utils::Vector3d v_in)
    {
      utils::Matrix3d rot_mat;
      rot_mat = q_in.toRotationMatrix();
      return rot_mat * v_in;
    }

    /*
     * Utility function to convert from one data type
     * to another.
     */
    geometry_msgs::Vector3 convert(utils::Vector3d v)
    {
      geometry_msgs::Vector3 v_out;
      v_out.x = v[0];
      v_out.y = v[1];
      v_out.z = v[2];
      return v_out;
    }

    /*
     * Utility function to convert from one data type
     * to another.
     */
    utils::Vector3d convert(geometry_msgs::Vector3 v)
    {
      utils::Vector3d v_out;
      v_out << v.x, v.y, v.z;
      return v_out;
    }


    /*
     * Utility function to provide markers for
     * visualizing the foot position of the robot.
     * A sphere is used to visualize the position marker.
     *
     * \param[in] name_space : Name space of the marker.
     *
     * \param[in] id : Id number of the marker
     *
     * \param[in] position: Position of the marker.
     *
     * \param[in] colour : Colour of the marker
     *
     * \param[in] radius : Radius of the sphere
     *
     * \ret Returns a Marker message.
     *
     * Note: It is be default assumed that the
     * maker will be specified w.r.t a world frame.
     */
    visualization_msgs::Marker positionMarker_(
        std::string name_space,
        int id,
        geometry_msgs::Vector3 position,
        Colour colour,
        double radius
        )
    {
      visualization_msgs::Marker marker;

      marker.header.frame_id = "world";
      marker.header.stamp = ros::Time();
      marker.ns = name_space;
      marker.id = id;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::MODIFY;
      marker.pose.position.x = position.x;
      marker.pose.position.y = position.y;
      marker.pose.position.z = position.z;
      marker.pose.orientation.x = 0;
      marker.pose.orientation.y = 0;
      marker.pose.orientation.z = 0;
      marker.pose.orientation.w = 1;
      marker.scale.x = radius;
      marker.scale.y = radius;
      marker.scale.z = radius;
      marker.color.a = colour.a;
      marker.color.r = colour.r;
      marker.color.g = colour.g;
      marker.color.b = colour.b;

      return marker;
    }

    /*
     * Utility function to provide markers to
     * visualize a vector. An arrow is used
     * to visualize the vector.
     * An example use case of such a marker would
     * be to visualize the ground reaction forces
     * acting at the foot.
     *
     * \param[in] name_space : Name space of the marker
     *
     * \param[in] id : Id of the marker
     *
     * \param[in] position : Position of the root of the arrow.
     *
     * \param[in] vec : The vector which is to be visualized.
     *
     * \param[in] colour : Colour of the marker
     *
     * \param[in] scale : The scaling value to apply to the vector
     *                    for visualization.
     *
     *  \ret Returns a Marker message.
     *
     *  Note: It is assumed that the position of the root of the arrow
     *  will be specified w.r.t. a world frame (named "world").
     */
    visualization_msgs::Marker vectorMarker_(
        std::string name_space,
        int id,
        geometry_msgs::Vector3 position,
        geometry_msgs::Vector3 vec,
        Colour colour,
        double scale
        )
    {
      visualization_msgs::Marker marker;
      geometry_msgs::Point start_point, end_point;

      marker.header.frame_id = "world";
      marker.header.stamp = ros::Time();
      marker.ns = name_space;
      marker.id = id;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::MODIFY;
      start_point.x = position.x;
      start_point.y = position.y;
      start_point.z = position.z;
      end_point.x = start_point.x + scale*vec.x;
      end_point.y = start_point.y + scale*vec.y;
      end_point.z = start_point.z + scale*vec.z;
      marker.points.push_back(start_point);
      marker.points.push_back(end_point);
      marker.pose.orientation.x = 0;
      marker.pose.orientation.y = 0;
      marker.pose.orientation.z = 0;
      marker.pose.orientation.w = 1;
      marker.scale.x = 0.015;
      marker.scale.y = 0.018;
      marker.scale.z = 0.018;
      marker.color.a = colour.a;
      marker.color.r = colour.r;
      marker.color.g = colour.g;
      marker.color.b = colour.b;

      return marker;
    }

    /*
     * Utility function to determine the
     * joint between two links. It searches
     * through all the joints of the root_link
     * to find the one that joins the specified child
     * link.
     *
     * \param[in] root_link : Parent link of the joint
     *
     * \param[in] child_link : Child link of the joint
     *
     * \ret Returns the pointer to the joint that joins
     * the two specified links. Returns null if no such
     * joint exists.
     */
    urdf::JointConstSharedPtr findJoint(
        const urdf::LinkConstSharedPtr& root_link,
        const urdf::LinkConstSharedPtr& child_link
        )
    {
      int num_joints=0;
      num_joints = root_link->child_joints.size();
      for(auto i=0; i<num_joints; i++)
      {
        urdf::JointConstSharedPtr child_joint;
        child_joint = root_link->child_joints[i];
        if(child_link->name == child_joint->child_link_name)
          return child_joint;
      }

      return nullptr;
    }

    double findJointValue(
        std::string joint_name,
        sensor_msgs::JointState joint_state
        )
    {
      int num_joints;
      num_joints = joint_state.name.size();
      for(auto i=0; i<num_joints; i++)
      {
        if(joint_state.name[i] == joint_name)
          return joint_state.position[i];
      }
      ROS_DEBUG("Unable to find the joint state for the joint : %s", joint_name.c_str());
      return 0;
    }


    /*
     * Function to determine the centre of mass
     * of the link provided.
     *
     * \param[in] link: Pointer to the link
     *
     * \param[out] position: Position of the CoM in the
     *                       links frame of reference
     *
     * \param[out] mass: Mass of the link.
     */
    void getCoM(
        const urdf::LinkConstSharedPtr& link,
        utils::Vector3d& position,
        double& mass
        )
    {
      mass = link->inertial->mass;

      position[0] =  link->inertial->origin.position.x;
      position[1] =  link->inertial->origin.position.y;
      position[2] =  link->inertial->origin.position.z;
      return;
    }

    /*
     * Function to determine the centre of mass
     * of the tree provided.
     *
     * \param[in] root_link: The root link of the tree
     *
     * \param[in] joint_state: Joint state message containing
     *                         the current state of all the joints.
     *
     * \param[out] position: The position of the CoM in the root_links
     *                       frame of reference.
     *
     * \param[out] mass: The total mass of the tree.
     */
    void getCoMTree(
        const urdf::LinkConstSharedPtr& root_link,
        const sensor_msgs::JointState& joint_state,
        utils::Vector3d& position,
        double& mass
        )
    {
      double total_mass = 0;
      utils::Vector3d weighted_position;
      int num_child_links = 0;
      num_child_links = root_link->child_links.size();

      total_mass = root_link->inertial->mass;

      weighted_position << 0., 0., 0.;
      weighted_position[0] = total_mass * root_link->inertial->origin.position.x;
      weighted_position[1] = total_mass * root_link->inertial->origin.position.y;
      weighted_position[2] = total_mass * root_link->inertial->origin.position.z;

      for(auto i=0; i<num_child_links; i++)
      {
        urdf::LinkConstSharedPtr child_link;
        urdf::JointConstSharedPtr child_joint;
        utils::Vector3d com_position;
        utils::Vector3d child_joint_axis;
        utils::Vector3d child_joint_position;
        utils::Vector3d child_link_position;
        utils::Vector3d child_com_position;
        utils::Quat<double> child_joint_orientation;
        utils::Quat<double> child_link_orientation;
        double joint_value;
        double child_tree_mass;

        child_link = root_link->child_links[i];
        child_joint = findJoint(root_link, child_link);

        if(!child_joint)
          continue;

        joint_value = findJointValue(child_joint->name, joint_state);

        child_joint_axis[0] = child_joint->axis.x;
        child_joint_axis[1] = child_joint->axis.y;
        child_joint_axis[2] = child_joint->axis.z;

        child_joint_position[0] = child_joint->parent_to_joint_origin_transform.position.x;
        child_joint_position[1] = child_joint->parent_to_joint_origin_transform.position.y;
        child_joint_position[2] = child_joint->parent_to_joint_origin_transform.position.z;

        child_joint_orientation.x() = child_joint->parent_to_joint_origin_transform.rotation.x;
        child_joint_orientation.y() = child_joint->parent_to_joint_origin_transform.rotation.y;
        child_joint_orientation.z() = child_joint->parent_to_joint_origin_transform.rotation.z;
        child_joint_orientation.w() = child_joint->parent_to_joint_origin_transform.rotation.w;

        if(child_joint->type == urdf::Joint::REVOLUTE)
        {
          utils::Matrix3d child_joint_rotation_mat;
          utils::Matrix3d child_link_rotation_mat;
          utils::Matrix3d child_link_rotation_mat_j; // orientation w.r.t joint frame

          child_joint_rotation_mat = child_joint_orientation.toRotationMatrix();
          child_link_rotation_mat_j = Eigen::AngleAxisd(joint_value, child_joint_axis).toRotationMatrix();
          child_link_rotation_mat = child_joint_rotation_mat * child_link_rotation_mat_j;

          child_link_position = child_joint_position;
          child_link_orientation = utils::Quat<double>(child_link_rotation_mat);
        }
        else if(child_joint->type == urdf::Joint::PRISMATIC)
        {
          child_link_position = child_joint_position + joint_value * child_joint_axis;
          child_link_orientation = child_joint_orientation;
        }
        else
        {
          ROS_DEBUG("Joint '%s' is neither PRISMATIC nor REVOLUTE.", child_joint->name.c_str());
          continue;
        }

        getCoMTree(child_link, joint_state, com_position, child_tree_mass);
        child_com_position = reorientVector(child_link_orientation, com_position);
        child_com_position = child_link_position + child_com_position;

        total_mass = total_mass + child_tree_mass;
        weighted_position = weighted_position + child_tree_mass * child_com_position;
      }

      mass = total_mass;
      position = (1./total_mass) * weighted_position;

      return;
    }

    /*
     * Callback function to handle the messages received
     * on the topic "joint_states".
     */
    void jointStateCB_(const sensor_msgs::JointStateConstPtr& msg)
    {
      joint_state_ = *msg;
      return;
    }

    /*
     * Callback function to handle messages on the
     * topic "/gazebo/model_states".
     */
    void gazeboCB_(const gazebo_msgs::ModelStatesConstPtr& msg)
    {
      geometry_msgs::Pose pose;

      for(auto i=0; i < msg->name.size(); i++)
      {
        if(msg->name[i] == "stoch3")
          pose = msg->pose[i];
      }

      visualization_msgs::Marker marker;
      visualization_msgs::MarkerArray marker_array;

      marker.header.frame_id = "world";
      marker.header.stamp = ros::Time();
      marker.ns = "ground_truth/body/pose";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::CUBE;
      marker.action = visualization_msgs::Marker::MODIFY;
      marker.pose = pose;
      marker.scale.x = 0.5;
      marker.scale.y = 0.25;
      marker.scale.z = 0.1;
      marker.color.a = 0.7; // Don't forget to set the alpha!
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;

      marker_array.markers.push_back(marker);
      marker_pub_.publish(marker_array);
    }

    /*
     * Callback function to handle messages on the topic
     * /stoch3/robot_state
     */
    void robotStateCB_(const stoch3_msgs::QuadrupedRobotStateConstPtr& msg)
    {
      visualization_msgs::MarkerArray marker_array;
      stoch3_msgs::QuadrupedRobotState rstate;
      rstate = *msg;

      rstate.pose.position.z = rstate.pose.position.z + URDF_OFFSET_Z;
      rstate.fl.position.z = rstate.fl.position.z + URDF_OFFSET_Z;
      rstate.fr.position.z = rstate.fr.position.z + URDF_OFFSET_Z;
      rstate.bl.position.z = rstate.bl.position.z + URDF_OFFSET_Z;
      rstate.br.position.z = rstate.br.position.z + URDF_OFFSET_Z;

      // Body
      {
        visualization_msgs::Marker marker;

        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time();
        marker.ns = "estimate/body/pose";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::MODIFY;
        marker.pose = rstate.pose;
        marker.scale.x = 0.5;
        marker.scale.y = 0.25;
        marker.scale.z = 0.1;
        marker.color.a = 0.7; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;

        marker.pose.position.z = rstate.pose.position.z;

        marker_array.markers.push_back(marker);
      }

      // Foot positions
      {
        visualization_msgs::Marker marker;
        std::string position_ns("estimate/foot/position");
        Colour colour(0.5, 0.05, 0, 0.75);
        double radius=0.03;

        marker = positionMarker_(position_ns, 0, rstate.fl.position, colour, radius);
        marker_array.markers.push_back(marker);

        marker = positionMarker_(position_ns, 1, rstate.fr.position, colour, radius);
        marker_array.markers.push_back(marker);

        marker = positionMarker_(position_ns, 2, rstate.bl.position, colour, radius);
        marker_array.markers.push_back(marker);

        marker = positionMarker_(position_ns, 3, rstate.br.position, colour, radius);
        marker_array.markers.push_back(marker);
      }

      // Ground Reaction Forces
      {
        visualization_msgs::Marker marker;
        std::string force_ns("estimate/grf");
        Colour colour(0.5, 0.05, 0.25, 0.75);
        double scale=0.002;
        geometry_msgs::Vector3 grf;

        grf.x = -rstate.fl.force.x;
        grf.y = -rstate.fl.force.y;
        grf.z = -rstate.fl.force.z;
        marker = vectorMarker_(force_ns, 0, rstate.fl.position, grf, colour, scale);
        marker_array.markers.push_back(marker);

        grf.x = -rstate.fr.force.x;
        grf.y = -rstate.fr.force.y;
        grf.z = -rstate.fr.force.z;
        marker = vectorMarker_(force_ns, 1, rstate.fr.position, grf, colour, scale);
        marker_array.markers.push_back(marker);

        grf.x = -rstate.bl.force.x;
        grf.y = -rstate.bl.force.y;
        grf.z = -rstate.bl.force.z;
        marker = vectorMarker_(force_ns, 2, rstate.bl.position, grf, colour, scale);
        marker_array.markers.push_back(marker);

        grf.x = -rstate.br.force.x;
        grf.y = -rstate.br.force.y;
        grf.z = -rstate.br.force.z;
        marker = vectorMarker_(force_ns, 3, rstate.br.position, grf, colour, scale);
        marker_array.markers.push_back(marker);
      }


      // Torso CoM position
      if(base_link)
      {
        visualization_msgs::Marker marker;
        Colour colour(0., 0.5, 0.5, 0.75);
        Colour colour_projection(0.2, 0.5, 0.5, 0.75);
        double radius = 0.03;
        geometry_msgs::Vector3 com_position;
        geometry_msgs::Vector3 com_position_a; // CoM position of base in aligned frame
        utils::Vector3d base_com_position_b; // CoM position of base in body frame
        double mass;

        // Note: Aligned frame is located at the centre of the body, but it is
        // aligned to the world frame.

        getCoM(base_link, base_com_position_b, mass);

        com_position = convert(base_com_position_b);

        com_position_a = reorientVector(rstate.pose.orientation, com_position);

        com_position_a.x = com_position_a.x + rstate.pose.position.x;
        com_position_a.y = com_position_a.y + rstate.pose.position.y;
        com_position_a.z = com_position_a.z + rstate.pose.position.z;
        marker = positionMarker_("estimate/body/com", 0, com_position_a, colour, radius);
        marker_array.markers.push_back(marker);

        com_position_a.z = 0; // Project onto ground plane
        marker = positionMarker_("estimate/body/com_projection", 0, com_position_a, colour_projection, radius);
        marker_array.markers.push_back(marker);
      }

      // Robot CoM position
      if(base_link)
      {
        visualization_msgs::Marker marker;
        Colour colour(0., 0.5, 0.5, 0.75);
        Colour colour_projection(0.2, 0.5, 0.5, 0.75);
        double radius = 0.03;
        geometry_msgs::Vector3 com_position;
        geometry_msgs::Vector3 com_position_a; // CoM position of robot in aligned frame.
        utils::Vector3d com_position_b; // CoM position of robot in body frame
        double mass;

        // Note: Aligned frame is located at the centre of the body, but it is
        // aligned to the world frame.

        getCoMTree(base_link, joint_state_, com_position_b, mass);

        com_position = convert(com_position_b);

        com_position_a = reorientVector(rstate.pose.orientation, com_position);

        com_position_a.x = com_position_a.x + rstate.pose.position.x;
        com_position_a.y = com_position_a.y + rstate.pose.position.y;
        com_position_a.z = com_position_a.z + rstate.pose.position.z;
        marker = positionMarker_("estimate/robot/com", 0, com_position_a, colour, radius);
        marker_array.markers.push_back(marker);

        com_position_a.z = 0; // Project onto ground plane
        marker = positionMarker_("estimate/robot/com_projection", 0, com_position_a, colour_projection, radius);
        marker_array.markers.push_back(marker);
      }

      // Support polygon
      {
        visualization_msgs::Marker marker;
        geometry_msgs::Point point;
        std::string name_space("estimate/robot/support_polygon");
        int id=0;
        Colour colour(0.5, 0., 0.1, 0.75);

        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time();
        marker.ns = name_space;
        marker.id = id;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::MODIFY;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1;
        marker.scale.x = 0.015;
        marker.scale.y = 0.018;
        marker.scale.z = 0.018;
        marker.color.a = colour.a;
        marker.color.r = colour.r;
        marker.color.g = colour.g;
        marker.color.b = colour.b;

        if(rstate.fl.support_probability > 0.75)
        {
          point.x = rstate.fl.position.x;
          point.y = rstate.fl.position.y;
          point.z = rstate.fl.position.z;
          marker.points.push_back(point);
        }
        if(rstate.fr.support_probability > 0.75)
        {
          point.x = rstate.fr.position.x;
          point.y = rstate.fr.position.y;
          point.z = rstate.fr.position.z;
          marker.points.push_back(point);
        }
        if(rstate.br.support_probability > 0.75)
        { // Note: BR should be specied before
          // BL to maintain the cyclic order.
          point.x = rstate.br.position.x;
          point.y = rstate.br.position.y;
          point.z = rstate.br.position.z;
          marker.points.push_back(point);
        }
        if(rstate.bl.support_probability > 0.75)
        {
          point.x = rstate.bl.position.x;
          point.y = rstate.bl.position.y;
          point.z = rstate.bl.position.z;
          marker.points.push_back(point);
        }

        if(marker.points.size() == 1)
        {
          // To avoid error prints in RViz. It expects more than one point
          // for marker of type LINE_STRIP.
          marker.type = visualization_msgs::Marker::POINTS;
        }
        else if(marker.points.size() > 2)
        {
          // Add the first point again to close the loop
          marker.points.push_back(marker.points[0]);
        }

        // Publish marker if one or more points are provided
        if(marker.points.size() > 0)
          marker_array.markers.push_back(marker);
      }

      marker_pub_.publish(marker_array);
    }

  public:

    Stoch3Visualizer(ros::NodeHandle* nh)
    {
      std::string robot_description;
      if(!nh->getParam("/robot_description", robot_description))
      {
        ROS_WARN("Unable to obtain robot description from the parameter server.");
      }
      else
      {
        if(!model_.initString(robot_description))
        {
          ROS_WARN("Unable to parse robot description string.");
          base_link = nullptr;
        }
        else
        {
          urdf::LinkConstSharedPtr world_link;
          world_link = model_.getRoot();
          base_link = world_link->child_links[0];
        }
      }

      gazebo_sub_ = nh->subscribe("/gazebo/model_states", 1, &Stoch3Visualizer::gazeboCB_, this);
      robot_state_sub_ = nh->subscribe("robot_state", 1, &Stoch3Visualizer::robotStateCB_, this);
      joint_state_sub_ = nh->subscribe("joint_states", 1, &Stoch3Visualizer::jointStateCB_, this);

      marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 1);
    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stoch3_visualizer_node");

  ros::NodeHandle nh;

  Stoch3Visualizer stoch3_viz(&nh);

  ros::spin();
  return 0;
}
