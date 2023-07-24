// Generated by gencpp from file stoch3_msgs/RobotCommand.msg
// DO NOT EDIT!


#ifndef STOCH3_MSGS_MESSAGE_ROBOTCOMMAND_H
#define STOCH3_MSGS_MESSAGE_ROBOTCOMMAND_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

namespace stoch3_msgs
{
template <class ContainerAllocator>
struct RobotCommand_
{
  typedef RobotCommand_<ContainerAllocator> Type;

  RobotCommand_()
    : header()
    , twist()
    , pose()  {
    }
  RobotCommand_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , twist(_alloc)
    , pose(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::geometry_msgs::Twist_<ContainerAllocator>  _twist_type;
  _twist_type twist;

   typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _pose_type;
  _pose_type pose;





  typedef boost::shared_ptr< ::stoch3_msgs::RobotCommand_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::stoch3_msgs::RobotCommand_<ContainerAllocator> const> ConstPtr;

}; // struct RobotCommand_

typedef ::stoch3_msgs::RobotCommand_<std::allocator<void> > RobotCommand;

typedef boost::shared_ptr< ::stoch3_msgs::RobotCommand > RobotCommandPtr;
typedef boost::shared_ptr< ::stoch3_msgs::RobotCommand const> RobotCommandConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::stoch3_msgs::RobotCommand_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::stoch3_msgs::RobotCommand_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::stoch3_msgs::RobotCommand_<ContainerAllocator1> & lhs, const ::stoch3_msgs::RobotCommand_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.twist == rhs.twist &&
    lhs.pose == rhs.pose;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::stoch3_msgs::RobotCommand_<ContainerAllocator1> & lhs, const ::stoch3_msgs::RobotCommand_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace stoch3_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::stoch3_msgs::RobotCommand_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::stoch3_msgs::RobotCommand_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::stoch3_msgs::RobotCommand_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::stoch3_msgs::RobotCommand_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::stoch3_msgs::RobotCommand_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::stoch3_msgs::RobotCommand_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::stoch3_msgs::RobotCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "57051507a48caf37604692cb50cc08fe";
  }

  static const char* value(const ::stoch3_msgs::RobotCommand_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x57051507a48caf37ULL;
  static const uint64_t static_value2 = 0x604692cb50cc08feULL;
};

template<class ContainerAllocator>
struct DataType< ::stoch3_msgs::RobotCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "stoch3_msgs/RobotCommand";
  }

  static const char* value(const ::stoch3_msgs::RobotCommand_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::stoch3_msgs::RobotCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header              header\n"
"geometry_msgs/Twist twist\n"
"geometry_msgs/Pose  pose\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Twist\n"
"# This expresses velocity in free space broken into its linear and angular parts.\n"
"Vector3  linear\n"
"Vector3  angular\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Vector3\n"
"# This represents a vector in free space. \n"
"# It is only meant to represent a direction. Therefore, it does not\n"
"# make sense to apply a translation to it (e.g., when applying a \n"
"# generic rigid transformation to a Vector3, tf2 will only apply the\n"
"# rotation). If you want your data to be translatable too, use the\n"
"# geometry_msgs/Point message instead.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose\n"
"# A representation of pose in free space, composed of position and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
;
  }

  static const char* value(const ::stoch3_msgs::RobotCommand_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::stoch3_msgs::RobotCommand_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.twist);
      stream.next(m.pose);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RobotCommand_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::stoch3_msgs::RobotCommand_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::stoch3_msgs::RobotCommand_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "twist: ";
    s << std::endl;
    Printer< ::geometry_msgs::Twist_<ContainerAllocator> >::stream(s, indent + "  ", v.twist);
    s << indent << "pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
  }
};

} // namespace message_operations
} // namespace ros

#endif // STOCH3_MSGS_MESSAGE_ROBOTCOMMAND_H
