// Generated by gencpp from file stoch3_msgs/QuadrupedLegFeedback.msg
// DO NOT EDIT!


#ifndef STOCH3_MSGS_MESSAGE_QUADRUPEDLEGFEEDBACK_H
#define STOCH3_MSGS_MESSAGE_QUADRUPEDLEGFEEDBACK_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace stoch3_msgs
{
template <class ContainerAllocator>
struct QuadrupedLegFeedback_
{
  typedef QuadrupedLegFeedback_<ContainerAllocator> Type;

  QuadrupedLegFeedback_()
    : header()
    , name()
    , foot_position()
    , foot_velocity()
    , foot_force()
    , foot_pos_error()
    , error_status(false)  {
    }
  QuadrupedLegFeedback_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , name(_alloc)
    , foot_position(_alloc)
    , foot_velocity(_alloc)
    , foot_force(_alloc)
    , foot_pos_error(_alloc)
    , error_status(false)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _name_type;
  _name_type name;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _foot_position_type;
  _foot_position_type foot_position;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _foot_velocity_type;
  _foot_velocity_type foot_velocity;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _foot_force_type;
  _foot_force_type foot_force;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _foot_pos_error_type;
  _foot_pos_error_type foot_pos_error;

   typedef uint8_t _error_status_type;
  _error_status_type error_status;





  typedef boost::shared_ptr< ::stoch3_msgs::QuadrupedLegFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::stoch3_msgs::QuadrupedLegFeedback_<ContainerAllocator> const> ConstPtr;

}; // struct QuadrupedLegFeedback_

typedef ::stoch3_msgs::QuadrupedLegFeedback_<std::allocator<void> > QuadrupedLegFeedback;

typedef boost::shared_ptr< ::stoch3_msgs::QuadrupedLegFeedback > QuadrupedLegFeedbackPtr;
typedef boost::shared_ptr< ::stoch3_msgs::QuadrupedLegFeedback const> QuadrupedLegFeedbackConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::stoch3_msgs::QuadrupedLegFeedback_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::stoch3_msgs::QuadrupedLegFeedback_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::stoch3_msgs::QuadrupedLegFeedback_<ContainerAllocator1> & lhs, const ::stoch3_msgs::QuadrupedLegFeedback_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.name == rhs.name &&
    lhs.foot_position == rhs.foot_position &&
    lhs.foot_velocity == rhs.foot_velocity &&
    lhs.foot_force == rhs.foot_force &&
    lhs.foot_pos_error == rhs.foot_pos_error &&
    lhs.error_status == rhs.error_status;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::stoch3_msgs::QuadrupedLegFeedback_<ContainerAllocator1> & lhs, const ::stoch3_msgs::QuadrupedLegFeedback_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace stoch3_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::stoch3_msgs::QuadrupedLegFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::stoch3_msgs::QuadrupedLegFeedback_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::stoch3_msgs::QuadrupedLegFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::stoch3_msgs::QuadrupedLegFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::stoch3_msgs::QuadrupedLegFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::stoch3_msgs::QuadrupedLegFeedback_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::stoch3_msgs::QuadrupedLegFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7475d2278216c40ff50e977798003eb3";
  }

  static const char* value(const ::stoch3_msgs::QuadrupedLegFeedback_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7475d2278216c40fULL;
  static const uint64_t static_value2 = 0xf50e977798003eb3ULL;
};

template<class ContainerAllocator>
struct DataType< ::stoch3_msgs::QuadrupedLegFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "stoch3_msgs/QuadrupedLegFeedback";
  }

  static const char* value(const ::stoch3_msgs::QuadrupedLegFeedback_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::stoch3_msgs::QuadrupedLegFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header    header\n"
"string[]  name\n"
"float64[] foot_position\n"
"float64[] foot_velocity\n"
"float64[] foot_force\n"
"float64[] foot_pos_error\n"
"bool error_status\n"
"\n"
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
;
  }

  static const char* value(const ::stoch3_msgs::QuadrupedLegFeedback_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::stoch3_msgs::QuadrupedLegFeedback_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.name);
      stream.next(m.foot_position);
      stream.next(m.foot_velocity);
      stream.next(m.foot_force);
      stream.next(m.foot_pos_error);
      stream.next(m.error_status);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct QuadrupedLegFeedback_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::stoch3_msgs::QuadrupedLegFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::stoch3_msgs::QuadrupedLegFeedback_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "name[]" << std::endl;
    for (size_t i = 0; i < v.name.size(); ++i)
    {
      s << indent << "  name[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.name[i]);
    }
    s << indent << "foot_position[]" << std::endl;
    for (size_t i = 0; i < v.foot_position.size(); ++i)
    {
      s << indent << "  foot_position[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.foot_position[i]);
    }
    s << indent << "foot_velocity[]" << std::endl;
    for (size_t i = 0; i < v.foot_velocity.size(); ++i)
    {
      s << indent << "  foot_velocity[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.foot_velocity[i]);
    }
    s << indent << "foot_force[]" << std::endl;
    for (size_t i = 0; i < v.foot_force.size(); ++i)
    {
      s << indent << "  foot_force[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.foot_force[i]);
    }
    s << indent << "foot_pos_error[]" << std::endl;
    for (size_t i = 0; i < v.foot_pos_error.size(); ++i)
    {
      s << indent << "  foot_pos_error[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.foot_pos_error[i]);
    }
    s << indent << "error_status: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.error_status);
  }
};

} // namespace message_operations
} // namespace ros

#endif // STOCH3_MSGS_MESSAGE_QUADRUPEDLEGFEEDBACK_H
