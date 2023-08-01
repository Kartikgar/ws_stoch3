// Generated by gencpp from file pronto_msgs/QuadrupedStance.msg
// DO NOT EDIT!


#ifndef PRONTO_MSGS_MESSAGE_QUADRUPEDSTANCE_H
#define PRONTO_MSGS_MESSAGE_QUADRUPEDSTANCE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace pronto_msgs
{
template <class ContainerAllocator>
struct QuadrupedStance_
{
  typedef QuadrupedStance_<ContainerAllocator> Type;

  QuadrupedStance_()
    : header()
    , lf(0.0)
    , rf(0.0)
    , lh(0.0)
    , rh(0.0)  {
    }
  QuadrupedStance_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , lf(0.0)
    , rf(0.0)
    , lh(0.0)
    , rh(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef float _lf_type;
  _lf_type lf;

   typedef float _rf_type;
  _rf_type rf;

   typedef float _lh_type;
  _lh_type lh;

   typedef float _rh_type;
  _rh_type rh;





  typedef boost::shared_ptr< ::pronto_msgs::QuadrupedStance_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pronto_msgs::QuadrupedStance_<ContainerAllocator> const> ConstPtr;

}; // struct QuadrupedStance_

typedef ::pronto_msgs::QuadrupedStance_<std::allocator<void> > QuadrupedStance;

typedef boost::shared_ptr< ::pronto_msgs::QuadrupedStance > QuadrupedStancePtr;
typedef boost::shared_ptr< ::pronto_msgs::QuadrupedStance const> QuadrupedStanceConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pronto_msgs::QuadrupedStance_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pronto_msgs::QuadrupedStance_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::pronto_msgs::QuadrupedStance_<ContainerAllocator1> & lhs, const ::pronto_msgs::QuadrupedStance_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.lf == rhs.lf &&
    lhs.rf == rhs.rf &&
    lhs.lh == rhs.lh &&
    lhs.rh == rhs.rh;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::pronto_msgs::QuadrupedStance_<ContainerAllocator1> & lhs, const ::pronto_msgs::QuadrupedStance_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace pronto_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::pronto_msgs::QuadrupedStance_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pronto_msgs::QuadrupedStance_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pronto_msgs::QuadrupedStance_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pronto_msgs::QuadrupedStance_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pronto_msgs::QuadrupedStance_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pronto_msgs::QuadrupedStance_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pronto_msgs::QuadrupedStance_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6e165cd35d6f3eecca48a2a9f715fd5f";
  }

  static const char* value(const ::pronto_msgs::QuadrupedStance_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6e165cd35d6f3eecULL;
  static const uint64_t static_value2 = 0xca48a2a9f715fd5fULL;
};

template<class ContainerAllocator>
struct DataType< ::pronto_msgs::QuadrupedStance_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pronto_msgs/QuadrupedStance";
  }

  static const char* value(const ::pronto_msgs::QuadrupedStance_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pronto_msgs::QuadrupedStance_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"float32 lf\n"
"float32 rf\n"
"float32 lh\n"
"float32 rh\n"
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

  static const char* value(const ::pronto_msgs::QuadrupedStance_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pronto_msgs::QuadrupedStance_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.lf);
      stream.next(m.rf);
      stream.next(m.lh);
      stream.next(m.rh);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct QuadrupedStance_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pronto_msgs::QuadrupedStance_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pronto_msgs::QuadrupedStance_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "lf: ";
    Printer<float>::stream(s, indent + "  ", v.lf);
    s << indent << "rf: ";
    Printer<float>::stream(s, indent + "  ", v.rf);
    s << indent << "lh: ";
    Printer<float>::stream(s, indent + "  ", v.lh);
    s << indent << "rh: ";
    Printer<float>::stream(s, indent + "  ", v.rh);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PRONTO_MSGS_MESSAGE_QUADRUPEDSTANCE_H
