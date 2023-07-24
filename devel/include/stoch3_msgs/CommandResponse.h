// Generated by gencpp from file stoch3_msgs/CommandResponse.msg
// DO NOT EDIT!


#ifndef STOCH3_MSGS_MESSAGE_COMMANDRESPONSE_H
#define STOCH3_MSGS_MESSAGE_COMMANDRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace stoch3_msgs
{
template <class ContainerAllocator>
struct CommandResponse_
{
  typedef CommandResponse_<ContainerAllocator> Type;

  CommandResponse_()
    : ok(false)  {
    }
  CommandResponse_(const ContainerAllocator& _alloc)
    : ok(false)  {
  (void)_alloc;
    }



   typedef uint8_t _ok_type;
  _ok_type ok;





  typedef boost::shared_ptr< ::stoch3_msgs::CommandResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::stoch3_msgs::CommandResponse_<ContainerAllocator> const> ConstPtr;

}; // struct CommandResponse_

typedef ::stoch3_msgs::CommandResponse_<std::allocator<void> > CommandResponse;

typedef boost::shared_ptr< ::stoch3_msgs::CommandResponse > CommandResponsePtr;
typedef boost::shared_ptr< ::stoch3_msgs::CommandResponse const> CommandResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::stoch3_msgs::CommandResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::stoch3_msgs::CommandResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::stoch3_msgs::CommandResponse_<ContainerAllocator1> & lhs, const ::stoch3_msgs::CommandResponse_<ContainerAllocator2> & rhs)
{
  return lhs.ok == rhs.ok;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::stoch3_msgs::CommandResponse_<ContainerAllocator1> & lhs, const ::stoch3_msgs::CommandResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace stoch3_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::stoch3_msgs::CommandResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::stoch3_msgs::CommandResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::stoch3_msgs::CommandResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::stoch3_msgs::CommandResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::stoch3_msgs::CommandResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::stoch3_msgs::CommandResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::stoch3_msgs::CommandResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6f6da3883749771fac40d6deb24a8c02";
  }

  static const char* value(const ::stoch3_msgs::CommandResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6f6da3883749771fULL;
  static const uint64_t static_value2 = 0xac40d6deb24a8c02ULL;
};

template<class ContainerAllocator>
struct DataType< ::stoch3_msgs::CommandResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "stoch3_msgs/CommandResponse";
  }

  static const char* value(const ::stoch3_msgs::CommandResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::stoch3_msgs::CommandResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool ok\n"
"\n"
;
  }

  static const char* value(const ::stoch3_msgs::CommandResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::stoch3_msgs::CommandResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.ok);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CommandResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::stoch3_msgs::CommandResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::stoch3_msgs::CommandResponse_<ContainerAllocator>& v)
  {
    s << indent << "ok: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.ok);
  }
};

} // namespace message_operations
} // namespace ros

#endif // STOCH3_MSGS_MESSAGE_COMMANDRESPONSE_H
