// Generated by gencpp from file stoch3_msgs/GaitRequest.msg
// DO NOT EDIT!


#ifndef STOCH3_MSGS_MESSAGE_GAITREQUEST_H
#define STOCH3_MSGS_MESSAGE_GAITREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Twist.h>

namespace stoch3_msgs
{
template <class ContainerAllocator>
struct GaitRequest_
{
  typedef GaitRequest_<ContainerAllocator> Type;

  GaitRequest_()
    : step_frequency(0.0)
    , swing_height(0.0)
    , stance_height(0.0)
    , max_vel()  {
    }
  GaitRequest_(const ContainerAllocator& _alloc)
    : step_frequency(0.0)
    , swing_height(0.0)
    , stance_height(0.0)
    , max_vel(_alloc)  {
  (void)_alloc;
    }



   typedef double _step_frequency_type;
  _step_frequency_type step_frequency;

   typedef double _swing_height_type;
  _swing_height_type swing_height;

   typedef double _stance_height_type;
  _stance_height_type stance_height;

   typedef  ::geometry_msgs::Twist_<ContainerAllocator>  _max_vel_type;
  _max_vel_type max_vel;





  typedef boost::shared_ptr< ::stoch3_msgs::GaitRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::stoch3_msgs::GaitRequest_<ContainerAllocator> const> ConstPtr;

}; // struct GaitRequest_

typedef ::stoch3_msgs::GaitRequest_<std::allocator<void> > GaitRequest;

typedef boost::shared_ptr< ::stoch3_msgs::GaitRequest > GaitRequestPtr;
typedef boost::shared_ptr< ::stoch3_msgs::GaitRequest const> GaitRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::stoch3_msgs::GaitRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::stoch3_msgs::GaitRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::stoch3_msgs::GaitRequest_<ContainerAllocator1> & lhs, const ::stoch3_msgs::GaitRequest_<ContainerAllocator2> & rhs)
{
  return lhs.step_frequency == rhs.step_frequency &&
    lhs.swing_height == rhs.swing_height &&
    lhs.stance_height == rhs.stance_height &&
    lhs.max_vel == rhs.max_vel;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::stoch3_msgs::GaitRequest_<ContainerAllocator1> & lhs, const ::stoch3_msgs::GaitRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace stoch3_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::stoch3_msgs::GaitRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::stoch3_msgs::GaitRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::stoch3_msgs::GaitRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::stoch3_msgs::GaitRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::stoch3_msgs::GaitRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::stoch3_msgs::GaitRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::stoch3_msgs::GaitRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ed56ce313d28e93dd09cffbe6ef6c08d";
  }

  static const char* value(const ::stoch3_msgs::GaitRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xed56ce313d28e93dULL;
  static const uint64_t static_value2 = 0xd09cffbe6ef6c08dULL;
};

template<class ContainerAllocator>
struct DataType< ::stoch3_msgs::GaitRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "stoch3_msgs/GaitRequest";
  }

  static const char* value(const ::stoch3_msgs::GaitRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::stoch3_msgs::GaitRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 step_frequency\n"
"float64 swing_height\n"
"float64 stance_height\n"
"geometry_msgs/Twist max_vel\n"
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
;
  }

  static const char* value(const ::stoch3_msgs::GaitRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::stoch3_msgs::GaitRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.step_frequency);
      stream.next(m.swing_height);
      stream.next(m.stance_height);
      stream.next(m.max_vel);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GaitRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::stoch3_msgs::GaitRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::stoch3_msgs::GaitRequest_<ContainerAllocator>& v)
  {
    s << indent << "step_frequency: ";
    Printer<double>::stream(s, indent + "  ", v.step_frequency);
    s << indent << "swing_height: ";
    Printer<double>::stream(s, indent + "  ", v.swing_height);
    s << indent << "stance_height: ";
    Printer<double>::stream(s, indent + "  ", v.stance_height);
    s << indent << "max_vel: ";
    s << std::endl;
    Printer< ::geometry_msgs::Twist_<ContainerAllocator> >::stream(s, indent + "  ", v.max_vel);
  }
};

} // namespace message_operations
} // namespace ros

#endif // STOCH3_MSGS_MESSAGE_GAITREQUEST_H