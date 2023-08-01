// Generated by gencpp from file fovis_msgs/Stats.msg
// DO NOT EDIT!


#ifndef FOVIS_MSGS_MESSAGE_STATS_H
#define FOVIS_MSGS_MESSAGE_STATS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace fovis_msgs
{
template <class ContainerAllocator>
struct Stats_
{
  typedef Stats_<ContainerAllocator> Type;

  Stats_()
    : header()
    , num_matches(0)
    , num_inliers(0)
    , num_detected_keypoints(0)
    , num_reprojection_failures(0)
    , num_keypoints(0)
    , mean_reprojection_error(0.0)
    , fast_threshold(0.0)
    , status(0)  {
    }
  Stats_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , num_matches(0)
    , num_inliers(0)
    , num_detected_keypoints(0)
    , num_reprojection_failures(0)
    , num_keypoints(0)
    , mean_reprojection_error(0.0)
    , fast_threshold(0.0)
    , status(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef int32_t _num_matches_type;
  _num_matches_type num_matches;

   typedef int32_t _num_inliers_type;
  _num_inliers_type num_inliers;

   typedef int32_t _num_detected_keypoints_type;
  _num_detected_keypoints_type num_detected_keypoints;

   typedef int32_t _num_reprojection_failures_type;
  _num_reprojection_failures_type num_reprojection_failures;

   typedef int32_t _num_keypoints_type;
  _num_keypoints_type num_keypoints;

   typedef double _mean_reprojection_error_type;
  _mean_reprojection_error_type mean_reprojection_error;

   typedef double _fast_threshold_type;
  _fast_threshold_type fast_threshold;

   typedef uint8_t _status_type;
  _status_type status;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(STATUS_CODE_NO_DATA)
  #undef STATUS_CODE_NO_DATA
#endif
#if defined(_WIN32) && defined(STATUS_CODE_SUCCESS)
  #undef STATUS_CODE_SUCCESS
#endif
#if defined(_WIN32) && defined(STATUS_CODE_INSUFFICIENT_INLIERS)
  #undef STATUS_CODE_INSUFFICIENT_INLIERS
#endif
#if defined(_WIN32) && defined(STATUS_CODE_OPTIMIZATION_FAILURE)
  #undef STATUS_CODE_OPTIMIZATION_FAILURE
#endif
#if defined(_WIN32) && defined(STATUS_CODE_REPROJECTION_ERROR)
  #undef STATUS_CODE_REPROJECTION_ERROR
#endif

  enum {
    STATUS_CODE_NO_DATA = 0u,
    STATUS_CODE_SUCCESS = 1u,
    STATUS_CODE_INSUFFICIENT_INLIERS = 2u,
    STATUS_CODE_OPTIMIZATION_FAILURE = 3u,
    STATUS_CODE_REPROJECTION_ERROR = 4u,
  };


  typedef boost::shared_ptr< ::fovis_msgs::Stats_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::fovis_msgs::Stats_<ContainerAllocator> const> ConstPtr;

}; // struct Stats_

typedef ::fovis_msgs::Stats_<std::allocator<void> > Stats;

typedef boost::shared_ptr< ::fovis_msgs::Stats > StatsPtr;
typedef boost::shared_ptr< ::fovis_msgs::Stats const> StatsConstPtr;

// constants requiring out of line definition

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::fovis_msgs::Stats_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::fovis_msgs::Stats_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::fovis_msgs::Stats_<ContainerAllocator1> & lhs, const ::fovis_msgs::Stats_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.num_matches == rhs.num_matches &&
    lhs.num_inliers == rhs.num_inliers &&
    lhs.num_detected_keypoints == rhs.num_detected_keypoints &&
    lhs.num_reprojection_failures == rhs.num_reprojection_failures &&
    lhs.num_keypoints == rhs.num_keypoints &&
    lhs.mean_reprojection_error == rhs.mean_reprojection_error &&
    lhs.fast_threshold == rhs.fast_threshold &&
    lhs.status == rhs.status;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::fovis_msgs::Stats_<ContainerAllocator1> & lhs, const ::fovis_msgs::Stats_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace fovis_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::fovis_msgs::Stats_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::fovis_msgs::Stats_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::fovis_msgs::Stats_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::fovis_msgs::Stats_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::fovis_msgs::Stats_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::fovis_msgs::Stats_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::fovis_msgs::Stats_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dbaf83c8b3afd2954f468abc4e36c223";
  }

  static const char* value(const ::fovis_msgs::Stats_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xdbaf83c8b3afd295ULL;
  static const uint64_t static_value2 = 0x4f468abc4e36c223ULL;
};

template<class ContainerAllocator>
struct DataType< ::fovis_msgs::Stats_<ContainerAllocator> >
{
  static const char* value()
  {
    return "fovis_msgs/Stats";
  }

  static const char* value(const ::fovis_msgs::Stats_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::fovis_msgs::Stats_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# state of fovis\n"
"# same as fovis::stats_t lcm message\n"
"Header header\n"
"\n"
"int32 num_matches\n"
"int32 num_inliers\n"
"int32 num_detected_keypoints\n"
"int32 num_reprojection_failures\n"
"\n"
"# keypoints after pruning\n"
"int32 num_keypoints\n"
"\n"
"float64 mean_reprojection_error\n"
"float64 fast_threshold\n"
"\n"
"uint8 STATUS_CODE_NO_DATA = 0\n"
"uint8 STATUS_CODE_SUCCESS = 1\n"
"uint8 STATUS_CODE_INSUFFICIENT_INLIERS = 2\n"
"uint8 STATUS_CODE_OPTIMIZATION_FAILURE = 3\n"
"uint8 STATUS_CODE_REPROJECTION_ERROR = 4\n"
"\n"
"uint8 status\n"
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

  static const char* value(const ::fovis_msgs::Stats_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::fovis_msgs::Stats_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.num_matches);
      stream.next(m.num_inliers);
      stream.next(m.num_detected_keypoints);
      stream.next(m.num_reprojection_failures);
      stream.next(m.num_keypoints);
      stream.next(m.mean_reprojection_error);
      stream.next(m.fast_threshold);
      stream.next(m.status);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Stats_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::fovis_msgs::Stats_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::fovis_msgs::Stats_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "num_matches: ";
    Printer<int32_t>::stream(s, indent + "  ", v.num_matches);
    s << indent << "num_inliers: ";
    Printer<int32_t>::stream(s, indent + "  ", v.num_inliers);
    s << indent << "num_detected_keypoints: ";
    Printer<int32_t>::stream(s, indent + "  ", v.num_detected_keypoints);
    s << indent << "num_reprojection_failures: ";
    Printer<int32_t>::stream(s, indent + "  ", v.num_reprojection_failures);
    s << indent << "num_keypoints: ";
    Printer<int32_t>::stream(s, indent + "  ", v.num_keypoints);
    s << indent << "mean_reprojection_error: ";
    Printer<double>::stream(s, indent + "  ", v.mean_reprojection_error);
    s << indent << "fast_threshold: ";
    Printer<double>::stream(s, indent + "  ", v.fast_threshold);
    s << indent << "status: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.status);
  }
};

} // namespace message_operations
} // namespace ros

#endif // FOVIS_MSGS_MESSAGE_STATS_H
