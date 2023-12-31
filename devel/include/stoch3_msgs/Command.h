// Generated by gencpp from file stoch3_msgs/Command.msg
// DO NOT EDIT!


#ifndef STOCH3_MSGS_MESSAGE_COMMAND_H
#define STOCH3_MSGS_MESSAGE_COMMAND_H

#include <ros/service_traits.h>


#include <stoch3_msgs/CommandRequest.h>
#include <stoch3_msgs/CommandResponse.h>


namespace stoch3_msgs
{

struct Command
{

typedef CommandRequest Request;
typedef CommandResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct Command
} // namespace stoch3_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::stoch3_msgs::Command > {
  static const char* value()
  {
    return "8d2fe1ada9945234c69964dd9f502541";
  }

  static const char* value(const ::stoch3_msgs::Command&) { return value(); }
};

template<>
struct DataType< ::stoch3_msgs::Command > {
  static const char* value()
  {
    return "stoch3_msgs/Command";
  }

  static const char* value(const ::stoch3_msgs::Command&) { return value(); }
};


// service_traits::MD5Sum< ::stoch3_msgs::CommandRequest> should match
// service_traits::MD5Sum< ::stoch3_msgs::Command >
template<>
struct MD5Sum< ::stoch3_msgs::CommandRequest>
{
  static const char* value()
  {
    return MD5Sum< ::stoch3_msgs::Command >::value();
  }
  static const char* value(const ::stoch3_msgs::CommandRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::stoch3_msgs::CommandRequest> should match
// service_traits::DataType< ::stoch3_msgs::Command >
template<>
struct DataType< ::stoch3_msgs::CommandRequest>
{
  static const char* value()
  {
    return DataType< ::stoch3_msgs::Command >::value();
  }
  static const char* value(const ::stoch3_msgs::CommandRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::stoch3_msgs::CommandResponse> should match
// service_traits::MD5Sum< ::stoch3_msgs::Command >
template<>
struct MD5Sum< ::stoch3_msgs::CommandResponse>
{
  static const char* value()
  {
    return MD5Sum< ::stoch3_msgs::Command >::value();
  }
  static const char* value(const ::stoch3_msgs::CommandResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::stoch3_msgs::CommandResponse> should match
// service_traits::DataType< ::stoch3_msgs::Command >
template<>
struct DataType< ::stoch3_msgs::CommandResponse>
{
  static const char* value()
  {
    return DataType< ::stoch3_msgs::Command >::value();
  }
  static const char* value(const ::stoch3_msgs::CommandResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // STOCH3_MSGS_MESSAGE_COMMAND_H
