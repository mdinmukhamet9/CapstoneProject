// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from clearpath_platform_msgs:msg/Drive.idl
// generated code does not contain a copyright notice

#ifndef CLEARPATH_PLATFORM_MSGS__MSG__DETAIL__DRIVE__BUILDER_HPP_
#define CLEARPATH_PLATFORM_MSGS__MSG__DETAIL__DRIVE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "clearpath_platform_msgs/msg/detail/drive__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace clearpath_platform_msgs
{

namespace msg
{

namespace builder
{

class Init_Drive_drivers
{
public:
  explicit Init_Drive_drivers(::clearpath_platform_msgs::msg::Drive & msg)
  : msg_(msg)
  {}
  ::clearpath_platform_msgs::msg::Drive drivers(::clearpath_platform_msgs::msg::Drive::_drivers_type arg)
  {
    msg_.drivers = std::move(arg);
    return std::move(msg_);
  }

private:
  ::clearpath_platform_msgs::msg::Drive msg_;
};

class Init_Drive_mode
{
public:
  Init_Drive_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Drive_drivers mode(::clearpath_platform_msgs::msg::Drive::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_Drive_drivers(msg_);
  }

private:
  ::clearpath_platform_msgs::msg::Drive msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::clearpath_platform_msgs::msg::Drive>()
{
  return clearpath_platform_msgs::msg::builder::Init_Drive_mode();
}

}  // namespace clearpath_platform_msgs

#endif  // CLEARPATH_PLATFORM_MSGS__MSG__DETAIL__DRIVE__BUILDER_HPP_
