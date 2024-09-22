// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from clearpath_platform_msgs:msg/DriveFeedback.idl
// generated code does not contain a copyright notice

#ifndef CLEARPATH_PLATFORM_MSGS__MSG__DETAIL__DRIVE_FEEDBACK__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define CLEARPATH_PLATFORM_MSGS__MSG__DETAIL__DRIVE_FEEDBACK__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "clearpath_platform_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "clearpath_platform_msgs/msg/detail/drive_feedback__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace clearpath_platform_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_clearpath_platform_msgs
cdr_serialize(
  const clearpath_platform_msgs::msg::DriveFeedback & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_clearpath_platform_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  clearpath_platform_msgs::msg::DriveFeedback & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_clearpath_platform_msgs
get_serialized_size(
  const clearpath_platform_msgs::msg::DriveFeedback & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_clearpath_platform_msgs
max_serialized_size_DriveFeedback(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace clearpath_platform_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_clearpath_platform_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, clearpath_platform_msgs, msg, DriveFeedback)();

#ifdef __cplusplus
}
#endif

#endif  // CLEARPATH_PLATFORM_MSGS__MSG__DETAIL__DRIVE_FEEDBACK__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
