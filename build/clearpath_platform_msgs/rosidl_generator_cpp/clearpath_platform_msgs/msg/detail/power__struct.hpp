// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from clearpath_platform_msgs:msg/Power.idl
// generated code does not contain a copyright notice

#ifndef CLEARPATH_PLATFORM_MSGS__MSG__DETAIL__POWER__STRUCT_HPP_
#define CLEARPATH_PLATFORM_MSGS__MSG__DETAIL__POWER__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__clearpath_platform_msgs__msg__Power __attribute__((deprecated))
#else
# define DEPRECATED__clearpath_platform_msgs__msg__Power __declspec(deprecated)
#endif

namespace clearpath_platform_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Power_
{
  using Type = Power_<ContainerAllocator>;

  explicit Power_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->shore_power_connected = 0;
      this->battery_connected = 0;
      this->power_12v_user_nominal = 0;
      this->charger_connected = 0;
      this->charging_complete = 0;
    }
  }

  explicit Power_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->shore_power_connected = 0;
      this->battery_connected = 0;
      this->power_12v_user_nominal = 0;
      this->charger_connected = 0;
      this->charging_complete = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _shore_power_connected_type =
    int8_t;
  _shore_power_connected_type shore_power_connected;
  using _battery_connected_type =
    int8_t;
  _battery_connected_type battery_connected;
  using _power_12v_user_nominal_type =
    int8_t;
  _power_12v_user_nominal_type power_12v_user_nominal;
  using _charger_connected_type =
    int8_t;
  _charger_connected_type charger_connected;
  using _charging_complete_type =
    int8_t;
  _charging_complete_type charging_complete;
  using _measured_voltages_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _measured_voltages_type measured_voltages;
  using _measured_currents_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _measured_currents_type measured_currents;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__shore_power_connected(
    const int8_t & _arg)
  {
    this->shore_power_connected = _arg;
    return *this;
  }
  Type & set__battery_connected(
    const int8_t & _arg)
  {
    this->battery_connected = _arg;
    return *this;
  }
  Type & set__power_12v_user_nominal(
    const int8_t & _arg)
  {
    this->power_12v_user_nominal = _arg;
    return *this;
  }
  Type & set__charger_connected(
    const int8_t & _arg)
  {
    this->charger_connected = _arg;
    return *this;
  }
  Type & set__charging_complete(
    const int8_t & _arg)
  {
    this->charging_complete = _arg;
    return *this;
  }
  Type & set__measured_voltages(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->measured_voltages = _arg;
    return *this;
  }
  Type & set__measured_currents(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->measured_currents = _arg;
    return *this;
  }

  // constant declarations
  static constexpr int8_t NOT_APPLICABLE =
    -1;
  static constexpr uint8_t J100_MEASURED_BATTERY =
    0u;
  static constexpr uint8_t J100_MEASURED_5V =
    1u;
  static constexpr uint8_t J100_MEASURED_12V =
    2u;
  static constexpr uint8_t D100_MEASURED_BATTERY =
    0u;
  static constexpr uint8_t D100_MEASURED_5V =
    1u;
  static constexpr uint8_t D100_MEASURED_12V =
    2u;
  static constexpr uint8_t D150_MEASURED_BATTERY =
    0u;
  static constexpr uint8_t D150_MEASURED_5V =
    1u;
  static constexpr uint8_t D150_MEASURED_12V =
    2u;
  static constexpr uint8_t W200_MEASURED_BATTERY =
    0u;
  static constexpr uint8_t W200_MEASURED_12V =
    1u;
  static constexpr uint8_t W200_MEASURED_24V =
    2u;
  static constexpr uint8_t W200_MEASURED_48V =
    3u;
  static constexpr uint8_t R100_MEASURED_BATTERY =
    0u;
  static constexpr uint8_t R100_MEASURED_5V =
    1u;
  static constexpr uint8_t R100_MEASURED_12V =
    2u;
  static constexpr uint8_t R100_MEASURED_INVERTER =
    3u;
  static constexpr uint8_t R100_MEASURED_FRONT_AXLE =
    4u;
  static constexpr uint8_t R100_MEASURED_REAR_AXLE =
    5u;
  static constexpr uint8_t R100_MEASURED_LIGHT =
    6u;
  static constexpr uint8_t A200_BATTERY_VOLTAGE =
    0u;
  static constexpr uint8_t A200_LEFT_DRIVER_VOLTAGE =
    1u;
  static constexpr uint8_t A200_RIGHT_DRIVER_VOLTAGE =
    2u;
  static constexpr uint8_t A200_VOLTAGES_SIZE =
    3u;
  static constexpr uint8_t J100_TOTAL_CURRENT =
    0u;
  static constexpr uint8_t J100_COMPUTER_CURRENT =
    1u;
  static constexpr uint8_t J100_DRIVE_CURRENT =
    2u;
  static constexpr uint8_t J100_USER_CURRENT =
    3u;
  static constexpr uint8_t D100_TOTAL_CURRENT =
    0u;
  static constexpr uint8_t D100_COMPUTER_CURRENT =
    1u;
  static constexpr uint8_t D150_TOTAL_CURRENT =
    0u;
  static constexpr uint8_t D150_COMPUTER_CURRENT =
    1u;
  static constexpr uint8_t W200_TOTAL_CURRENT =
    0u;
  static constexpr uint8_t W200_COMPUTER_CURRENT =
    1u;
  static constexpr uint8_t W200_12V_CURRENT =
    2u;
  static constexpr uint8_t W200_24V_CURRENT =
    3u;
  static constexpr uint8_t R100_TOTAL_CURRENT =
    0u;
  static constexpr uint8_t A200_MCU_AND_USER_PORT_CURRENT =
    0u;
  static constexpr uint8_t A200_LEFT_DRIVER_CURRENT =
    1u;
  static constexpr uint8_t A200_RIGHT_DRIVER_CURRENT =
    2u;
  static constexpr uint8_t A200_CURRENTS_SIZE =
    3u;

  // pointer types
  using RawPtr =
    clearpath_platform_msgs::msg::Power_<ContainerAllocator> *;
  using ConstRawPtr =
    const clearpath_platform_msgs::msg::Power_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<clearpath_platform_msgs::msg::Power_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<clearpath_platform_msgs::msg::Power_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      clearpath_platform_msgs::msg::Power_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<clearpath_platform_msgs::msg::Power_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      clearpath_platform_msgs::msg::Power_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<clearpath_platform_msgs::msg::Power_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<clearpath_platform_msgs::msg::Power_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<clearpath_platform_msgs::msg::Power_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__clearpath_platform_msgs__msg__Power
    std::shared_ptr<clearpath_platform_msgs::msg::Power_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__clearpath_platform_msgs__msg__Power
    std::shared_ptr<clearpath_platform_msgs::msg::Power_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Power_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->shore_power_connected != other.shore_power_connected) {
      return false;
    }
    if (this->battery_connected != other.battery_connected) {
      return false;
    }
    if (this->power_12v_user_nominal != other.power_12v_user_nominal) {
      return false;
    }
    if (this->charger_connected != other.charger_connected) {
      return false;
    }
    if (this->charging_complete != other.charging_complete) {
      return false;
    }
    if (this->measured_voltages != other.measured_voltages) {
      return false;
    }
    if (this->measured_currents != other.measured_currents) {
      return false;
    }
    return true;
  }
  bool operator!=(const Power_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Power_

// alias to use template instance with default allocator
using Power =
  clearpath_platform_msgs::msg::Power_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int8_t Power_<ContainerAllocator>::NOT_APPLICABLE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Power_<ContainerAllocator>::J100_MEASURED_BATTERY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Power_<ContainerAllocator>::J100_MEASURED_5V;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Power_<ContainerAllocator>::J100_MEASURED_12V;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Power_<ContainerAllocator>::D100_MEASURED_BATTERY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Power_<ContainerAllocator>::D100_MEASURED_5V;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Power_<ContainerAllocator>::D100_MEASURED_12V;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Power_<ContainerAllocator>::D150_MEASURED_BATTERY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Power_<ContainerAllocator>::D150_MEASURED_5V;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Power_<ContainerAllocator>::D150_MEASURED_12V;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Power_<ContainerAllocator>::W200_MEASURED_BATTERY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Power_<ContainerAllocator>::W200_MEASURED_12V;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Power_<ContainerAllocator>::W200_MEASURED_24V;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Power_<ContainerAllocator>::W200_MEASURED_48V;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Power_<ContainerAllocator>::R100_MEASURED_BATTERY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Power_<ContainerAllocator>::R100_MEASURED_5V;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Power_<ContainerAllocator>::R100_MEASURED_12V;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Power_<ContainerAllocator>::R100_MEASURED_INVERTER;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Power_<ContainerAllocator>::R100_MEASURED_FRONT_AXLE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Power_<ContainerAllocator>::R100_MEASURED_REAR_AXLE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Power_<ContainerAllocator>::R100_MEASURED_LIGHT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Power_<ContainerAllocator>::A200_BATTERY_VOLTAGE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Power_<ContainerAllocator>::A200_LEFT_DRIVER_VOLTAGE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Power_<ContainerAllocator>::A200_RIGHT_DRIVER_VOLTAGE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Power_<ContainerAllocator>::A200_VOLTAGES_SIZE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Power_<ContainerAllocator>::J100_TOTAL_CURRENT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Power_<ContainerAllocator>::J100_COMPUTER_CURRENT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Power_<ContainerAllocator>::J100_DRIVE_CURRENT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Power_<ContainerAllocator>::J100_USER_CURRENT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Power_<ContainerAllocator>::D100_TOTAL_CURRENT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Power_<ContainerAllocator>::D100_COMPUTER_CURRENT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Power_<ContainerAllocator>::D150_TOTAL_CURRENT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Power_<ContainerAllocator>::D150_COMPUTER_CURRENT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Power_<ContainerAllocator>::W200_TOTAL_CURRENT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Power_<ContainerAllocator>::W200_COMPUTER_CURRENT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Power_<ContainerAllocator>::W200_12V_CURRENT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Power_<ContainerAllocator>::W200_24V_CURRENT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Power_<ContainerAllocator>::R100_TOTAL_CURRENT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Power_<ContainerAllocator>::A200_MCU_AND_USER_PORT_CURRENT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Power_<ContainerAllocator>::A200_LEFT_DRIVER_CURRENT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Power_<ContainerAllocator>::A200_RIGHT_DRIVER_CURRENT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Power_<ContainerAllocator>::A200_CURRENTS_SIZE;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace clearpath_platform_msgs

#endif  // CLEARPATH_PLATFORM_MSGS__MSG__DETAIL__POWER__STRUCT_HPP_
