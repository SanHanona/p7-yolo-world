// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from isaac_ros2_messages:srv/SetPrimAttribute.idl
// generated code does not contain a copyright notice

#ifndef ISAAC_ROS2_MESSAGES__SRV__DETAIL__SET_PRIM_ATTRIBUTE__TRAITS_HPP_
#define ISAAC_ROS2_MESSAGES__SRV__DETAIL__SET_PRIM_ATTRIBUTE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "isaac_ros2_messages/srv/detail/set_prim_attribute__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace isaac_ros2_messages
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetPrimAttribute_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: path
  {
    out << "path: ";
    rosidl_generator_traits::value_to_yaml(msg.path, out);
    out << ", ";
  }

  // member: attribute
  {
    out << "attribute: ";
    rosidl_generator_traits::value_to_yaml(msg.attribute, out);
    out << ", ";
  }

  // member: value
  {
    out << "value: ";
    rosidl_generator_traits::value_to_yaml(msg.value, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetPrimAttribute_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "path: ";
    rosidl_generator_traits::value_to_yaml(msg.path, out);
    out << "\n";
  }

  // member: attribute
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "attribute: ";
    rosidl_generator_traits::value_to_yaml(msg.attribute, out);
    out << "\n";
  }

  // member: value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "value: ";
    rosidl_generator_traits::value_to_yaml(msg.value, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetPrimAttribute_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace isaac_ros2_messages

namespace rosidl_generator_traits
{

[[deprecated("use isaac_ros2_messages::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const isaac_ros2_messages::srv::SetPrimAttribute_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  isaac_ros2_messages::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use isaac_ros2_messages::srv::to_yaml() instead")]]
inline std::string to_yaml(const isaac_ros2_messages::srv::SetPrimAttribute_Request & msg)
{
  return isaac_ros2_messages::srv::to_yaml(msg);
}

template<>
inline const char * data_type<isaac_ros2_messages::srv::SetPrimAttribute_Request>()
{
  return "isaac_ros2_messages::srv::SetPrimAttribute_Request";
}

template<>
inline const char * name<isaac_ros2_messages::srv::SetPrimAttribute_Request>()
{
  return "isaac_ros2_messages/srv/SetPrimAttribute_Request";
}

template<>
struct has_fixed_size<isaac_ros2_messages::srv::SetPrimAttribute_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<isaac_ros2_messages::srv::SetPrimAttribute_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<isaac_ros2_messages::srv::SetPrimAttribute_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace isaac_ros2_messages
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetPrimAttribute_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetPrimAttribute_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetPrimAttribute_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace isaac_ros2_messages

namespace rosidl_generator_traits
{

[[deprecated("use isaac_ros2_messages::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const isaac_ros2_messages::srv::SetPrimAttribute_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  isaac_ros2_messages::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use isaac_ros2_messages::srv::to_yaml() instead")]]
inline std::string to_yaml(const isaac_ros2_messages::srv::SetPrimAttribute_Response & msg)
{
  return isaac_ros2_messages::srv::to_yaml(msg);
}

template<>
inline const char * data_type<isaac_ros2_messages::srv::SetPrimAttribute_Response>()
{
  return "isaac_ros2_messages::srv::SetPrimAttribute_Response";
}

template<>
inline const char * name<isaac_ros2_messages::srv::SetPrimAttribute_Response>()
{
  return "isaac_ros2_messages/srv/SetPrimAttribute_Response";
}

template<>
struct has_fixed_size<isaac_ros2_messages::srv::SetPrimAttribute_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<isaac_ros2_messages::srv::SetPrimAttribute_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<isaac_ros2_messages::srv::SetPrimAttribute_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<isaac_ros2_messages::srv::SetPrimAttribute>()
{
  return "isaac_ros2_messages::srv::SetPrimAttribute";
}

template<>
inline const char * name<isaac_ros2_messages::srv::SetPrimAttribute>()
{
  return "isaac_ros2_messages/srv/SetPrimAttribute";
}

template<>
struct has_fixed_size<isaac_ros2_messages::srv::SetPrimAttribute>
  : std::integral_constant<
    bool,
    has_fixed_size<isaac_ros2_messages::srv::SetPrimAttribute_Request>::value &&
    has_fixed_size<isaac_ros2_messages::srv::SetPrimAttribute_Response>::value
  >
{
};

template<>
struct has_bounded_size<isaac_ros2_messages::srv::SetPrimAttribute>
  : std::integral_constant<
    bool,
    has_bounded_size<isaac_ros2_messages::srv::SetPrimAttribute_Request>::value &&
    has_bounded_size<isaac_ros2_messages::srv::SetPrimAttribute_Response>::value
  >
{
};

template<>
struct is_service<isaac_ros2_messages::srv::SetPrimAttribute>
  : std::true_type
{
};

template<>
struct is_service_request<isaac_ros2_messages::srv::SetPrimAttribute_Request>
  : std::true_type
{
};

template<>
struct is_service_response<isaac_ros2_messages::srv::SetPrimAttribute_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ISAAC_ROS2_MESSAGES__SRV__DETAIL__SET_PRIM_ATTRIBUTE__TRAITS_HPP_
