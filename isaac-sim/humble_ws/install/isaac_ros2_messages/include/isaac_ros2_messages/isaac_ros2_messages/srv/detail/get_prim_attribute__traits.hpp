// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from isaac_ros2_messages:srv/GetPrimAttribute.idl
// generated code does not contain a copyright notice

#ifndef ISAAC_ROS2_MESSAGES__SRV__DETAIL__GET_PRIM_ATTRIBUTE__TRAITS_HPP_
#define ISAAC_ROS2_MESSAGES__SRV__DETAIL__GET_PRIM_ATTRIBUTE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "isaac_ros2_messages/srv/detail/get_prim_attribute__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace isaac_ros2_messages
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetPrimAttribute_Request & msg,
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
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetPrimAttribute_Request & msg,
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetPrimAttribute_Request & msg, bool use_flow_style = false)
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
  const isaac_ros2_messages::srv::GetPrimAttribute_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  isaac_ros2_messages::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use isaac_ros2_messages::srv::to_yaml() instead")]]
inline std::string to_yaml(const isaac_ros2_messages::srv::GetPrimAttribute_Request & msg)
{
  return isaac_ros2_messages::srv::to_yaml(msg);
}

template<>
inline const char * data_type<isaac_ros2_messages::srv::GetPrimAttribute_Request>()
{
  return "isaac_ros2_messages::srv::GetPrimAttribute_Request";
}

template<>
inline const char * name<isaac_ros2_messages::srv::GetPrimAttribute_Request>()
{
  return "isaac_ros2_messages/srv/GetPrimAttribute_Request";
}

template<>
struct has_fixed_size<isaac_ros2_messages::srv::GetPrimAttribute_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<isaac_ros2_messages::srv::GetPrimAttribute_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<isaac_ros2_messages::srv::GetPrimAttribute_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace isaac_ros2_messages
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetPrimAttribute_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: value
  {
    out << "value: ";
    rosidl_generator_traits::value_to_yaml(msg.value, out);
    out << ", ";
  }

  // member: type
  {
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
    out << ", ";
  }

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
  const GetPrimAttribute_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "value: ";
    rosidl_generator_traits::value_to_yaml(msg.value, out);
    out << "\n";
  }

  // member: type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
    out << "\n";
  }

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

inline std::string to_yaml(const GetPrimAttribute_Response & msg, bool use_flow_style = false)
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
  const isaac_ros2_messages::srv::GetPrimAttribute_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  isaac_ros2_messages::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use isaac_ros2_messages::srv::to_yaml() instead")]]
inline std::string to_yaml(const isaac_ros2_messages::srv::GetPrimAttribute_Response & msg)
{
  return isaac_ros2_messages::srv::to_yaml(msg);
}

template<>
inline const char * data_type<isaac_ros2_messages::srv::GetPrimAttribute_Response>()
{
  return "isaac_ros2_messages::srv::GetPrimAttribute_Response";
}

template<>
inline const char * name<isaac_ros2_messages::srv::GetPrimAttribute_Response>()
{
  return "isaac_ros2_messages/srv/GetPrimAttribute_Response";
}

template<>
struct has_fixed_size<isaac_ros2_messages::srv::GetPrimAttribute_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<isaac_ros2_messages::srv::GetPrimAttribute_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<isaac_ros2_messages::srv::GetPrimAttribute_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<isaac_ros2_messages::srv::GetPrimAttribute>()
{
  return "isaac_ros2_messages::srv::GetPrimAttribute";
}

template<>
inline const char * name<isaac_ros2_messages::srv::GetPrimAttribute>()
{
  return "isaac_ros2_messages/srv/GetPrimAttribute";
}

template<>
struct has_fixed_size<isaac_ros2_messages::srv::GetPrimAttribute>
  : std::integral_constant<
    bool,
    has_fixed_size<isaac_ros2_messages::srv::GetPrimAttribute_Request>::value &&
    has_fixed_size<isaac_ros2_messages::srv::GetPrimAttribute_Response>::value
  >
{
};

template<>
struct has_bounded_size<isaac_ros2_messages::srv::GetPrimAttribute>
  : std::integral_constant<
    bool,
    has_bounded_size<isaac_ros2_messages::srv::GetPrimAttribute_Request>::value &&
    has_bounded_size<isaac_ros2_messages::srv::GetPrimAttribute_Response>::value
  >
{
};

template<>
struct is_service<isaac_ros2_messages::srv::GetPrimAttribute>
  : std::true_type
{
};

template<>
struct is_service_request<isaac_ros2_messages::srv::GetPrimAttribute_Request>
  : std::true_type
{
};

template<>
struct is_service_response<isaac_ros2_messages::srv::GetPrimAttribute_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ISAAC_ROS2_MESSAGES__SRV__DETAIL__GET_PRIM_ATTRIBUTE__TRAITS_HPP_
