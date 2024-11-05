// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from isaac_ros2_messages:srv/GetPrims.idl
// generated code does not contain a copyright notice

#ifndef ISAAC_ROS2_MESSAGES__SRV__DETAIL__GET_PRIMS__TRAITS_HPP_
#define ISAAC_ROS2_MESSAGES__SRV__DETAIL__GET_PRIMS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "isaac_ros2_messages/srv/detail/get_prims__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace isaac_ros2_messages
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetPrims_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: path
  {
    out << "path: ";
    rosidl_generator_traits::value_to_yaml(msg.path, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetPrims_Request & msg,
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetPrims_Request & msg, bool use_flow_style = false)
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
  const isaac_ros2_messages::srv::GetPrims_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  isaac_ros2_messages::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use isaac_ros2_messages::srv::to_yaml() instead")]]
inline std::string to_yaml(const isaac_ros2_messages::srv::GetPrims_Request & msg)
{
  return isaac_ros2_messages::srv::to_yaml(msg);
}

template<>
inline const char * data_type<isaac_ros2_messages::srv::GetPrims_Request>()
{
  return "isaac_ros2_messages::srv::GetPrims_Request";
}

template<>
inline const char * name<isaac_ros2_messages::srv::GetPrims_Request>()
{
  return "isaac_ros2_messages/srv/GetPrims_Request";
}

template<>
struct has_fixed_size<isaac_ros2_messages::srv::GetPrims_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<isaac_ros2_messages::srv::GetPrims_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<isaac_ros2_messages::srv::GetPrims_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace isaac_ros2_messages
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetPrims_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: paths
  {
    if (msg.paths.size() == 0) {
      out << "paths: []";
    } else {
      out << "paths: [";
      size_t pending_items = msg.paths.size();
      for (auto item : msg.paths) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: types
  {
    if (msg.types.size() == 0) {
      out << "types: []";
    } else {
      out << "types: [";
      size_t pending_items = msg.types.size();
      for (auto item : msg.types) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
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
  const GetPrims_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: paths
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.paths.size() == 0) {
      out << "paths: []\n";
    } else {
      out << "paths:\n";
      for (auto item : msg.paths) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: types
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.types.size() == 0) {
      out << "types: []\n";
    } else {
      out << "types:\n";
      for (auto item : msg.types) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
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

inline std::string to_yaml(const GetPrims_Response & msg, bool use_flow_style = false)
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
  const isaac_ros2_messages::srv::GetPrims_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  isaac_ros2_messages::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use isaac_ros2_messages::srv::to_yaml() instead")]]
inline std::string to_yaml(const isaac_ros2_messages::srv::GetPrims_Response & msg)
{
  return isaac_ros2_messages::srv::to_yaml(msg);
}

template<>
inline const char * data_type<isaac_ros2_messages::srv::GetPrims_Response>()
{
  return "isaac_ros2_messages::srv::GetPrims_Response";
}

template<>
inline const char * name<isaac_ros2_messages::srv::GetPrims_Response>()
{
  return "isaac_ros2_messages/srv/GetPrims_Response";
}

template<>
struct has_fixed_size<isaac_ros2_messages::srv::GetPrims_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<isaac_ros2_messages::srv::GetPrims_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<isaac_ros2_messages::srv::GetPrims_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<isaac_ros2_messages::srv::GetPrims>()
{
  return "isaac_ros2_messages::srv::GetPrims";
}

template<>
inline const char * name<isaac_ros2_messages::srv::GetPrims>()
{
  return "isaac_ros2_messages/srv/GetPrims";
}

template<>
struct has_fixed_size<isaac_ros2_messages::srv::GetPrims>
  : std::integral_constant<
    bool,
    has_fixed_size<isaac_ros2_messages::srv::GetPrims_Request>::value &&
    has_fixed_size<isaac_ros2_messages::srv::GetPrims_Response>::value
  >
{
};

template<>
struct has_bounded_size<isaac_ros2_messages::srv::GetPrims>
  : std::integral_constant<
    bool,
    has_bounded_size<isaac_ros2_messages::srv::GetPrims_Request>::value &&
    has_bounded_size<isaac_ros2_messages::srv::GetPrims_Response>::value
  >
{
};

template<>
struct is_service<isaac_ros2_messages::srv::GetPrims>
  : std::true_type
{
};

template<>
struct is_service_request<isaac_ros2_messages::srv::GetPrims_Request>
  : std::true_type
{
};

template<>
struct is_service_response<isaac_ros2_messages::srv::GetPrims_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ISAAC_ROS2_MESSAGES__SRV__DETAIL__GET_PRIMS__TRAITS_HPP_
