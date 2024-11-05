// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from isaac_ros2_messages:srv/IsaacPose.idl
// generated code does not contain a copyright notice

#ifndef ISAAC_ROS2_MESSAGES__SRV__DETAIL__ISAAC_POSE__TRAITS_HPP_
#define ISAAC_ROS2_MESSAGES__SRV__DETAIL__ISAAC_POSE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "isaac_ros2_messages/srv/detail/isaac_pose__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'poses'
#include "geometry_msgs/msg/detail/pose__traits.hpp"
// Member 'velocities'
#include "geometry_msgs/msg/detail/twist__traits.hpp"
// Member 'scales'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace isaac_ros2_messages
{

namespace srv
{

inline void to_flow_style_yaml(
  const IsaacPose_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: names
  {
    if (msg.names.size() == 0) {
      out << "names: []";
    } else {
      out << "names: [";
      size_t pending_items = msg.names.size();
      for (auto item : msg.names) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: poses
  {
    if (msg.poses.size() == 0) {
      out << "poses: []";
    } else {
      out << "poses: [";
      size_t pending_items = msg.poses.size();
      for (auto item : msg.poses) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: velocities
  {
    if (msg.velocities.size() == 0) {
      out << "velocities: []";
    } else {
      out << "velocities: [";
      size_t pending_items = msg.velocities.size();
      for (auto item : msg.velocities) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: scales
  {
    if (msg.scales.size() == 0) {
      out << "scales: []";
    } else {
      out << "scales: [";
      size_t pending_items = msg.scales.size();
      for (auto item : msg.scales) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const IsaacPose_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: names
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.names.size() == 0) {
      out << "names: []\n";
    } else {
      out << "names:\n";
      for (auto item : msg.names) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: poses
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.poses.size() == 0) {
      out << "poses: []\n";
    } else {
      out << "poses:\n";
      for (auto item : msg.poses) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: velocities
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.velocities.size() == 0) {
      out << "velocities: []\n";
    } else {
      out << "velocities:\n";
      for (auto item : msg.velocities) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: scales
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.scales.size() == 0) {
      out << "scales: []\n";
    } else {
      out << "scales:\n";
      for (auto item : msg.scales) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const IsaacPose_Request & msg, bool use_flow_style = false)
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
  const isaac_ros2_messages::srv::IsaacPose_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  isaac_ros2_messages::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use isaac_ros2_messages::srv::to_yaml() instead")]]
inline std::string to_yaml(const isaac_ros2_messages::srv::IsaacPose_Request & msg)
{
  return isaac_ros2_messages::srv::to_yaml(msg);
}

template<>
inline const char * data_type<isaac_ros2_messages::srv::IsaacPose_Request>()
{
  return "isaac_ros2_messages::srv::IsaacPose_Request";
}

template<>
inline const char * name<isaac_ros2_messages::srv::IsaacPose_Request>()
{
  return "isaac_ros2_messages/srv/IsaacPose_Request";
}

template<>
struct has_fixed_size<isaac_ros2_messages::srv::IsaacPose_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<isaac_ros2_messages::srv::IsaacPose_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<isaac_ros2_messages::srv::IsaacPose_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace isaac_ros2_messages
{

namespace srv
{

inline void to_flow_style_yaml(
  const IsaacPose_Response & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const IsaacPose_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const IsaacPose_Response & msg, bool use_flow_style = false)
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
  const isaac_ros2_messages::srv::IsaacPose_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  isaac_ros2_messages::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use isaac_ros2_messages::srv::to_yaml() instead")]]
inline std::string to_yaml(const isaac_ros2_messages::srv::IsaacPose_Response & msg)
{
  return isaac_ros2_messages::srv::to_yaml(msg);
}

template<>
inline const char * data_type<isaac_ros2_messages::srv::IsaacPose_Response>()
{
  return "isaac_ros2_messages::srv::IsaacPose_Response";
}

template<>
inline const char * name<isaac_ros2_messages::srv::IsaacPose_Response>()
{
  return "isaac_ros2_messages/srv/IsaacPose_Response";
}

template<>
struct has_fixed_size<isaac_ros2_messages::srv::IsaacPose_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<isaac_ros2_messages::srv::IsaacPose_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<isaac_ros2_messages::srv::IsaacPose_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<isaac_ros2_messages::srv::IsaacPose>()
{
  return "isaac_ros2_messages::srv::IsaacPose";
}

template<>
inline const char * name<isaac_ros2_messages::srv::IsaacPose>()
{
  return "isaac_ros2_messages/srv/IsaacPose";
}

template<>
struct has_fixed_size<isaac_ros2_messages::srv::IsaacPose>
  : std::integral_constant<
    bool,
    has_fixed_size<isaac_ros2_messages::srv::IsaacPose_Request>::value &&
    has_fixed_size<isaac_ros2_messages::srv::IsaacPose_Response>::value
  >
{
};

template<>
struct has_bounded_size<isaac_ros2_messages::srv::IsaacPose>
  : std::integral_constant<
    bool,
    has_bounded_size<isaac_ros2_messages::srv::IsaacPose_Request>::value &&
    has_bounded_size<isaac_ros2_messages::srv::IsaacPose_Response>::value
  >
{
};

template<>
struct is_service<isaac_ros2_messages::srv::IsaacPose>
  : std::true_type
{
};

template<>
struct is_service_request<isaac_ros2_messages::srv::IsaacPose_Request>
  : std::true_type
{
};

template<>
struct is_service_response<isaac_ros2_messages::srv::IsaacPose_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ISAAC_ROS2_MESSAGES__SRV__DETAIL__ISAAC_POSE__TRAITS_HPP_
