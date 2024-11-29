// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from people_msgs:msg/PositionMeasurement.idl
// generated code does not contain a copyright notice

#ifndef PEOPLE_MSGS__MSG__DETAIL__POSITION_MEASUREMENT__TRAITS_HPP_
#define PEOPLE_MSGS__MSG__DETAIL__POSITION_MEASUREMENT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "people_msgs/msg/detail/position_measurement__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'pos'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace people_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const PositionMeasurement & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: name
  {
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << ", ";
  }

  // member: object_id
  {
    out << "object_id: ";
    rosidl_generator_traits::value_to_yaml(msg.object_id, out);
    out << ", ";
  }

  // member: pos
  {
    out << "pos: ";
    to_flow_style_yaml(msg.pos, out);
    out << ", ";
  }

  // member: reliability
  {
    out << "reliability: ";
    rosidl_generator_traits::value_to_yaml(msg.reliability, out);
    out << ", ";
  }

  // member: covariance
  {
    if (msg.covariance.size() == 0) {
      out << "covariance: []";
    } else {
      out << "covariance: [";
      size_t pending_items = msg.covariance.size();
      for (auto item : msg.covariance) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: initialization
  {
    out << "initialization: ";
    rosidl_generator_traits::character_value_to_yaml(msg.initialization, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PositionMeasurement & msg,
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

  // member: name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << "\n";
  }

  // member: object_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "object_id: ";
    rosidl_generator_traits::value_to_yaml(msg.object_id, out);
    out << "\n";
  }

  // member: pos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pos:\n";
    to_block_style_yaml(msg.pos, out, indentation + 2);
  }

  // member: reliability
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reliability: ";
    rosidl_generator_traits::value_to_yaml(msg.reliability, out);
    out << "\n";
  }

  // member: covariance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.covariance.size() == 0) {
      out << "covariance: []\n";
    } else {
      out << "covariance:\n";
      for (auto item : msg.covariance) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: initialization
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "initialization: ";
    rosidl_generator_traits::character_value_to_yaml(msg.initialization, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PositionMeasurement & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace people_msgs

namespace rosidl_generator_traits
{

[[deprecated("use people_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const people_msgs::msg::PositionMeasurement & msg,
  std::ostream & out, size_t indentation = 0)
{
  people_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use people_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const people_msgs::msg::PositionMeasurement & msg)
{
  return people_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<people_msgs::msg::PositionMeasurement>()
{
  return "people_msgs::msg::PositionMeasurement";
}

template<>
inline const char * name<people_msgs::msg::PositionMeasurement>()
{
  return "people_msgs/msg/PositionMeasurement";
}

template<>
struct has_fixed_size<people_msgs::msg::PositionMeasurement>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<people_msgs::msg::PositionMeasurement>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<people_msgs::msg::PositionMeasurement>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PEOPLE_MSGS__MSG__DETAIL__POSITION_MEASUREMENT__TRAITS_HPP_
