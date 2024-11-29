// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from people_msgs:msg/Person.idl
// generated code does not contain a copyright notice

#ifndef PEOPLE_MSGS__MSG__DETAIL__PERSON__TRAITS_HPP_
#define PEOPLE_MSGS__MSG__DETAIL__PERSON__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "people_msgs/msg/detail/person__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'position'
// Member 'velocity'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace people_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Person & msg,
  std::ostream & out)
{
  out << "{";
  // member: name
  {
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << ", ";
  }

  // member: position
  {
    out << "position: ";
    to_flow_style_yaml(msg.position, out);
    out << ", ";
  }

  // member: velocity
  {
    out << "velocity: ";
    to_flow_style_yaml(msg.velocity, out);
    out << ", ";
  }

  // member: reliability
  {
    out << "reliability: ";
    rosidl_generator_traits::value_to_yaml(msg.reliability, out);
    out << ", ";
  }

  // member: tagnames
  {
    if (msg.tagnames.size() == 0) {
      out << "tagnames: []";
    } else {
      out << "tagnames: [";
      size_t pending_items = msg.tagnames.size();
      for (auto item : msg.tagnames) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: tags
  {
    if (msg.tags.size() == 0) {
      out << "tags: []";
    } else {
      out << "tags: [";
      size_t pending_items = msg.tags.size();
      for (auto item : msg.tags) {
        rosidl_generator_traits::value_to_yaml(item, out);
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
  const Person & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << "\n";
  }

  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position:\n";
    to_block_style_yaml(msg.position, out, indentation + 2);
  }

  // member: velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity:\n";
    to_block_style_yaml(msg.velocity, out, indentation + 2);
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

  // member: tagnames
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.tagnames.size() == 0) {
      out << "tagnames: []\n";
    } else {
      out << "tagnames:\n";
      for (auto item : msg.tagnames) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: tags
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.tags.size() == 0) {
      out << "tags: []\n";
    } else {
      out << "tags:\n";
      for (auto item : msg.tags) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Person & msg, bool use_flow_style = false)
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
  const people_msgs::msg::Person & msg,
  std::ostream & out, size_t indentation = 0)
{
  people_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use people_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const people_msgs::msg::Person & msg)
{
  return people_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<people_msgs::msg::Person>()
{
  return "people_msgs::msg::Person";
}

template<>
inline const char * name<people_msgs::msg::Person>()
{
  return "people_msgs/msg/Person";
}

template<>
struct has_fixed_size<people_msgs::msg::Person>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<people_msgs::msg::Person>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<people_msgs::msg::Person>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PEOPLE_MSGS__MSG__DETAIL__PERSON__TRAITS_HPP_
