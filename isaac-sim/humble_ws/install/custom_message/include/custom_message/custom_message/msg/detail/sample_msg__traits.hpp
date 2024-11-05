// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_message:msg/SampleMsg.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MESSAGE__MSG__DETAIL__SAMPLE_MSG__TRAITS_HPP_
#define CUSTOM_MESSAGE__MSG__DETAIL__SAMPLE_MSG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_message/msg/detail/sample_msg__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'my_string'
#include "std_msgs/msg/detail/string__traits.hpp"

namespace custom_message
{

namespace msg
{

inline void to_flow_style_yaml(
  const SampleMsg & msg,
  std::ostream & out)
{
  out << "{";
  // member: my_string
  {
    out << "my_string: ";
    to_flow_style_yaml(msg.my_string, out);
    out << ", ";
  }

  // member: my_num
  {
    out << "my_num: ";
    rosidl_generator_traits::value_to_yaml(msg.my_num, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SampleMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: my_string
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "my_string:\n";
    to_block_style_yaml(msg.my_string, out, indentation + 2);
  }

  // member: my_num
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "my_num: ";
    rosidl_generator_traits::value_to_yaml(msg.my_num, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SampleMsg & msg, bool use_flow_style = false)
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

}  // namespace custom_message

namespace rosidl_generator_traits
{

[[deprecated("use custom_message::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_message::msg::SampleMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_message::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_message::msg::to_yaml() instead")]]
inline std::string to_yaml(const custom_message::msg::SampleMsg & msg)
{
  return custom_message::msg::to_yaml(msg);
}

template<>
inline const char * data_type<custom_message::msg::SampleMsg>()
{
  return "custom_message::msg::SampleMsg";
}

template<>
inline const char * name<custom_message::msg::SampleMsg>()
{
  return "custom_message/msg/SampleMsg";
}

template<>
struct has_fixed_size<custom_message::msg::SampleMsg>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::String>::value> {};

template<>
struct has_bounded_size<custom_message::msg::SampleMsg>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::String>::value> {};

template<>
struct is_message<custom_message::msg::SampleMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_MESSAGE__MSG__DETAIL__SAMPLE_MSG__TRAITS_HPP_
