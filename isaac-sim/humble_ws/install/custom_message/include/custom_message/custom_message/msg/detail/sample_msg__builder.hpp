// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_message:msg/SampleMsg.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MESSAGE__MSG__DETAIL__SAMPLE_MSG__BUILDER_HPP_
#define CUSTOM_MESSAGE__MSG__DETAIL__SAMPLE_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_message/msg/detail/sample_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_message
{

namespace msg
{

namespace builder
{

class Init_SampleMsg_my_num
{
public:
  explicit Init_SampleMsg_my_num(::custom_message::msg::SampleMsg & msg)
  : msg_(msg)
  {}
  ::custom_message::msg::SampleMsg my_num(::custom_message::msg::SampleMsg::_my_num_type arg)
  {
    msg_.my_num = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_message::msg::SampleMsg msg_;
};

class Init_SampleMsg_my_string
{
public:
  Init_SampleMsg_my_string()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SampleMsg_my_num my_string(::custom_message::msg::SampleMsg::_my_string_type arg)
  {
    msg_.my_string = std::move(arg);
    return Init_SampleMsg_my_num(msg_);
  }

private:
  ::custom_message::msg::SampleMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_message::msg::SampleMsg>()
{
  return custom_message::msg::builder::Init_SampleMsg_my_string();
}

}  // namespace custom_message

#endif  // CUSTOM_MESSAGE__MSG__DETAIL__SAMPLE_MSG__BUILDER_HPP_
