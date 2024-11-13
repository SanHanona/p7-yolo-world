// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_message:msg/SampleMsg.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MESSAGE__MSG__DETAIL__SAMPLE_MSG__STRUCT_HPP_
#define CUSTOM_MESSAGE__MSG__DETAIL__SAMPLE_MSG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'my_string'
#include "std_msgs/msg/detail/string__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__custom_message__msg__SampleMsg __attribute__((deprecated))
#else
# define DEPRECATED__custom_message__msg__SampleMsg __declspec(deprecated)
#endif

namespace custom_message
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SampleMsg_
{
  using Type = SampleMsg_<ContainerAllocator>;

  explicit SampleMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : my_string(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->my_num = 0ll;
    }
  }

  explicit SampleMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : my_string(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->my_num = 0ll;
    }
  }

  // field types and members
  using _my_string_type =
    std_msgs::msg::String_<ContainerAllocator>;
  _my_string_type my_string;
  using _my_num_type =
    int64_t;
  _my_num_type my_num;

  // setters for named parameter idiom
  Type & set__my_string(
    const std_msgs::msg::String_<ContainerAllocator> & _arg)
  {
    this->my_string = _arg;
    return *this;
  }
  Type & set__my_num(
    const int64_t & _arg)
  {
    this->my_num = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_message::msg::SampleMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_message::msg::SampleMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_message::msg::SampleMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_message::msg::SampleMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_message::msg::SampleMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_message::msg::SampleMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_message::msg::SampleMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_message::msg::SampleMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_message::msg::SampleMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_message::msg::SampleMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_message__msg__SampleMsg
    std::shared_ptr<custom_message::msg::SampleMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_message__msg__SampleMsg
    std::shared_ptr<custom_message::msg::SampleMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SampleMsg_ & other) const
  {
    if (this->my_string != other.my_string) {
      return false;
    }
    if (this->my_num != other.my_num) {
      return false;
    }
    return true;
  }
  bool operator!=(const SampleMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SampleMsg_

// alias to use template instance with default allocator
using SampleMsg =
  custom_message::msg::SampleMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_message

#endif  // CUSTOM_MESSAGE__MSG__DETAIL__SAMPLE_MSG__STRUCT_HPP_
