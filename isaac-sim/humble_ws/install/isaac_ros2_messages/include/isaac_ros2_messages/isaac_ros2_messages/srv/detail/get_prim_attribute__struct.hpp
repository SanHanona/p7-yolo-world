// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from isaac_ros2_messages:srv/GetPrimAttribute.idl
// generated code does not contain a copyright notice

#ifndef ISAAC_ROS2_MESSAGES__SRV__DETAIL__GET_PRIM_ATTRIBUTE__STRUCT_HPP_
#define ISAAC_ROS2_MESSAGES__SRV__DETAIL__GET_PRIM_ATTRIBUTE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__isaac_ros2_messages__srv__GetPrimAttribute_Request __attribute__((deprecated))
#else
# define DEPRECATED__isaac_ros2_messages__srv__GetPrimAttribute_Request __declspec(deprecated)
#endif

namespace isaac_ros2_messages
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetPrimAttribute_Request_
{
  using Type = GetPrimAttribute_Request_<ContainerAllocator>;

  explicit GetPrimAttribute_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->path = "";
      this->attribute = "";
    }
  }

  explicit GetPrimAttribute_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : path(_alloc),
    attribute(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->path = "";
      this->attribute = "";
    }
  }

  // field types and members
  using _path_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _path_type path;
  using _attribute_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _attribute_type attribute;

  // setters for named parameter idiom
  Type & set__path(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->path = _arg;
    return *this;
  }
  Type & set__attribute(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->attribute = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    isaac_ros2_messages::srv::GetPrimAttribute_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const isaac_ros2_messages::srv::GetPrimAttribute_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<isaac_ros2_messages::srv::GetPrimAttribute_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<isaac_ros2_messages::srv::GetPrimAttribute_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      isaac_ros2_messages::srv::GetPrimAttribute_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<isaac_ros2_messages::srv::GetPrimAttribute_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      isaac_ros2_messages::srv::GetPrimAttribute_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<isaac_ros2_messages::srv::GetPrimAttribute_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<isaac_ros2_messages::srv::GetPrimAttribute_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<isaac_ros2_messages::srv::GetPrimAttribute_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__isaac_ros2_messages__srv__GetPrimAttribute_Request
    std::shared_ptr<isaac_ros2_messages::srv::GetPrimAttribute_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__isaac_ros2_messages__srv__GetPrimAttribute_Request
    std::shared_ptr<isaac_ros2_messages::srv::GetPrimAttribute_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetPrimAttribute_Request_ & other) const
  {
    if (this->path != other.path) {
      return false;
    }
    if (this->attribute != other.attribute) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetPrimAttribute_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetPrimAttribute_Request_

// alias to use template instance with default allocator
using GetPrimAttribute_Request =
  isaac_ros2_messages::srv::GetPrimAttribute_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace isaac_ros2_messages


#ifndef _WIN32
# define DEPRECATED__isaac_ros2_messages__srv__GetPrimAttribute_Response __attribute__((deprecated))
#else
# define DEPRECATED__isaac_ros2_messages__srv__GetPrimAttribute_Response __declspec(deprecated)
#endif

namespace isaac_ros2_messages
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetPrimAttribute_Response_
{
  using Type = GetPrimAttribute_Response_<ContainerAllocator>;

  explicit GetPrimAttribute_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->value = "";
      this->type = "";
      this->success = false;
      this->message = "";
    }
  }

  explicit GetPrimAttribute_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : value(_alloc),
    type(_alloc),
    message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->value = "";
      this->type = "";
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _value_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _value_type value;
  using _type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _type_type type;
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__value(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->value = _arg;
    return *this;
  }
  Type & set__type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->type = _arg;
    return *this;
  }
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    isaac_ros2_messages::srv::GetPrimAttribute_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const isaac_ros2_messages::srv::GetPrimAttribute_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<isaac_ros2_messages::srv::GetPrimAttribute_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<isaac_ros2_messages::srv::GetPrimAttribute_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      isaac_ros2_messages::srv::GetPrimAttribute_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<isaac_ros2_messages::srv::GetPrimAttribute_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      isaac_ros2_messages::srv::GetPrimAttribute_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<isaac_ros2_messages::srv::GetPrimAttribute_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<isaac_ros2_messages::srv::GetPrimAttribute_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<isaac_ros2_messages::srv::GetPrimAttribute_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__isaac_ros2_messages__srv__GetPrimAttribute_Response
    std::shared_ptr<isaac_ros2_messages::srv::GetPrimAttribute_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__isaac_ros2_messages__srv__GetPrimAttribute_Response
    std::shared_ptr<isaac_ros2_messages::srv::GetPrimAttribute_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetPrimAttribute_Response_ & other) const
  {
    if (this->value != other.value) {
      return false;
    }
    if (this->type != other.type) {
      return false;
    }
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetPrimAttribute_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetPrimAttribute_Response_

// alias to use template instance with default allocator
using GetPrimAttribute_Response =
  isaac_ros2_messages::srv::GetPrimAttribute_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace isaac_ros2_messages

namespace isaac_ros2_messages
{

namespace srv
{

struct GetPrimAttribute
{
  using Request = isaac_ros2_messages::srv::GetPrimAttribute_Request;
  using Response = isaac_ros2_messages::srv::GetPrimAttribute_Response;
};

}  // namespace srv

}  // namespace isaac_ros2_messages

#endif  // ISAAC_ROS2_MESSAGES__SRV__DETAIL__GET_PRIM_ATTRIBUTE__STRUCT_HPP_
