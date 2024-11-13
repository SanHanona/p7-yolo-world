// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from isaac_ros2_messages:srv/IsaacPose.idl
// generated code does not contain a copyright notice

#ifndef ISAAC_ROS2_MESSAGES__SRV__DETAIL__ISAAC_POSE__STRUCT_HPP_
#define ISAAC_ROS2_MESSAGES__SRV__DETAIL__ISAAC_POSE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'poses'
#include "geometry_msgs/msg/detail/pose__struct.hpp"
// Member 'velocities'
#include "geometry_msgs/msg/detail/twist__struct.hpp"
// Member 'scales'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__isaac_ros2_messages__srv__IsaacPose_Request __attribute__((deprecated))
#else
# define DEPRECATED__isaac_ros2_messages__srv__IsaacPose_Request __declspec(deprecated)
#endif

namespace isaac_ros2_messages
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct IsaacPose_Request_
{
  using Type = IsaacPose_Request_<ContainerAllocator>;

  explicit IsaacPose_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit IsaacPose_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _names_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _names_type names;
  using _poses_type =
    std::vector<geometry_msgs::msg::Pose_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Pose_<ContainerAllocator>>>;
  _poses_type poses;
  using _velocities_type =
    std::vector<geometry_msgs::msg::Twist_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Twist_<ContainerAllocator>>>;
  _velocities_type velocities;
  using _scales_type =
    std::vector<geometry_msgs::msg::Vector3_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Vector3_<ContainerAllocator>>>;
  _scales_type scales;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__names(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->names = _arg;
    return *this;
  }
  Type & set__poses(
    const std::vector<geometry_msgs::msg::Pose_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Pose_<ContainerAllocator>>> & _arg)
  {
    this->poses = _arg;
    return *this;
  }
  Type & set__velocities(
    const std::vector<geometry_msgs::msg::Twist_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Twist_<ContainerAllocator>>> & _arg)
  {
    this->velocities = _arg;
    return *this;
  }
  Type & set__scales(
    const std::vector<geometry_msgs::msg::Vector3_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Vector3_<ContainerAllocator>>> & _arg)
  {
    this->scales = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    isaac_ros2_messages::srv::IsaacPose_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const isaac_ros2_messages::srv::IsaacPose_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<isaac_ros2_messages::srv::IsaacPose_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<isaac_ros2_messages::srv::IsaacPose_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      isaac_ros2_messages::srv::IsaacPose_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<isaac_ros2_messages::srv::IsaacPose_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      isaac_ros2_messages::srv::IsaacPose_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<isaac_ros2_messages::srv::IsaacPose_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<isaac_ros2_messages::srv::IsaacPose_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<isaac_ros2_messages::srv::IsaacPose_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__isaac_ros2_messages__srv__IsaacPose_Request
    std::shared_ptr<isaac_ros2_messages::srv::IsaacPose_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__isaac_ros2_messages__srv__IsaacPose_Request
    std::shared_ptr<isaac_ros2_messages::srv::IsaacPose_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const IsaacPose_Request_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->names != other.names) {
      return false;
    }
    if (this->poses != other.poses) {
      return false;
    }
    if (this->velocities != other.velocities) {
      return false;
    }
    if (this->scales != other.scales) {
      return false;
    }
    return true;
  }
  bool operator!=(const IsaacPose_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct IsaacPose_Request_

// alias to use template instance with default allocator
using IsaacPose_Request =
  isaac_ros2_messages::srv::IsaacPose_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace isaac_ros2_messages


#ifndef _WIN32
# define DEPRECATED__isaac_ros2_messages__srv__IsaacPose_Response __attribute__((deprecated))
#else
# define DEPRECATED__isaac_ros2_messages__srv__IsaacPose_Response __declspec(deprecated)
#endif

namespace isaac_ros2_messages
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct IsaacPose_Response_
{
  using Type = IsaacPose_Response_<ContainerAllocator>;

  explicit IsaacPose_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit IsaacPose_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    isaac_ros2_messages::srv::IsaacPose_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const isaac_ros2_messages::srv::IsaacPose_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<isaac_ros2_messages::srv::IsaacPose_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<isaac_ros2_messages::srv::IsaacPose_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      isaac_ros2_messages::srv::IsaacPose_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<isaac_ros2_messages::srv::IsaacPose_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      isaac_ros2_messages::srv::IsaacPose_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<isaac_ros2_messages::srv::IsaacPose_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<isaac_ros2_messages::srv::IsaacPose_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<isaac_ros2_messages::srv::IsaacPose_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__isaac_ros2_messages__srv__IsaacPose_Response
    std::shared_ptr<isaac_ros2_messages::srv::IsaacPose_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__isaac_ros2_messages__srv__IsaacPose_Response
    std::shared_ptr<isaac_ros2_messages::srv::IsaacPose_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const IsaacPose_Response_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const IsaacPose_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct IsaacPose_Response_

// alias to use template instance with default allocator
using IsaacPose_Response =
  isaac_ros2_messages::srv::IsaacPose_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace isaac_ros2_messages

namespace isaac_ros2_messages
{

namespace srv
{

struct IsaacPose
{
  using Request = isaac_ros2_messages::srv::IsaacPose_Request;
  using Response = isaac_ros2_messages::srv::IsaacPose_Response;
};

}  // namespace srv

}  // namespace isaac_ros2_messages

#endif  // ISAAC_ROS2_MESSAGES__SRV__DETAIL__ISAAC_POSE__STRUCT_HPP_
