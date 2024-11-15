// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from isaac_ros2_messages:srv/IsaacPose.idl
// generated code does not contain a copyright notice

#ifndef ISAAC_ROS2_MESSAGES__SRV__DETAIL__ISAAC_POSE__BUILDER_HPP_
#define ISAAC_ROS2_MESSAGES__SRV__DETAIL__ISAAC_POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "isaac_ros2_messages/srv/detail/isaac_pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace isaac_ros2_messages
{

namespace srv
{

namespace builder
{

class Init_IsaacPose_Request_scales
{
public:
  explicit Init_IsaacPose_Request_scales(::isaac_ros2_messages::srv::IsaacPose_Request & msg)
  : msg_(msg)
  {}
  ::isaac_ros2_messages::srv::IsaacPose_Request scales(::isaac_ros2_messages::srv::IsaacPose_Request::_scales_type arg)
  {
    msg_.scales = std::move(arg);
    return std::move(msg_);
  }

private:
  ::isaac_ros2_messages::srv::IsaacPose_Request msg_;
};

class Init_IsaacPose_Request_velocities
{
public:
  explicit Init_IsaacPose_Request_velocities(::isaac_ros2_messages::srv::IsaacPose_Request & msg)
  : msg_(msg)
  {}
  Init_IsaacPose_Request_scales velocities(::isaac_ros2_messages::srv::IsaacPose_Request::_velocities_type arg)
  {
    msg_.velocities = std::move(arg);
    return Init_IsaacPose_Request_scales(msg_);
  }

private:
  ::isaac_ros2_messages::srv::IsaacPose_Request msg_;
};

class Init_IsaacPose_Request_poses
{
public:
  explicit Init_IsaacPose_Request_poses(::isaac_ros2_messages::srv::IsaacPose_Request & msg)
  : msg_(msg)
  {}
  Init_IsaacPose_Request_velocities poses(::isaac_ros2_messages::srv::IsaacPose_Request::_poses_type arg)
  {
    msg_.poses = std::move(arg);
    return Init_IsaacPose_Request_velocities(msg_);
  }

private:
  ::isaac_ros2_messages::srv::IsaacPose_Request msg_;
};

class Init_IsaacPose_Request_names
{
public:
  explicit Init_IsaacPose_Request_names(::isaac_ros2_messages::srv::IsaacPose_Request & msg)
  : msg_(msg)
  {}
  Init_IsaacPose_Request_poses names(::isaac_ros2_messages::srv::IsaacPose_Request::_names_type arg)
  {
    msg_.names = std::move(arg);
    return Init_IsaacPose_Request_poses(msg_);
  }

private:
  ::isaac_ros2_messages::srv::IsaacPose_Request msg_;
};

class Init_IsaacPose_Request_header
{
public:
  Init_IsaacPose_Request_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_IsaacPose_Request_names header(::isaac_ros2_messages::srv::IsaacPose_Request::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_IsaacPose_Request_names(msg_);
  }

private:
  ::isaac_ros2_messages::srv::IsaacPose_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::isaac_ros2_messages::srv::IsaacPose_Request>()
{
  return isaac_ros2_messages::srv::builder::Init_IsaacPose_Request_header();
}

}  // namespace isaac_ros2_messages


namespace isaac_ros2_messages
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::isaac_ros2_messages::srv::IsaacPose_Response>()
{
  return ::isaac_ros2_messages::srv::IsaacPose_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace isaac_ros2_messages

#endif  // ISAAC_ROS2_MESSAGES__SRV__DETAIL__ISAAC_POSE__BUILDER_HPP_
