// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from isaac_ros2_messages:srv/IsaacPose.idl
// generated code does not contain a copyright notice

#ifndef ISAAC_ROS2_MESSAGES__SRV__DETAIL__ISAAC_POSE__STRUCT_H_
#define ISAAC_ROS2_MESSAGES__SRV__DETAIL__ISAAC_POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'names'
#include "rosidl_runtime_c/string.h"
// Member 'poses'
#include "geometry_msgs/msg/detail/pose__struct.h"
// Member 'velocities'
#include "geometry_msgs/msg/detail/twist__struct.h"
// Member 'scales'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in srv/IsaacPose in the package isaac_ros2_messages.
typedef struct isaac_ros2_messages__srv__IsaacPose_Request
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String__Sequence names;
  geometry_msgs__msg__Pose__Sequence poses;
  geometry_msgs__msg__Twist__Sequence velocities;
  geometry_msgs__msg__Vector3__Sequence scales;
} isaac_ros2_messages__srv__IsaacPose_Request;

// Struct for a sequence of isaac_ros2_messages__srv__IsaacPose_Request.
typedef struct isaac_ros2_messages__srv__IsaacPose_Request__Sequence
{
  isaac_ros2_messages__srv__IsaacPose_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} isaac_ros2_messages__srv__IsaacPose_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/IsaacPose in the package isaac_ros2_messages.
typedef struct isaac_ros2_messages__srv__IsaacPose_Response
{
  uint8_t structure_needs_at_least_one_member;
} isaac_ros2_messages__srv__IsaacPose_Response;

// Struct for a sequence of isaac_ros2_messages__srv__IsaacPose_Response.
typedef struct isaac_ros2_messages__srv__IsaacPose_Response__Sequence
{
  isaac_ros2_messages__srv__IsaacPose_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} isaac_ros2_messages__srv__IsaacPose_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ISAAC_ROS2_MESSAGES__SRV__DETAIL__ISAAC_POSE__STRUCT_H_
