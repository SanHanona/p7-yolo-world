// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from isaac_ros2_messages:srv/GetPrimAttribute.idl
// generated code does not contain a copyright notice

#ifndef ISAAC_ROS2_MESSAGES__SRV__DETAIL__GET_PRIM_ATTRIBUTE__STRUCT_H_
#define ISAAC_ROS2_MESSAGES__SRV__DETAIL__GET_PRIM_ATTRIBUTE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'path'
// Member 'attribute'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GetPrimAttribute in the package isaac_ros2_messages.
typedef struct isaac_ros2_messages__srv__GetPrimAttribute_Request
{
  /// prim path
  rosidl_runtime_c__String path;
  /// attribute name
  rosidl_runtime_c__String attribute;
} isaac_ros2_messages__srv__GetPrimAttribute_Request;

// Struct for a sequence of isaac_ros2_messages__srv__GetPrimAttribute_Request.
typedef struct isaac_ros2_messages__srv__GetPrimAttribute_Request__Sequence
{
  isaac_ros2_messages__srv__GetPrimAttribute_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} isaac_ros2_messages__srv__GetPrimAttribute_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'value'
// Member 'type'
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GetPrimAttribute in the package isaac_ros2_messages.
typedef struct isaac_ros2_messages__srv__GetPrimAttribute_Response
{
  /// attribute value (as JSON)
  rosidl_runtime_c__String value;
  /// attribute type
  rosidl_runtime_c__String type;
  /// indicate a successful execution of the service
  bool success;
  /// informational, e.g. for error messages
  rosidl_runtime_c__String message;
} isaac_ros2_messages__srv__GetPrimAttribute_Response;

// Struct for a sequence of isaac_ros2_messages__srv__GetPrimAttribute_Response.
typedef struct isaac_ros2_messages__srv__GetPrimAttribute_Response__Sequence
{
  isaac_ros2_messages__srv__GetPrimAttribute_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} isaac_ros2_messages__srv__GetPrimAttribute_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ISAAC_ROS2_MESSAGES__SRV__DETAIL__GET_PRIM_ATTRIBUTE__STRUCT_H_
