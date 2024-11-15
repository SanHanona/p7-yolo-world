// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_message:msg/SampleMsg.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MESSAGE__MSG__DETAIL__SAMPLE_MSG__STRUCT_H_
#define CUSTOM_MESSAGE__MSG__DETAIL__SAMPLE_MSG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'my_string'
#include "std_msgs/msg/detail/string__struct.h"

/// Struct defined in msg/SampleMsg in the package custom_message.
typedef struct custom_message__msg__SampleMsg
{
  std_msgs__msg__String my_string;
  int64_t my_num;
} custom_message__msg__SampleMsg;

// Struct for a sequence of custom_message__msg__SampleMsg.
typedef struct custom_message__msg__SampleMsg__Sequence
{
  custom_message__msg__SampleMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_message__msg__SampleMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MESSAGE__MSG__DETAIL__SAMPLE_MSG__STRUCT_H_
