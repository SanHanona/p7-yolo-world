// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from custom_message:msg/SampleMsg.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "custom_message/msg/detail/sample_msg__rosidl_typesupport_introspection_c.h"
#include "custom_message/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "custom_message/msg/detail/sample_msg__functions.h"
#include "custom_message/msg/detail/sample_msg__struct.h"


// Include directives for member types
// Member `my_string`
#include "std_msgs/msg/string.h"
// Member `my_string`
#include "std_msgs/msg/detail/string__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void custom_message__msg__SampleMsg__rosidl_typesupport_introspection_c__SampleMsg_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  custom_message__msg__SampleMsg__init(message_memory);
}

void custom_message__msg__SampleMsg__rosidl_typesupport_introspection_c__SampleMsg_fini_function(void * message_memory)
{
  custom_message__msg__SampleMsg__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember custom_message__msg__SampleMsg__rosidl_typesupport_introspection_c__SampleMsg_message_member_array[2] = {
  {
    "my_string",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_message__msg__SampleMsg, my_string),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "my_num",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_message__msg__SampleMsg, my_num),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers custom_message__msg__SampleMsg__rosidl_typesupport_introspection_c__SampleMsg_message_members = {
  "custom_message__msg",  // message namespace
  "SampleMsg",  // message name
  2,  // number of fields
  sizeof(custom_message__msg__SampleMsg),
  custom_message__msg__SampleMsg__rosidl_typesupport_introspection_c__SampleMsg_message_member_array,  // message members
  custom_message__msg__SampleMsg__rosidl_typesupport_introspection_c__SampleMsg_init_function,  // function to initialize message memory (memory has to be allocated)
  custom_message__msg__SampleMsg__rosidl_typesupport_introspection_c__SampleMsg_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t custom_message__msg__SampleMsg__rosidl_typesupport_introspection_c__SampleMsg_message_type_support_handle = {
  0,
  &custom_message__msg__SampleMsg__rosidl_typesupport_introspection_c__SampleMsg_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_custom_message
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_message, msg, SampleMsg)() {
  custom_message__msg__SampleMsg__rosidl_typesupport_introspection_c__SampleMsg_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, String)();
  if (!custom_message__msg__SampleMsg__rosidl_typesupport_introspection_c__SampleMsg_message_type_support_handle.typesupport_identifier) {
    custom_message__msg__SampleMsg__rosidl_typesupport_introspection_c__SampleMsg_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &custom_message__msg__SampleMsg__rosidl_typesupport_introspection_c__SampleMsg_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
