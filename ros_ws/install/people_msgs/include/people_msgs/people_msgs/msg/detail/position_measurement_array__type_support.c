// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from people_msgs:msg/PositionMeasurementArray.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "people_msgs/msg/detail/position_measurement_array__rosidl_typesupport_introspection_c.h"
#include "people_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "people_msgs/msg/detail/position_measurement_array__functions.h"
#include "people_msgs/msg/detail/position_measurement_array__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `people`
#include "people_msgs/msg/position_measurement.h"
// Member `people`
#include "people_msgs/msg/detail/position_measurement__rosidl_typesupport_introspection_c.h"
// Member `cooccurrence`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void people_msgs__msg__PositionMeasurementArray__rosidl_typesupport_introspection_c__PositionMeasurementArray_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  people_msgs__msg__PositionMeasurementArray__init(message_memory);
}

void people_msgs__msg__PositionMeasurementArray__rosidl_typesupport_introspection_c__PositionMeasurementArray_fini_function(void * message_memory)
{
  people_msgs__msg__PositionMeasurementArray__fini(message_memory);
}

size_t people_msgs__msg__PositionMeasurementArray__rosidl_typesupport_introspection_c__size_function__PositionMeasurementArray__people(
  const void * untyped_member)
{
  const people_msgs__msg__PositionMeasurement__Sequence * member =
    (const people_msgs__msg__PositionMeasurement__Sequence *)(untyped_member);
  return member->size;
}

const void * people_msgs__msg__PositionMeasurementArray__rosidl_typesupport_introspection_c__get_const_function__PositionMeasurementArray__people(
  const void * untyped_member, size_t index)
{
  const people_msgs__msg__PositionMeasurement__Sequence * member =
    (const people_msgs__msg__PositionMeasurement__Sequence *)(untyped_member);
  return &member->data[index];
}

void * people_msgs__msg__PositionMeasurementArray__rosidl_typesupport_introspection_c__get_function__PositionMeasurementArray__people(
  void * untyped_member, size_t index)
{
  people_msgs__msg__PositionMeasurement__Sequence * member =
    (people_msgs__msg__PositionMeasurement__Sequence *)(untyped_member);
  return &member->data[index];
}

void people_msgs__msg__PositionMeasurementArray__rosidl_typesupport_introspection_c__fetch_function__PositionMeasurementArray__people(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const people_msgs__msg__PositionMeasurement * item =
    ((const people_msgs__msg__PositionMeasurement *)
    people_msgs__msg__PositionMeasurementArray__rosidl_typesupport_introspection_c__get_const_function__PositionMeasurementArray__people(untyped_member, index));
  people_msgs__msg__PositionMeasurement * value =
    (people_msgs__msg__PositionMeasurement *)(untyped_value);
  *value = *item;
}

void people_msgs__msg__PositionMeasurementArray__rosidl_typesupport_introspection_c__assign_function__PositionMeasurementArray__people(
  void * untyped_member, size_t index, const void * untyped_value)
{
  people_msgs__msg__PositionMeasurement * item =
    ((people_msgs__msg__PositionMeasurement *)
    people_msgs__msg__PositionMeasurementArray__rosidl_typesupport_introspection_c__get_function__PositionMeasurementArray__people(untyped_member, index));
  const people_msgs__msg__PositionMeasurement * value =
    (const people_msgs__msg__PositionMeasurement *)(untyped_value);
  *item = *value;
}

bool people_msgs__msg__PositionMeasurementArray__rosidl_typesupport_introspection_c__resize_function__PositionMeasurementArray__people(
  void * untyped_member, size_t size)
{
  people_msgs__msg__PositionMeasurement__Sequence * member =
    (people_msgs__msg__PositionMeasurement__Sequence *)(untyped_member);
  people_msgs__msg__PositionMeasurement__Sequence__fini(member);
  return people_msgs__msg__PositionMeasurement__Sequence__init(member, size);
}

size_t people_msgs__msg__PositionMeasurementArray__rosidl_typesupport_introspection_c__size_function__PositionMeasurementArray__cooccurrence(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * people_msgs__msg__PositionMeasurementArray__rosidl_typesupport_introspection_c__get_const_function__PositionMeasurementArray__cooccurrence(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * people_msgs__msg__PositionMeasurementArray__rosidl_typesupport_introspection_c__get_function__PositionMeasurementArray__cooccurrence(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void people_msgs__msg__PositionMeasurementArray__rosidl_typesupport_introspection_c__fetch_function__PositionMeasurementArray__cooccurrence(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    people_msgs__msg__PositionMeasurementArray__rosidl_typesupport_introspection_c__get_const_function__PositionMeasurementArray__cooccurrence(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void people_msgs__msg__PositionMeasurementArray__rosidl_typesupport_introspection_c__assign_function__PositionMeasurementArray__cooccurrence(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    people_msgs__msg__PositionMeasurementArray__rosidl_typesupport_introspection_c__get_function__PositionMeasurementArray__cooccurrence(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool people_msgs__msg__PositionMeasurementArray__rosidl_typesupport_introspection_c__resize_function__PositionMeasurementArray__cooccurrence(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember people_msgs__msg__PositionMeasurementArray__rosidl_typesupport_introspection_c__PositionMeasurementArray_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(people_msgs__msg__PositionMeasurementArray, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "people",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(people_msgs__msg__PositionMeasurementArray, people),  // bytes offset in struct
    NULL,  // default value
    people_msgs__msg__PositionMeasurementArray__rosidl_typesupport_introspection_c__size_function__PositionMeasurementArray__people,  // size() function pointer
    people_msgs__msg__PositionMeasurementArray__rosidl_typesupport_introspection_c__get_const_function__PositionMeasurementArray__people,  // get_const(index) function pointer
    people_msgs__msg__PositionMeasurementArray__rosidl_typesupport_introspection_c__get_function__PositionMeasurementArray__people,  // get(index) function pointer
    people_msgs__msg__PositionMeasurementArray__rosidl_typesupport_introspection_c__fetch_function__PositionMeasurementArray__people,  // fetch(index, &value) function pointer
    people_msgs__msg__PositionMeasurementArray__rosidl_typesupport_introspection_c__assign_function__PositionMeasurementArray__people,  // assign(index, value) function pointer
    people_msgs__msg__PositionMeasurementArray__rosidl_typesupport_introspection_c__resize_function__PositionMeasurementArray__people  // resize(index) function pointer
  },
  {
    "cooccurrence",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(people_msgs__msg__PositionMeasurementArray, cooccurrence),  // bytes offset in struct
    NULL,  // default value
    people_msgs__msg__PositionMeasurementArray__rosidl_typesupport_introspection_c__size_function__PositionMeasurementArray__cooccurrence,  // size() function pointer
    people_msgs__msg__PositionMeasurementArray__rosidl_typesupport_introspection_c__get_const_function__PositionMeasurementArray__cooccurrence,  // get_const(index) function pointer
    people_msgs__msg__PositionMeasurementArray__rosidl_typesupport_introspection_c__get_function__PositionMeasurementArray__cooccurrence,  // get(index) function pointer
    people_msgs__msg__PositionMeasurementArray__rosidl_typesupport_introspection_c__fetch_function__PositionMeasurementArray__cooccurrence,  // fetch(index, &value) function pointer
    people_msgs__msg__PositionMeasurementArray__rosidl_typesupport_introspection_c__assign_function__PositionMeasurementArray__cooccurrence,  // assign(index, value) function pointer
    people_msgs__msg__PositionMeasurementArray__rosidl_typesupport_introspection_c__resize_function__PositionMeasurementArray__cooccurrence  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers people_msgs__msg__PositionMeasurementArray__rosidl_typesupport_introspection_c__PositionMeasurementArray_message_members = {
  "people_msgs__msg",  // message namespace
  "PositionMeasurementArray",  // message name
  3,  // number of fields
  sizeof(people_msgs__msg__PositionMeasurementArray),
  people_msgs__msg__PositionMeasurementArray__rosidl_typesupport_introspection_c__PositionMeasurementArray_message_member_array,  // message members
  people_msgs__msg__PositionMeasurementArray__rosidl_typesupport_introspection_c__PositionMeasurementArray_init_function,  // function to initialize message memory (memory has to be allocated)
  people_msgs__msg__PositionMeasurementArray__rosidl_typesupport_introspection_c__PositionMeasurementArray_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t people_msgs__msg__PositionMeasurementArray__rosidl_typesupport_introspection_c__PositionMeasurementArray_message_type_support_handle = {
  0,
  &people_msgs__msg__PositionMeasurementArray__rosidl_typesupport_introspection_c__PositionMeasurementArray_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_people_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, people_msgs, msg, PositionMeasurementArray)() {
  people_msgs__msg__PositionMeasurementArray__rosidl_typesupport_introspection_c__PositionMeasurementArray_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  people_msgs__msg__PositionMeasurementArray__rosidl_typesupport_introspection_c__PositionMeasurementArray_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, people_msgs, msg, PositionMeasurement)();
  if (!people_msgs__msg__PositionMeasurementArray__rosidl_typesupport_introspection_c__PositionMeasurementArray_message_type_support_handle.typesupport_identifier) {
    people_msgs__msg__PositionMeasurementArray__rosidl_typesupport_introspection_c__PositionMeasurementArray_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &people_msgs__msg__PositionMeasurementArray__rosidl_typesupport_introspection_c__PositionMeasurementArray_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
