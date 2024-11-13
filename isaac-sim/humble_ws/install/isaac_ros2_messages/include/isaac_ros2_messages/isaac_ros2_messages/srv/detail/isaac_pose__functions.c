// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from isaac_ros2_messages:srv/IsaacPose.idl
// generated code does not contain a copyright notice
#include "isaac_ros2_messages/srv/detail/isaac_pose__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `names`
#include "rosidl_runtime_c/string_functions.h"
// Member `poses`
#include "geometry_msgs/msg/detail/pose__functions.h"
// Member `velocities`
#include "geometry_msgs/msg/detail/twist__functions.h"
// Member `scales`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
isaac_ros2_messages__srv__IsaacPose_Request__init(isaac_ros2_messages__srv__IsaacPose_Request * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    isaac_ros2_messages__srv__IsaacPose_Request__fini(msg);
    return false;
  }
  // names
  if (!rosidl_runtime_c__String__Sequence__init(&msg->names, 0)) {
    isaac_ros2_messages__srv__IsaacPose_Request__fini(msg);
    return false;
  }
  // poses
  if (!geometry_msgs__msg__Pose__Sequence__init(&msg->poses, 0)) {
    isaac_ros2_messages__srv__IsaacPose_Request__fini(msg);
    return false;
  }
  // velocities
  if (!geometry_msgs__msg__Twist__Sequence__init(&msg->velocities, 0)) {
    isaac_ros2_messages__srv__IsaacPose_Request__fini(msg);
    return false;
  }
  // scales
  if (!geometry_msgs__msg__Vector3__Sequence__init(&msg->scales, 0)) {
    isaac_ros2_messages__srv__IsaacPose_Request__fini(msg);
    return false;
  }
  return true;
}

void
isaac_ros2_messages__srv__IsaacPose_Request__fini(isaac_ros2_messages__srv__IsaacPose_Request * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // names
  rosidl_runtime_c__String__Sequence__fini(&msg->names);
  // poses
  geometry_msgs__msg__Pose__Sequence__fini(&msg->poses);
  // velocities
  geometry_msgs__msg__Twist__Sequence__fini(&msg->velocities);
  // scales
  geometry_msgs__msg__Vector3__Sequence__fini(&msg->scales);
}

bool
isaac_ros2_messages__srv__IsaacPose_Request__are_equal(const isaac_ros2_messages__srv__IsaacPose_Request * lhs, const isaac_ros2_messages__srv__IsaacPose_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // names
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->names), &(rhs->names)))
  {
    return false;
  }
  // poses
  if (!geometry_msgs__msg__Pose__Sequence__are_equal(
      &(lhs->poses), &(rhs->poses)))
  {
    return false;
  }
  // velocities
  if (!geometry_msgs__msg__Twist__Sequence__are_equal(
      &(lhs->velocities), &(rhs->velocities)))
  {
    return false;
  }
  // scales
  if (!geometry_msgs__msg__Vector3__Sequence__are_equal(
      &(lhs->scales), &(rhs->scales)))
  {
    return false;
  }
  return true;
}

bool
isaac_ros2_messages__srv__IsaacPose_Request__copy(
  const isaac_ros2_messages__srv__IsaacPose_Request * input,
  isaac_ros2_messages__srv__IsaacPose_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // names
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->names), &(output->names)))
  {
    return false;
  }
  // poses
  if (!geometry_msgs__msg__Pose__Sequence__copy(
      &(input->poses), &(output->poses)))
  {
    return false;
  }
  // velocities
  if (!geometry_msgs__msg__Twist__Sequence__copy(
      &(input->velocities), &(output->velocities)))
  {
    return false;
  }
  // scales
  if (!geometry_msgs__msg__Vector3__Sequence__copy(
      &(input->scales), &(output->scales)))
  {
    return false;
  }
  return true;
}

isaac_ros2_messages__srv__IsaacPose_Request *
isaac_ros2_messages__srv__IsaacPose_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  isaac_ros2_messages__srv__IsaacPose_Request * msg = (isaac_ros2_messages__srv__IsaacPose_Request *)allocator.allocate(sizeof(isaac_ros2_messages__srv__IsaacPose_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(isaac_ros2_messages__srv__IsaacPose_Request));
  bool success = isaac_ros2_messages__srv__IsaacPose_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
isaac_ros2_messages__srv__IsaacPose_Request__destroy(isaac_ros2_messages__srv__IsaacPose_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    isaac_ros2_messages__srv__IsaacPose_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
isaac_ros2_messages__srv__IsaacPose_Request__Sequence__init(isaac_ros2_messages__srv__IsaacPose_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  isaac_ros2_messages__srv__IsaacPose_Request * data = NULL;

  if (size) {
    data = (isaac_ros2_messages__srv__IsaacPose_Request *)allocator.zero_allocate(size, sizeof(isaac_ros2_messages__srv__IsaacPose_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = isaac_ros2_messages__srv__IsaacPose_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        isaac_ros2_messages__srv__IsaacPose_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
isaac_ros2_messages__srv__IsaacPose_Request__Sequence__fini(isaac_ros2_messages__srv__IsaacPose_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      isaac_ros2_messages__srv__IsaacPose_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

isaac_ros2_messages__srv__IsaacPose_Request__Sequence *
isaac_ros2_messages__srv__IsaacPose_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  isaac_ros2_messages__srv__IsaacPose_Request__Sequence * array = (isaac_ros2_messages__srv__IsaacPose_Request__Sequence *)allocator.allocate(sizeof(isaac_ros2_messages__srv__IsaacPose_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = isaac_ros2_messages__srv__IsaacPose_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
isaac_ros2_messages__srv__IsaacPose_Request__Sequence__destroy(isaac_ros2_messages__srv__IsaacPose_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    isaac_ros2_messages__srv__IsaacPose_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
isaac_ros2_messages__srv__IsaacPose_Request__Sequence__are_equal(const isaac_ros2_messages__srv__IsaacPose_Request__Sequence * lhs, const isaac_ros2_messages__srv__IsaacPose_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!isaac_ros2_messages__srv__IsaacPose_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
isaac_ros2_messages__srv__IsaacPose_Request__Sequence__copy(
  const isaac_ros2_messages__srv__IsaacPose_Request__Sequence * input,
  isaac_ros2_messages__srv__IsaacPose_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(isaac_ros2_messages__srv__IsaacPose_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    isaac_ros2_messages__srv__IsaacPose_Request * data =
      (isaac_ros2_messages__srv__IsaacPose_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!isaac_ros2_messages__srv__IsaacPose_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          isaac_ros2_messages__srv__IsaacPose_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!isaac_ros2_messages__srv__IsaacPose_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
isaac_ros2_messages__srv__IsaacPose_Response__init(isaac_ros2_messages__srv__IsaacPose_Response * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
isaac_ros2_messages__srv__IsaacPose_Response__fini(isaac_ros2_messages__srv__IsaacPose_Response * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
isaac_ros2_messages__srv__IsaacPose_Response__are_equal(const isaac_ros2_messages__srv__IsaacPose_Response * lhs, const isaac_ros2_messages__srv__IsaacPose_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
isaac_ros2_messages__srv__IsaacPose_Response__copy(
  const isaac_ros2_messages__srv__IsaacPose_Response * input,
  isaac_ros2_messages__srv__IsaacPose_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

isaac_ros2_messages__srv__IsaacPose_Response *
isaac_ros2_messages__srv__IsaacPose_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  isaac_ros2_messages__srv__IsaacPose_Response * msg = (isaac_ros2_messages__srv__IsaacPose_Response *)allocator.allocate(sizeof(isaac_ros2_messages__srv__IsaacPose_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(isaac_ros2_messages__srv__IsaacPose_Response));
  bool success = isaac_ros2_messages__srv__IsaacPose_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
isaac_ros2_messages__srv__IsaacPose_Response__destroy(isaac_ros2_messages__srv__IsaacPose_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    isaac_ros2_messages__srv__IsaacPose_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
isaac_ros2_messages__srv__IsaacPose_Response__Sequence__init(isaac_ros2_messages__srv__IsaacPose_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  isaac_ros2_messages__srv__IsaacPose_Response * data = NULL;

  if (size) {
    data = (isaac_ros2_messages__srv__IsaacPose_Response *)allocator.zero_allocate(size, sizeof(isaac_ros2_messages__srv__IsaacPose_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = isaac_ros2_messages__srv__IsaacPose_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        isaac_ros2_messages__srv__IsaacPose_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
isaac_ros2_messages__srv__IsaacPose_Response__Sequence__fini(isaac_ros2_messages__srv__IsaacPose_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      isaac_ros2_messages__srv__IsaacPose_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

isaac_ros2_messages__srv__IsaacPose_Response__Sequence *
isaac_ros2_messages__srv__IsaacPose_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  isaac_ros2_messages__srv__IsaacPose_Response__Sequence * array = (isaac_ros2_messages__srv__IsaacPose_Response__Sequence *)allocator.allocate(sizeof(isaac_ros2_messages__srv__IsaacPose_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = isaac_ros2_messages__srv__IsaacPose_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
isaac_ros2_messages__srv__IsaacPose_Response__Sequence__destroy(isaac_ros2_messages__srv__IsaacPose_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    isaac_ros2_messages__srv__IsaacPose_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
isaac_ros2_messages__srv__IsaacPose_Response__Sequence__are_equal(const isaac_ros2_messages__srv__IsaacPose_Response__Sequence * lhs, const isaac_ros2_messages__srv__IsaacPose_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!isaac_ros2_messages__srv__IsaacPose_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
isaac_ros2_messages__srv__IsaacPose_Response__Sequence__copy(
  const isaac_ros2_messages__srv__IsaacPose_Response__Sequence * input,
  isaac_ros2_messages__srv__IsaacPose_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(isaac_ros2_messages__srv__IsaacPose_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    isaac_ros2_messages__srv__IsaacPose_Response * data =
      (isaac_ros2_messages__srv__IsaacPose_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!isaac_ros2_messages__srv__IsaacPose_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          isaac_ros2_messages__srv__IsaacPose_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!isaac_ros2_messages__srv__IsaacPose_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
