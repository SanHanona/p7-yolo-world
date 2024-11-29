// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from people_msgs:msg/Person.idl
// generated code does not contain a copyright notice
#include "people_msgs/msg/detail/person__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `name`
// Member `tagnames`
// Member `tags`
#include "rosidl_runtime_c/string_functions.h"
// Member `position`
// Member `velocity`
#include "geometry_msgs/msg/detail/point__functions.h"

bool
people_msgs__msg__Person__init(people_msgs__msg__Person * msg)
{
  if (!msg) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__init(&msg->name)) {
    people_msgs__msg__Person__fini(msg);
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__init(&msg->position)) {
    people_msgs__msg__Person__fini(msg);
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Point__init(&msg->velocity)) {
    people_msgs__msg__Person__fini(msg);
    return false;
  }
  // reliability
  // tagnames
  if (!rosidl_runtime_c__String__Sequence__init(&msg->tagnames, 0)) {
    people_msgs__msg__Person__fini(msg);
    return false;
  }
  // tags
  if (!rosidl_runtime_c__String__Sequence__init(&msg->tags, 0)) {
    people_msgs__msg__Person__fini(msg);
    return false;
  }
  return true;
}

void
people_msgs__msg__Person__fini(people_msgs__msg__Person * msg)
{
  if (!msg) {
    return;
  }
  // name
  rosidl_runtime_c__String__fini(&msg->name);
  // position
  geometry_msgs__msg__Point__fini(&msg->position);
  // velocity
  geometry_msgs__msg__Point__fini(&msg->velocity);
  // reliability
  // tagnames
  rosidl_runtime_c__String__Sequence__fini(&msg->tagnames);
  // tags
  rosidl_runtime_c__String__Sequence__fini(&msg->tags);
}

bool
people_msgs__msg__Person__are_equal(const people_msgs__msg__Person * lhs, const people_msgs__msg__Person * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->name), &(rhs->name)))
  {
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->position), &(rhs->position)))
  {
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->velocity), &(rhs->velocity)))
  {
    return false;
  }
  // reliability
  if (lhs->reliability != rhs->reliability) {
    return false;
  }
  // tagnames
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->tagnames), &(rhs->tagnames)))
  {
    return false;
  }
  // tags
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->tags), &(rhs->tags)))
  {
    return false;
  }
  return true;
}

bool
people_msgs__msg__Person__copy(
  const people_msgs__msg__Person * input,
  people_msgs__msg__Person * output)
{
  if (!input || !output) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__copy(
      &(input->name), &(output->name)))
  {
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__copy(
      &(input->position), &(output->position)))
  {
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Point__copy(
      &(input->velocity), &(output->velocity)))
  {
    return false;
  }
  // reliability
  output->reliability = input->reliability;
  // tagnames
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->tagnames), &(output->tagnames)))
  {
    return false;
  }
  // tags
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->tags), &(output->tags)))
  {
    return false;
  }
  return true;
}

people_msgs__msg__Person *
people_msgs__msg__Person__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  people_msgs__msg__Person * msg = (people_msgs__msg__Person *)allocator.allocate(sizeof(people_msgs__msg__Person), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(people_msgs__msg__Person));
  bool success = people_msgs__msg__Person__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
people_msgs__msg__Person__destroy(people_msgs__msg__Person * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    people_msgs__msg__Person__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
people_msgs__msg__Person__Sequence__init(people_msgs__msg__Person__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  people_msgs__msg__Person * data = NULL;

  if (size) {
    data = (people_msgs__msg__Person *)allocator.zero_allocate(size, sizeof(people_msgs__msg__Person), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = people_msgs__msg__Person__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        people_msgs__msg__Person__fini(&data[i - 1]);
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
people_msgs__msg__Person__Sequence__fini(people_msgs__msg__Person__Sequence * array)
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
      people_msgs__msg__Person__fini(&array->data[i]);
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

people_msgs__msg__Person__Sequence *
people_msgs__msg__Person__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  people_msgs__msg__Person__Sequence * array = (people_msgs__msg__Person__Sequence *)allocator.allocate(sizeof(people_msgs__msg__Person__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = people_msgs__msg__Person__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
people_msgs__msg__Person__Sequence__destroy(people_msgs__msg__Person__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    people_msgs__msg__Person__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
people_msgs__msg__Person__Sequence__are_equal(const people_msgs__msg__Person__Sequence * lhs, const people_msgs__msg__Person__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!people_msgs__msg__Person__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
people_msgs__msg__Person__Sequence__copy(
  const people_msgs__msg__Person__Sequence * input,
  people_msgs__msg__Person__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(people_msgs__msg__Person);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    people_msgs__msg__Person * data =
      (people_msgs__msg__Person *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!people_msgs__msg__Person__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          people_msgs__msg__Person__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!people_msgs__msg__Person__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
