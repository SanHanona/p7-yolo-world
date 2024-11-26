// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from people_msgs:msg/PositionMeasurementArray.idl
// generated code does not contain a copyright notice
#include "people_msgs/msg/detail/position_measurement_array__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `people`
#include "people_msgs/msg/detail/position_measurement__functions.h"
// Member `cooccurrence`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
people_msgs__msg__PositionMeasurementArray__init(people_msgs__msg__PositionMeasurementArray * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    people_msgs__msg__PositionMeasurementArray__fini(msg);
    return false;
  }
  // people
  if (!people_msgs__msg__PositionMeasurement__Sequence__init(&msg->people, 0)) {
    people_msgs__msg__PositionMeasurementArray__fini(msg);
    return false;
  }
  // cooccurrence
  if (!rosidl_runtime_c__float__Sequence__init(&msg->cooccurrence, 0)) {
    people_msgs__msg__PositionMeasurementArray__fini(msg);
    return false;
  }
  return true;
}

void
people_msgs__msg__PositionMeasurementArray__fini(people_msgs__msg__PositionMeasurementArray * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // people
  people_msgs__msg__PositionMeasurement__Sequence__fini(&msg->people);
  // cooccurrence
  rosidl_runtime_c__float__Sequence__fini(&msg->cooccurrence);
}

bool
people_msgs__msg__PositionMeasurementArray__are_equal(const people_msgs__msg__PositionMeasurementArray * lhs, const people_msgs__msg__PositionMeasurementArray * rhs)
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
  // people
  if (!people_msgs__msg__PositionMeasurement__Sequence__are_equal(
      &(lhs->people), &(rhs->people)))
  {
    return false;
  }
  // cooccurrence
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->cooccurrence), &(rhs->cooccurrence)))
  {
    return false;
  }
  return true;
}

bool
people_msgs__msg__PositionMeasurementArray__copy(
  const people_msgs__msg__PositionMeasurementArray * input,
  people_msgs__msg__PositionMeasurementArray * output)
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
  // people
  if (!people_msgs__msg__PositionMeasurement__Sequence__copy(
      &(input->people), &(output->people)))
  {
    return false;
  }
  // cooccurrence
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->cooccurrence), &(output->cooccurrence)))
  {
    return false;
  }
  return true;
}

people_msgs__msg__PositionMeasurementArray *
people_msgs__msg__PositionMeasurementArray__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  people_msgs__msg__PositionMeasurementArray * msg = (people_msgs__msg__PositionMeasurementArray *)allocator.allocate(sizeof(people_msgs__msg__PositionMeasurementArray), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(people_msgs__msg__PositionMeasurementArray));
  bool success = people_msgs__msg__PositionMeasurementArray__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
people_msgs__msg__PositionMeasurementArray__destroy(people_msgs__msg__PositionMeasurementArray * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    people_msgs__msg__PositionMeasurementArray__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
people_msgs__msg__PositionMeasurementArray__Sequence__init(people_msgs__msg__PositionMeasurementArray__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  people_msgs__msg__PositionMeasurementArray * data = NULL;

  if (size) {
    data = (people_msgs__msg__PositionMeasurementArray *)allocator.zero_allocate(size, sizeof(people_msgs__msg__PositionMeasurementArray), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = people_msgs__msg__PositionMeasurementArray__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        people_msgs__msg__PositionMeasurementArray__fini(&data[i - 1]);
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
people_msgs__msg__PositionMeasurementArray__Sequence__fini(people_msgs__msg__PositionMeasurementArray__Sequence * array)
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
      people_msgs__msg__PositionMeasurementArray__fini(&array->data[i]);
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

people_msgs__msg__PositionMeasurementArray__Sequence *
people_msgs__msg__PositionMeasurementArray__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  people_msgs__msg__PositionMeasurementArray__Sequence * array = (people_msgs__msg__PositionMeasurementArray__Sequence *)allocator.allocate(sizeof(people_msgs__msg__PositionMeasurementArray__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = people_msgs__msg__PositionMeasurementArray__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
people_msgs__msg__PositionMeasurementArray__Sequence__destroy(people_msgs__msg__PositionMeasurementArray__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    people_msgs__msg__PositionMeasurementArray__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
people_msgs__msg__PositionMeasurementArray__Sequence__are_equal(const people_msgs__msg__PositionMeasurementArray__Sequence * lhs, const people_msgs__msg__PositionMeasurementArray__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!people_msgs__msg__PositionMeasurementArray__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
people_msgs__msg__PositionMeasurementArray__Sequence__copy(
  const people_msgs__msg__PositionMeasurementArray__Sequence * input,
  people_msgs__msg__PositionMeasurementArray__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(people_msgs__msg__PositionMeasurementArray);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    people_msgs__msg__PositionMeasurementArray * data =
      (people_msgs__msg__PositionMeasurementArray *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!people_msgs__msg__PositionMeasurementArray__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          people_msgs__msg__PositionMeasurementArray__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!people_msgs__msg__PositionMeasurementArray__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
