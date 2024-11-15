// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from custom_message:msg/SampleMsg.idl
// generated code does not contain a copyright notice
#include "custom_message/msg/detail/sample_msg__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `my_string`
#include "std_msgs/msg/detail/string__functions.h"

bool
custom_message__msg__SampleMsg__init(custom_message__msg__SampleMsg * msg)
{
  if (!msg) {
    return false;
  }
  // my_string
  if (!std_msgs__msg__String__init(&msg->my_string)) {
    custom_message__msg__SampleMsg__fini(msg);
    return false;
  }
  // my_num
  return true;
}

void
custom_message__msg__SampleMsg__fini(custom_message__msg__SampleMsg * msg)
{
  if (!msg) {
    return;
  }
  // my_string
  std_msgs__msg__String__fini(&msg->my_string);
  // my_num
}

bool
custom_message__msg__SampleMsg__are_equal(const custom_message__msg__SampleMsg * lhs, const custom_message__msg__SampleMsg * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // my_string
  if (!std_msgs__msg__String__are_equal(
      &(lhs->my_string), &(rhs->my_string)))
  {
    return false;
  }
  // my_num
  if (lhs->my_num != rhs->my_num) {
    return false;
  }
  return true;
}

bool
custom_message__msg__SampleMsg__copy(
  const custom_message__msg__SampleMsg * input,
  custom_message__msg__SampleMsg * output)
{
  if (!input || !output) {
    return false;
  }
  // my_string
  if (!std_msgs__msg__String__copy(
      &(input->my_string), &(output->my_string)))
  {
    return false;
  }
  // my_num
  output->my_num = input->my_num;
  return true;
}

custom_message__msg__SampleMsg *
custom_message__msg__SampleMsg__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_message__msg__SampleMsg * msg = (custom_message__msg__SampleMsg *)allocator.allocate(sizeof(custom_message__msg__SampleMsg), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(custom_message__msg__SampleMsg));
  bool success = custom_message__msg__SampleMsg__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
custom_message__msg__SampleMsg__destroy(custom_message__msg__SampleMsg * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    custom_message__msg__SampleMsg__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
custom_message__msg__SampleMsg__Sequence__init(custom_message__msg__SampleMsg__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_message__msg__SampleMsg * data = NULL;

  if (size) {
    data = (custom_message__msg__SampleMsg *)allocator.zero_allocate(size, sizeof(custom_message__msg__SampleMsg), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = custom_message__msg__SampleMsg__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        custom_message__msg__SampleMsg__fini(&data[i - 1]);
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
custom_message__msg__SampleMsg__Sequence__fini(custom_message__msg__SampleMsg__Sequence * array)
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
      custom_message__msg__SampleMsg__fini(&array->data[i]);
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

custom_message__msg__SampleMsg__Sequence *
custom_message__msg__SampleMsg__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_message__msg__SampleMsg__Sequence * array = (custom_message__msg__SampleMsg__Sequence *)allocator.allocate(sizeof(custom_message__msg__SampleMsg__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = custom_message__msg__SampleMsg__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
custom_message__msg__SampleMsg__Sequence__destroy(custom_message__msg__SampleMsg__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    custom_message__msg__SampleMsg__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
custom_message__msg__SampleMsg__Sequence__are_equal(const custom_message__msg__SampleMsg__Sequence * lhs, const custom_message__msg__SampleMsg__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!custom_message__msg__SampleMsg__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
custom_message__msg__SampleMsg__Sequence__copy(
  const custom_message__msg__SampleMsg__Sequence * input,
  custom_message__msg__SampleMsg__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(custom_message__msg__SampleMsg);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    custom_message__msg__SampleMsg * data =
      (custom_message__msg__SampleMsg *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!custom_message__msg__SampleMsg__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          custom_message__msg__SampleMsg__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!custom_message__msg__SampleMsg__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
