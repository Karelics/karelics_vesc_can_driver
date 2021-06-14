// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from karelics_vesc_can_driver:msg/VescStatus.idl
// generated code does not contain a copyright notice
#include "karelics_vesc_can_driver/msg/detail/vesc_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
karelics_vesc_can_driver__msg__VescStatus__init(karelics_vesc_can_driver__msg__VescStatus * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    karelics_vesc_can_driver__msg__VescStatus__fini(msg);
    return false;
  }
  // erpm
  // rpm
  // duty_cycle
  // current
  // amp_hours
  // amp_hours_charged
  // watt_hours
  // watt_hours_charged
  // temp_fet
  // temp_motor
  // current_in
  // pid_pos_now
  // tacho_value
  // v_in
  // rotations
  return true;
}

void
karelics_vesc_can_driver__msg__VescStatus__fini(karelics_vesc_can_driver__msg__VescStatus * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // erpm
  // rpm
  // duty_cycle
  // current
  // amp_hours
  // amp_hours_charged
  // watt_hours
  // watt_hours_charged
  // temp_fet
  // temp_motor
  // current_in
  // pid_pos_now
  // tacho_value
  // v_in
  // rotations
}

karelics_vesc_can_driver__msg__VescStatus *
karelics_vesc_can_driver__msg__VescStatus__create()
{
  karelics_vesc_can_driver__msg__VescStatus * msg = (karelics_vesc_can_driver__msg__VescStatus *)malloc(sizeof(karelics_vesc_can_driver__msg__VescStatus));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(karelics_vesc_can_driver__msg__VescStatus));
  bool success = karelics_vesc_can_driver__msg__VescStatus__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
karelics_vesc_can_driver__msg__VescStatus__destroy(karelics_vesc_can_driver__msg__VescStatus * msg)
{
  if (msg) {
    karelics_vesc_can_driver__msg__VescStatus__fini(msg);
  }
  free(msg);
}


bool
karelics_vesc_can_driver__msg__VescStatus__Sequence__init(karelics_vesc_can_driver__msg__VescStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  karelics_vesc_can_driver__msg__VescStatus * data = NULL;
  if (size) {
    data = (karelics_vesc_can_driver__msg__VescStatus *)calloc(size, sizeof(karelics_vesc_can_driver__msg__VescStatus));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = karelics_vesc_can_driver__msg__VescStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        karelics_vesc_can_driver__msg__VescStatus__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
karelics_vesc_can_driver__msg__VescStatus__Sequence__fini(karelics_vesc_can_driver__msg__VescStatus__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      karelics_vesc_can_driver__msg__VescStatus__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

karelics_vesc_can_driver__msg__VescStatus__Sequence *
karelics_vesc_can_driver__msg__VescStatus__Sequence__create(size_t size)
{
  karelics_vesc_can_driver__msg__VescStatus__Sequence * array = (karelics_vesc_can_driver__msg__VescStatus__Sequence *)malloc(sizeof(karelics_vesc_can_driver__msg__VescStatus__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = karelics_vesc_can_driver__msg__VescStatus__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
karelics_vesc_can_driver__msg__VescStatus__Sequence__destroy(karelics_vesc_can_driver__msg__VescStatus__Sequence * array)
{
  if (array) {
    karelics_vesc_can_driver__msg__VescStatus__Sequence__fini(array);
  }
  free(array);
}
