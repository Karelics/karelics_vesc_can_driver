// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from karelics_vesc_can_driver:msg/VescStatus.idl
// generated code does not contain a copyright notice

#ifndef KARELICS_VESC_CAN_DRIVER__MSG__DETAIL__VESC_STATUS__STRUCT_H_
#define KARELICS_VESC_CAN_DRIVER__MSG__DETAIL__VESC_STATUS__STRUCT_H_

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

// Struct defined in msg/VescStatus in the package karelics_vesc_can_driver.
typedef struct karelics_vesc_can_driver__msg__VescStatus
{
  std_msgs__msg__Header header;
  int32_t erpm;
  int32_t rpm;
  float duty_cycle;
  float current;
  float amp_hours;
  float amp_hours_charged;
  float watt_hours;
  float watt_hours_charged;
  float temp_fet;
  float temp_motor;
  float current_in;
  float pid_pos_now;
  float tacho_value;
  float v_in;
  float rotations;
} karelics_vesc_can_driver__msg__VescStatus;

// Struct for a sequence of karelics_vesc_can_driver__msg__VescStatus.
typedef struct karelics_vesc_can_driver__msg__VescStatus__Sequence
{
  karelics_vesc_can_driver__msg__VescStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} karelics_vesc_can_driver__msg__VescStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KARELICS_VESC_CAN_DRIVER__MSG__DETAIL__VESC_STATUS__STRUCT_H_
