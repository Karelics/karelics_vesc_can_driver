// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from karelics_vesc_can_driver:msg/VescStatus.idl
// generated code does not contain a copyright notice
#include "karelics_vesc_can_driver/msg/detail/vesc_status__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "karelics_vesc_can_driver/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "karelics_vesc_can_driver/msg/detail/vesc_status__struct.h"
#include "karelics_vesc_can_driver/msg/detail/vesc_status__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "std_msgs/msg/detail/header__functions.h"  // header

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_karelics_vesc_can_driver
size_t get_serialized_size_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_karelics_vesc_can_driver
size_t max_serialized_size_std_msgs__msg__Header(
  bool & full_bounded,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_karelics_vesc_can_driver
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Header)();


using _VescStatus__ros_msg_type = karelics_vesc_can_driver__msg__VescStatus;

static bool _VescStatus__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _VescStatus__ros_msg_type * ros_message = static_cast<const _VescStatus__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->header, cdr))
    {
      return false;
    }
  }

  // Field name: erpm
  {
    cdr << ros_message->erpm;
  }

  // Field name: rpm
  {
    cdr << ros_message->rpm;
  }

  // Field name: duty_cycle
  {
    cdr << ros_message->duty_cycle;
  }

  // Field name: current
  {
    cdr << ros_message->current;
  }

  // Field name: amp_hours
  {
    cdr << ros_message->amp_hours;
  }

  // Field name: amp_hours_charged
  {
    cdr << ros_message->amp_hours_charged;
  }

  // Field name: watt_hours
  {
    cdr << ros_message->watt_hours;
  }

  // Field name: watt_hours_charged
  {
    cdr << ros_message->watt_hours_charged;
  }

  // Field name: temp_fet
  {
    cdr << ros_message->temp_fet;
  }

  // Field name: temp_motor
  {
    cdr << ros_message->temp_motor;
  }

  // Field name: current_in
  {
    cdr << ros_message->current_in;
  }

  // Field name: pid_pos_now
  {
    cdr << ros_message->pid_pos_now;
  }

  // Field name: tacho_value
  {
    cdr << ros_message->tacho_value;
  }

  // Field name: v_in
  {
    cdr << ros_message->v_in;
  }

  // Field name: rotations
  {
    cdr << ros_message->rotations;
  }

  return true;
}

static bool _VescStatus__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _VescStatus__ros_msg_type * ros_message = static_cast<_VescStatus__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->header))
    {
      return false;
    }
  }

  // Field name: erpm
  {
    cdr >> ros_message->erpm;
  }

  // Field name: rpm
  {
    cdr >> ros_message->rpm;
  }

  // Field name: duty_cycle
  {
    cdr >> ros_message->duty_cycle;
  }

  // Field name: current
  {
    cdr >> ros_message->current;
  }

  // Field name: amp_hours
  {
    cdr >> ros_message->amp_hours;
  }

  // Field name: amp_hours_charged
  {
    cdr >> ros_message->amp_hours_charged;
  }

  // Field name: watt_hours
  {
    cdr >> ros_message->watt_hours;
  }

  // Field name: watt_hours_charged
  {
    cdr >> ros_message->watt_hours_charged;
  }

  // Field name: temp_fet
  {
    cdr >> ros_message->temp_fet;
  }

  // Field name: temp_motor
  {
    cdr >> ros_message->temp_motor;
  }

  // Field name: current_in
  {
    cdr >> ros_message->current_in;
  }

  // Field name: pid_pos_now
  {
    cdr >> ros_message->pid_pos_now;
  }

  // Field name: tacho_value
  {
    cdr >> ros_message->tacho_value;
  }

  // Field name: v_in
  {
    cdr >> ros_message->v_in;
  }

  // Field name: rotations
  {
    cdr >> ros_message->rotations;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_karelics_vesc_can_driver
size_t get_serialized_size_karelics_vesc_can_driver__msg__VescStatus(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _VescStatus__ros_msg_type * ros_message = static_cast<const _VescStatus__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name header

  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);
  // field.name erpm
  {
    size_t item_size = sizeof(ros_message->erpm);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name rpm
  {
    size_t item_size = sizeof(ros_message->rpm);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name duty_cycle
  {
    size_t item_size = sizeof(ros_message->duty_cycle);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name current
  {
    size_t item_size = sizeof(ros_message->current);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name amp_hours
  {
    size_t item_size = sizeof(ros_message->amp_hours);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name amp_hours_charged
  {
    size_t item_size = sizeof(ros_message->amp_hours_charged);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name watt_hours
  {
    size_t item_size = sizeof(ros_message->watt_hours);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name watt_hours_charged
  {
    size_t item_size = sizeof(ros_message->watt_hours_charged);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name temp_fet
  {
    size_t item_size = sizeof(ros_message->temp_fet);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name temp_motor
  {
    size_t item_size = sizeof(ros_message->temp_motor);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name current_in
  {
    size_t item_size = sizeof(ros_message->current_in);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name pid_pos_now
  {
    size_t item_size = sizeof(ros_message->pid_pos_now);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name tacho_value
  {
    size_t item_size = sizeof(ros_message->tacho_value);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name v_in
  {
    size_t item_size = sizeof(ros_message->v_in);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name rotations
  {
    size_t item_size = sizeof(ros_message->rotations);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _VescStatus__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_karelics_vesc_can_driver__msg__VescStatus(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_karelics_vesc_can_driver
size_t max_serialized_size_karelics_vesc_can_driver__msg__VescStatus(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: header
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_std_msgs__msg__Header(
        full_bounded, current_alignment);
    }
  }
  // member: erpm
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: rpm
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: duty_cycle
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: current
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: amp_hours
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: amp_hours_charged
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: watt_hours
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: watt_hours_charged
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: temp_fet
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: temp_motor
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: current_in
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: pid_pos_now
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: tacho_value
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: v_in
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: rotations
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _VescStatus__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_karelics_vesc_can_driver__msg__VescStatus(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_VescStatus = {
  "karelics_vesc_can_driver::msg",
  "VescStatus",
  _VescStatus__cdr_serialize,
  _VescStatus__cdr_deserialize,
  _VescStatus__get_serialized_size,
  _VescStatus__max_serialized_size
};

static rosidl_message_type_support_t _VescStatus__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_VescStatus,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, karelics_vesc_can_driver, msg, VescStatus)() {
  return &_VescStatus__type_support;
}

#if defined(__cplusplus)
}
#endif
