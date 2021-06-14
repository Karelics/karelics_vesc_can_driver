// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from karelics_vesc_can_driver:msg/VescStatus.idl
// generated code does not contain a copyright notice
#include "karelics_vesc_can_driver/msg/detail/vesc_status__rosidl_typesupport_fastrtps_cpp.hpp"
#include "karelics_vesc_can_driver/msg/detail/vesc_status__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace std_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const std_msgs::msg::Header &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  std_msgs::msg::Header &);
size_t get_serialized_size(
  const std_msgs::msg::Header &,
  size_t current_alignment);
size_t
max_serialized_size_Header(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace std_msgs


namespace karelics_vesc_can_driver
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_karelics_vesc_can_driver
cdr_serialize(
  const karelics_vesc_can_driver::msg::VescStatus & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.header,
    cdr);
  // Member: erpm
  cdr << ros_message.erpm;
  // Member: rpm
  cdr << ros_message.rpm;
  // Member: duty_cycle
  cdr << ros_message.duty_cycle;
  // Member: current
  cdr << ros_message.current;
  // Member: amp_hours
  cdr << ros_message.amp_hours;
  // Member: amp_hours_charged
  cdr << ros_message.amp_hours_charged;
  // Member: watt_hours
  cdr << ros_message.watt_hours;
  // Member: watt_hours_charged
  cdr << ros_message.watt_hours_charged;
  // Member: temp_fet
  cdr << ros_message.temp_fet;
  // Member: temp_motor
  cdr << ros_message.temp_motor;
  // Member: current_in
  cdr << ros_message.current_in;
  // Member: pid_pos_now
  cdr << ros_message.pid_pos_now;
  // Member: tacho_value
  cdr << ros_message.tacho_value;
  // Member: v_in
  cdr << ros_message.v_in;
  // Member: rotations
  cdr << ros_message.rotations;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_karelics_vesc_can_driver
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  karelics_vesc_can_driver::msg::VescStatus & ros_message)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.header);

  // Member: erpm
  cdr >> ros_message.erpm;

  // Member: rpm
  cdr >> ros_message.rpm;

  // Member: duty_cycle
  cdr >> ros_message.duty_cycle;

  // Member: current
  cdr >> ros_message.current;

  // Member: amp_hours
  cdr >> ros_message.amp_hours;

  // Member: amp_hours_charged
  cdr >> ros_message.amp_hours_charged;

  // Member: watt_hours
  cdr >> ros_message.watt_hours;

  // Member: watt_hours_charged
  cdr >> ros_message.watt_hours_charged;

  // Member: temp_fet
  cdr >> ros_message.temp_fet;

  // Member: temp_motor
  cdr >> ros_message.temp_motor;

  // Member: current_in
  cdr >> ros_message.current_in;

  // Member: pid_pos_now
  cdr >> ros_message.pid_pos_now;

  // Member: tacho_value
  cdr >> ros_message.tacho_value;

  // Member: v_in
  cdr >> ros_message.v_in;

  // Member: rotations
  cdr >> ros_message.rotations;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_karelics_vesc_can_driver
get_serialized_size(
  const karelics_vesc_can_driver::msg::VescStatus & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: header

  current_alignment +=
    std_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.header, current_alignment);
  // Member: erpm
  {
    size_t item_size = sizeof(ros_message.erpm);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: rpm
  {
    size_t item_size = sizeof(ros_message.rpm);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: duty_cycle
  {
    size_t item_size = sizeof(ros_message.duty_cycle);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: current
  {
    size_t item_size = sizeof(ros_message.current);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: amp_hours
  {
    size_t item_size = sizeof(ros_message.amp_hours);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: amp_hours_charged
  {
    size_t item_size = sizeof(ros_message.amp_hours_charged);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: watt_hours
  {
    size_t item_size = sizeof(ros_message.watt_hours);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: watt_hours_charged
  {
    size_t item_size = sizeof(ros_message.watt_hours_charged);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: temp_fet
  {
    size_t item_size = sizeof(ros_message.temp_fet);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: temp_motor
  {
    size_t item_size = sizeof(ros_message.temp_motor);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: current_in
  {
    size_t item_size = sizeof(ros_message.current_in);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: pid_pos_now
  {
    size_t item_size = sizeof(ros_message.pid_pos_now);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: tacho_value
  {
    size_t item_size = sizeof(ros_message.tacho_value);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: v_in
  {
    size_t item_size = sizeof(ros_message.v_in);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: rotations
  {
    size_t item_size = sizeof(ros_message.rotations);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_karelics_vesc_can_driver
max_serialized_size_VescStatus(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: header
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        std_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Header(
        full_bounded, current_alignment);
    }
  }

  // Member: erpm
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: rpm
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: duty_cycle
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: current
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: amp_hours
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: amp_hours_charged
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: watt_hours
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: watt_hours_charged
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: temp_fet
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: temp_motor
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: current_in
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: pid_pos_now
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: tacho_value
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: v_in
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: rotations
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static bool _VescStatus__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const karelics_vesc_can_driver::msg::VescStatus *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _VescStatus__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<karelics_vesc_can_driver::msg::VescStatus *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _VescStatus__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const karelics_vesc_can_driver::msg::VescStatus *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _VescStatus__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_VescStatus(full_bounded, 0);
}

static message_type_support_callbacks_t _VescStatus__callbacks = {
  "karelics_vesc_can_driver::msg",
  "VescStatus",
  _VescStatus__cdr_serialize,
  _VescStatus__cdr_deserialize,
  _VescStatus__get_serialized_size,
  _VescStatus__max_serialized_size
};

static rosidl_message_type_support_t _VescStatus__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_VescStatus__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace karelics_vesc_can_driver

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_karelics_vesc_can_driver
const rosidl_message_type_support_t *
get_message_type_support_handle<karelics_vesc_can_driver::msg::VescStatus>()
{
  return &karelics_vesc_can_driver::msg::typesupport_fastrtps_cpp::_VescStatus__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, karelics_vesc_can_driver, msg, VescStatus)() {
  return &karelics_vesc_can_driver::msg::typesupport_fastrtps_cpp::_VescStatus__handle;
}

#ifdef __cplusplus
}
#endif
