// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from karelics_vesc_can_driver:msg/VescStatus.idl
// generated code does not contain a copyright notice

#ifndef KARELICS_VESC_CAN_DRIVER__MSG__DETAIL__VESC_STATUS__BUILDER_HPP_
#define KARELICS_VESC_CAN_DRIVER__MSG__DETAIL__VESC_STATUS__BUILDER_HPP_

#include "karelics_vesc_can_driver/msg/detail/vesc_status__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace karelics_vesc_can_driver
{

namespace msg
{

namespace builder
{

class Init_VescStatus_rotations
{
public:
  explicit Init_VescStatus_rotations(::karelics_vesc_can_driver::msg::VescStatus & msg)
  : msg_(msg)
  {}
  ::karelics_vesc_can_driver::msg::VescStatus rotations(::karelics_vesc_can_driver::msg::VescStatus::_rotations_type arg)
  {
    msg_.rotations = std::move(arg);
    return std::move(msg_);
  }

private:
  ::karelics_vesc_can_driver::msg::VescStatus msg_;
};

class Init_VescStatus_v_in
{
public:
  explicit Init_VescStatus_v_in(::karelics_vesc_can_driver::msg::VescStatus & msg)
  : msg_(msg)
  {}
  Init_VescStatus_rotations v_in(::karelics_vesc_can_driver::msg::VescStatus::_v_in_type arg)
  {
    msg_.v_in = std::move(arg);
    return Init_VescStatus_rotations(msg_);
  }

private:
  ::karelics_vesc_can_driver::msg::VescStatus msg_;
};

class Init_VescStatus_tacho_value
{
public:
  explicit Init_VescStatus_tacho_value(::karelics_vesc_can_driver::msg::VescStatus & msg)
  : msg_(msg)
  {}
  Init_VescStatus_v_in tacho_value(::karelics_vesc_can_driver::msg::VescStatus::_tacho_value_type arg)
  {
    msg_.tacho_value = std::move(arg);
    return Init_VescStatus_v_in(msg_);
  }

private:
  ::karelics_vesc_can_driver::msg::VescStatus msg_;
};

class Init_VescStatus_pid_pos_now
{
public:
  explicit Init_VescStatus_pid_pos_now(::karelics_vesc_can_driver::msg::VescStatus & msg)
  : msg_(msg)
  {}
  Init_VescStatus_tacho_value pid_pos_now(::karelics_vesc_can_driver::msg::VescStatus::_pid_pos_now_type arg)
  {
    msg_.pid_pos_now = std::move(arg);
    return Init_VescStatus_tacho_value(msg_);
  }

private:
  ::karelics_vesc_can_driver::msg::VescStatus msg_;
};

class Init_VescStatus_current_in
{
public:
  explicit Init_VescStatus_current_in(::karelics_vesc_can_driver::msg::VescStatus & msg)
  : msg_(msg)
  {}
  Init_VescStatus_pid_pos_now current_in(::karelics_vesc_can_driver::msg::VescStatus::_current_in_type arg)
  {
    msg_.current_in = std::move(arg);
    return Init_VescStatus_pid_pos_now(msg_);
  }

private:
  ::karelics_vesc_can_driver::msg::VescStatus msg_;
};

class Init_VescStatus_temp_motor
{
public:
  explicit Init_VescStatus_temp_motor(::karelics_vesc_can_driver::msg::VescStatus & msg)
  : msg_(msg)
  {}
  Init_VescStatus_current_in temp_motor(::karelics_vesc_can_driver::msg::VescStatus::_temp_motor_type arg)
  {
    msg_.temp_motor = std::move(arg);
    return Init_VescStatus_current_in(msg_);
  }

private:
  ::karelics_vesc_can_driver::msg::VescStatus msg_;
};

class Init_VescStatus_temp_fet
{
public:
  explicit Init_VescStatus_temp_fet(::karelics_vesc_can_driver::msg::VescStatus & msg)
  : msg_(msg)
  {}
  Init_VescStatus_temp_motor temp_fet(::karelics_vesc_can_driver::msg::VescStatus::_temp_fet_type arg)
  {
    msg_.temp_fet = std::move(arg);
    return Init_VescStatus_temp_motor(msg_);
  }

private:
  ::karelics_vesc_can_driver::msg::VescStatus msg_;
};

class Init_VescStatus_watt_hours_charged
{
public:
  explicit Init_VescStatus_watt_hours_charged(::karelics_vesc_can_driver::msg::VescStatus & msg)
  : msg_(msg)
  {}
  Init_VescStatus_temp_fet watt_hours_charged(::karelics_vesc_can_driver::msg::VescStatus::_watt_hours_charged_type arg)
  {
    msg_.watt_hours_charged = std::move(arg);
    return Init_VescStatus_temp_fet(msg_);
  }

private:
  ::karelics_vesc_can_driver::msg::VescStatus msg_;
};

class Init_VescStatus_watt_hours
{
public:
  explicit Init_VescStatus_watt_hours(::karelics_vesc_can_driver::msg::VescStatus & msg)
  : msg_(msg)
  {}
  Init_VescStatus_watt_hours_charged watt_hours(::karelics_vesc_can_driver::msg::VescStatus::_watt_hours_type arg)
  {
    msg_.watt_hours = std::move(arg);
    return Init_VescStatus_watt_hours_charged(msg_);
  }

private:
  ::karelics_vesc_can_driver::msg::VescStatus msg_;
};

class Init_VescStatus_amp_hours_charged
{
public:
  explicit Init_VescStatus_amp_hours_charged(::karelics_vesc_can_driver::msg::VescStatus & msg)
  : msg_(msg)
  {}
  Init_VescStatus_watt_hours amp_hours_charged(::karelics_vesc_can_driver::msg::VescStatus::_amp_hours_charged_type arg)
  {
    msg_.amp_hours_charged = std::move(arg);
    return Init_VescStatus_watt_hours(msg_);
  }

private:
  ::karelics_vesc_can_driver::msg::VescStatus msg_;
};

class Init_VescStatus_amp_hours
{
public:
  explicit Init_VescStatus_amp_hours(::karelics_vesc_can_driver::msg::VescStatus & msg)
  : msg_(msg)
  {}
  Init_VescStatus_amp_hours_charged amp_hours(::karelics_vesc_can_driver::msg::VescStatus::_amp_hours_type arg)
  {
    msg_.amp_hours = std::move(arg);
    return Init_VescStatus_amp_hours_charged(msg_);
  }

private:
  ::karelics_vesc_can_driver::msg::VescStatus msg_;
};

class Init_VescStatus_current
{
public:
  explicit Init_VescStatus_current(::karelics_vesc_can_driver::msg::VescStatus & msg)
  : msg_(msg)
  {}
  Init_VescStatus_amp_hours current(::karelics_vesc_can_driver::msg::VescStatus::_current_type arg)
  {
    msg_.current = std::move(arg);
    return Init_VescStatus_amp_hours(msg_);
  }

private:
  ::karelics_vesc_can_driver::msg::VescStatus msg_;
};

class Init_VescStatus_duty_cycle
{
public:
  explicit Init_VescStatus_duty_cycle(::karelics_vesc_can_driver::msg::VescStatus & msg)
  : msg_(msg)
  {}
  Init_VescStatus_current duty_cycle(::karelics_vesc_can_driver::msg::VescStatus::_duty_cycle_type arg)
  {
    msg_.duty_cycle = std::move(arg);
    return Init_VescStatus_current(msg_);
  }

private:
  ::karelics_vesc_can_driver::msg::VescStatus msg_;
};

class Init_VescStatus_rpm
{
public:
  explicit Init_VescStatus_rpm(::karelics_vesc_can_driver::msg::VescStatus & msg)
  : msg_(msg)
  {}
  Init_VescStatus_duty_cycle rpm(::karelics_vesc_can_driver::msg::VescStatus::_rpm_type arg)
  {
    msg_.rpm = std::move(arg);
    return Init_VescStatus_duty_cycle(msg_);
  }

private:
  ::karelics_vesc_can_driver::msg::VescStatus msg_;
};

class Init_VescStatus_erpm
{
public:
  explicit Init_VescStatus_erpm(::karelics_vesc_can_driver::msg::VescStatus & msg)
  : msg_(msg)
  {}
  Init_VescStatus_rpm erpm(::karelics_vesc_can_driver::msg::VescStatus::_erpm_type arg)
  {
    msg_.erpm = std::move(arg);
    return Init_VescStatus_rpm(msg_);
  }

private:
  ::karelics_vesc_can_driver::msg::VescStatus msg_;
};

class Init_VescStatus_header
{
public:
  Init_VescStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VescStatus_erpm header(::karelics_vesc_can_driver::msg::VescStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_VescStatus_erpm(msg_);
  }

private:
  ::karelics_vesc_can_driver::msg::VescStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::karelics_vesc_can_driver::msg::VescStatus>()
{
  return karelics_vesc_can_driver::msg::builder::Init_VescStatus_header();
}

}  // namespace karelics_vesc_can_driver

#endif  // KARELICS_VESC_CAN_DRIVER__MSG__DETAIL__VESC_STATUS__BUILDER_HPP_
