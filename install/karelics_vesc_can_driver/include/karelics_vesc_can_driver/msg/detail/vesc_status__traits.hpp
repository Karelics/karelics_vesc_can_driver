// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from karelics_vesc_can_driver:msg/VescStatus.idl
// generated code does not contain a copyright notice

#ifndef KARELICS_VESC_CAN_DRIVER__MSG__DETAIL__VESC_STATUS__TRAITS_HPP_
#define KARELICS_VESC_CAN_DRIVER__MSG__DETAIL__VESC_STATUS__TRAITS_HPP_

#include "karelics_vesc_can_driver/msg/detail/vesc_status__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<karelics_vesc_can_driver::msg::VescStatus>()
{
  return "karelics_vesc_can_driver::msg::VescStatus";
}

template<>
inline const char * name<karelics_vesc_can_driver::msg::VescStatus>()
{
  return "karelics_vesc_can_driver/msg/VescStatus";
}

template<>
struct has_fixed_size<karelics_vesc_can_driver::msg::VescStatus>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<karelics_vesc_can_driver::msg::VescStatus>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<karelics_vesc_can_driver::msg::VescStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // KARELICS_VESC_CAN_DRIVER__MSG__DETAIL__VESC_STATUS__TRAITS_HPP_
