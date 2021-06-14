// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from karelics_vesc_can_driver:msg/VescStatus.idl
// generated code does not contain a copyright notice

#ifndef KARELICS_VESC_CAN_DRIVER__MSG__DETAIL__VESC_STATUS__STRUCT_HPP_
#define KARELICS_VESC_CAN_DRIVER__MSG__DETAIL__VESC_STATUS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__karelics_vesc_can_driver__msg__VescStatus __attribute__((deprecated))
#else
# define DEPRECATED__karelics_vesc_can_driver__msg__VescStatus __declspec(deprecated)
#endif

namespace karelics_vesc_can_driver
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct VescStatus_
{
  using Type = VescStatus_<ContainerAllocator>;

  explicit VescStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->erpm = 0l;
      this->rpm = 0l;
      this->duty_cycle = 0.0f;
      this->current = 0.0f;
      this->amp_hours = 0.0f;
      this->amp_hours_charged = 0.0f;
      this->watt_hours = 0.0f;
      this->watt_hours_charged = 0.0f;
      this->temp_fet = 0.0f;
      this->temp_motor = 0.0f;
      this->current_in = 0.0f;
      this->pid_pos_now = 0.0f;
      this->tacho_value = 0.0f;
      this->v_in = 0.0f;
      this->rotations = 0.0f;
    }
  }

  explicit VescStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->erpm = 0l;
      this->rpm = 0l;
      this->duty_cycle = 0.0f;
      this->current = 0.0f;
      this->amp_hours = 0.0f;
      this->amp_hours_charged = 0.0f;
      this->watt_hours = 0.0f;
      this->watt_hours_charged = 0.0f;
      this->temp_fet = 0.0f;
      this->temp_motor = 0.0f;
      this->current_in = 0.0f;
      this->pid_pos_now = 0.0f;
      this->tacho_value = 0.0f;
      this->v_in = 0.0f;
      this->rotations = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _erpm_type =
    int32_t;
  _erpm_type erpm;
  using _rpm_type =
    int32_t;
  _rpm_type rpm;
  using _duty_cycle_type =
    float;
  _duty_cycle_type duty_cycle;
  using _current_type =
    float;
  _current_type current;
  using _amp_hours_type =
    float;
  _amp_hours_type amp_hours;
  using _amp_hours_charged_type =
    float;
  _amp_hours_charged_type amp_hours_charged;
  using _watt_hours_type =
    float;
  _watt_hours_type watt_hours;
  using _watt_hours_charged_type =
    float;
  _watt_hours_charged_type watt_hours_charged;
  using _temp_fet_type =
    float;
  _temp_fet_type temp_fet;
  using _temp_motor_type =
    float;
  _temp_motor_type temp_motor;
  using _current_in_type =
    float;
  _current_in_type current_in;
  using _pid_pos_now_type =
    float;
  _pid_pos_now_type pid_pos_now;
  using _tacho_value_type =
    float;
  _tacho_value_type tacho_value;
  using _v_in_type =
    float;
  _v_in_type v_in;
  using _rotations_type =
    float;
  _rotations_type rotations;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__erpm(
    const int32_t & _arg)
  {
    this->erpm = _arg;
    return *this;
  }
  Type & set__rpm(
    const int32_t & _arg)
  {
    this->rpm = _arg;
    return *this;
  }
  Type & set__duty_cycle(
    const float & _arg)
  {
    this->duty_cycle = _arg;
    return *this;
  }
  Type & set__current(
    const float & _arg)
  {
    this->current = _arg;
    return *this;
  }
  Type & set__amp_hours(
    const float & _arg)
  {
    this->amp_hours = _arg;
    return *this;
  }
  Type & set__amp_hours_charged(
    const float & _arg)
  {
    this->amp_hours_charged = _arg;
    return *this;
  }
  Type & set__watt_hours(
    const float & _arg)
  {
    this->watt_hours = _arg;
    return *this;
  }
  Type & set__watt_hours_charged(
    const float & _arg)
  {
    this->watt_hours_charged = _arg;
    return *this;
  }
  Type & set__temp_fet(
    const float & _arg)
  {
    this->temp_fet = _arg;
    return *this;
  }
  Type & set__temp_motor(
    const float & _arg)
  {
    this->temp_motor = _arg;
    return *this;
  }
  Type & set__current_in(
    const float & _arg)
  {
    this->current_in = _arg;
    return *this;
  }
  Type & set__pid_pos_now(
    const float & _arg)
  {
    this->pid_pos_now = _arg;
    return *this;
  }
  Type & set__tacho_value(
    const float & _arg)
  {
    this->tacho_value = _arg;
    return *this;
  }
  Type & set__v_in(
    const float & _arg)
  {
    this->v_in = _arg;
    return *this;
  }
  Type & set__rotations(
    const float & _arg)
  {
    this->rotations = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    karelics_vesc_can_driver::msg::VescStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const karelics_vesc_can_driver::msg::VescStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<karelics_vesc_can_driver::msg::VescStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<karelics_vesc_can_driver::msg::VescStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      karelics_vesc_can_driver::msg::VescStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<karelics_vesc_can_driver::msg::VescStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      karelics_vesc_can_driver::msg::VescStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<karelics_vesc_can_driver::msg::VescStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<karelics_vesc_can_driver::msg::VescStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<karelics_vesc_can_driver::msg::VescStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__karelics_vesc_can_driver__msg__VescStatus
    std::shared_ptr<karelics_vesc_can_driver::msg::VescStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__karelics_vesc_can_driver__msg__VescStatus
    std::shared_ptr<karelics_vesc_can_driver::msg::VescStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VescStatus_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->erpm != other.erpm) {
      return false;
    }
    if (this->rpm != other.rpm) {
      return false;
    }
    if (this->duty_cycle != other.duty_cycle) {
      return false;
    }
    if (this->current != other.current) {
      return false;
    }
    if (this->amp_hours != other.amp_hours) {
      return false;
    }
    if (this->amp_hours_charged != other.amp_hours_charged) {
      return false;
    }
    if (this->watt_hours != other.watt_hours) {
      return false;
    }
    if (this->watt_hours_charged != other.watt_hours_charged) {
      return false;
    }
    if (this->temp_fet != other.temp_fet) {
      return false;
    }
    if (this->temp_motor != other.temp_motor) {
      return false;
    }
    if (this->current_in != other.current_in) {
      return false;
    }
    if (this->pid_pos_now != other.pid_pos_now) {
      return false;
    }
    if (this->tacho_value != other.tacho_value) {
      return false;
    }
    if (this->v_in != other.v_in) {
      return false;
    }
    if (this->rotations != other.rotations) {
      return false;
    }
    return true;
  }
  bool operator!=(const VescStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VescStatus_

// alias to use template instance with default allocator
using VescStatus =
  karelics_vesc_can_driver::msg::VescStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace karelics_vesc_can_driver

#endif  // KARELICS_VESC_CAN_DRIVER__MSG__DETAIL__VESC_STATUS__STRUCT_HPP_
