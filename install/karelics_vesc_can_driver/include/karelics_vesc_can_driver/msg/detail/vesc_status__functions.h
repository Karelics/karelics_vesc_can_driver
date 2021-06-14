// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from karelics_vesc_can_driver:msg/VescStatus.idl
// generated code does not contain a copyright notice

#ifndef KARELICS_VESC_CAN_DRIVER__MSG__DETAIL__VESC_STATUS__FUNCTIONS_H_
#define KARELICS_VESC_CAN_DRIVER__MSG__DETAIL__VESC_STATUS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "karelics_vesc_can_driver/msg/rosidl_generator_c__visibility_control.h"

#include "karelics_vesc_can_driver/msg/detail/vesc_status__struct.h"

/// Initialize msg/VescStatus message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * karelics_vesc_can_driver__msg__VescStatus
 * )) before or use
 * karelics_vesc_can_driver__msg__VescStatus__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_karelics_vesc_can_driver
bool
karelics_vesc_can_driver__msg__VescStatus__init(karelics_vesc_can_driver__msg__VescStatus * msg);

/// Finalize msg/VescStatus message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_karelics_vesc_can_driver
void
karelics_vesc_can_driver__msg__VescStatus__fini(karelics_vesc_can_driver__msg__VescStatus * msg);

/// Create msg/VescStatus message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * karelics_vesc_can_driver__msg__VescStatus__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_karelics_vesc_can_driver
karelics_vesc_can_driver__msg__VescStatus *
karelics_vesc_can_driver__msg__VescStatus__create();

/// Destroy msg/VescStatus message.
/**
 * It calls
 * karelics_vesc_can_driver__msg__VescStatus__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_karelics_vesc_can_driver
void
karelics_vesc_can_driver__msg__VescStatus__destroy(karelics_vesc_can_driver__msg__VescStatus * msg);


/// Initialize array of msg/VescStatus messages.
/**
 * It allocates the memory for the number of elements and calls
 * karelics_vesc_can_driver__msg__VescStatus__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_karelics_vesc_can_driver
bool
karelics_vesc_can_driver__msg__VescStatus__Sequence__init(karelics_vesc_can_driver__msg__VescStatus__Sequence * array, size_t size);

/// Finalize array of msg/VescStatus messages.
/**
 * It calls
 * karelics_vesc_can_driver__msg__VescStatus__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_karelics_vesc_can_driver
void
karelics_vesc_can_driver__msg__VescStatus__Sequence__fini(karelics_vesc_can_driver__msg__VescStatus__Sequence * array);

/// Create array of msg/VescStatus messages.
/**
 * It allocates the memory for the array and calls
 * karelics_vesc_can_driver__msg__VescStatus__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_karelics_vesc_can_driver
karelics_vesc_can_driver__msg__VescStatus__Sequence *
karelics_vesc_can_driver__msg__VescStatus__Sequence__create(size_t size);

/// Destroy array of msg/VescStatus messages.
/**
 * It calls
 * karelics_vesc_can_driver__msg__VescStatus__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_karelics_vesc_can_driver
void
karelics_vesc_can_driver__msg__VescStatus__Sequence__destroy(karelics_vesc_can_driver__msg__VescStatus__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // KARELICS_VESC_CAN_DRIVER__MSG__DETAIL__VESC_STATUS__FUNCTIONS_H_
