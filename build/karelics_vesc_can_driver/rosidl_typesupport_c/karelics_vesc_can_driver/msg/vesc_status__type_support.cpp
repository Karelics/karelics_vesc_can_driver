// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from karelics_vesc_can_driver:msg/VescStatus.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "karelics_vesc_can_driver/msg/rosidl_typesupport_c__visibility_control.h"
#include "karelics_vesc_can_driver/msg/detail/vesc_status__struct.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace karelics_vesc_can_driver
{

namespace msg
{

namespace rosidl_typesupport_c
{

typedef struct _VescStatus_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _VescStatus_type_support_ids_t;

static const _VescStatus_type_support_ids_t _VescStatus_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _VescStatus_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _VescStatus_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _VescStatus_type_support_symbol_names_t _VescStatus_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, karelics_vesc_can_driver, msg, VescStatus)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, karelics_vesc_can_driver, msg, VescStatus)),
  }
};

typedef struct _VescStatus_type_support_data_t
{
  void * data[2];
} _VescStatus_type_support_data_t;

static _VescStatus_type_support_data_t _VescStatus_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _VescStatus_message_typesupport_map = {
  2,
  "karelics_vesc_can_driver",
  &_VescStatus_message_typesupport_ids.typesupport_identifier[0],
  &_VescStatus_message_typesupport_symbol_names.symbol_name[0],
  &_VescStatus_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t VescStatus_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_VescStatus_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace msg

}  // namespace karelics_vesc_can_driver

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_C_EXPORT_karelics_vesc_can_driver
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, karelics_vesc_can_driver, msg, VescStatus)() {
  return &::karelics_vesc_can_driver::msg::rosidl_typesupport_c::VescStatus_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
