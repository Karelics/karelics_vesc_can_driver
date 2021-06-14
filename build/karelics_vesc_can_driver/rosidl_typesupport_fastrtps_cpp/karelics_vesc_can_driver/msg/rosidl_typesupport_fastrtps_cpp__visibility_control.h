// generated from
// rosidl_typesupport_fastrtps_cpp/resource/rosidl_typesupport_fastrtps_cpp__visibility_control.h.in
// generated code does not contain a copyright notice

#ifndef KARELICS_VESC_CAN_DRIVER__MSG__ROSIDL_TYPESUPPORT_FASTRTPS_CPP__VISIBILITY_CONTROL_H_
#define KARELICS_VESC_CAN_DRIVER__MSG__ROSIDL_TYPESUPPORT_FASTRTPS_CPP__VISIBILITY_CONTROL_H_

#if __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_karelics_vesc_can_driver __attribute__ ((dllexport))
    #define ROSIDL_TYPESUPPORT_FASTRTPS_CPP_IMPORT_karelics_vesc_can_driver __attribute__ ((dllimport))
  #else
    #define ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_karelics_vesc_can_driver __declspec(dllexport)
    #define ROSIDL_TYPESUPPORT_FASTRTPS_CPP_IMPORT_karelics_vesc_can_driver __declspec(dllimport)
  #endif
  #ifdef ROSIDL_TYPESUPPORT_FASTRTPS_CPP_BUILDING_DLL_karelics_vesc_can_driver
    #define ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_karelics_vesc_can_driver ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_karelics_vesc_can_driver
  #else
    #define ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_karelics_vesc_can_driver ROSIDL_TYPESUPPORT_FASTRTPS_CPP_IMPORT_karelics_vesc_can_driver
  #endif
#else
  #define ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_karelics_vesc_can_driver __attribute__ ((visibility("default")))
  #define ROSIDL_TYPESUPPORT_FASTRTPS_CPP_IMPORT_karelics_vesc_can_driver
  #if __GNUC__ >= 4
    #define ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_karelics_vesc_can_driver __attribute__ ((visibility("default")))
  #else
    #define ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_karelics_vesc_can_driver
  #endif
#endif

#if __cplusplus
}
#endif

#endif  // KARELICS_VESC_CAN_DRIVER__MSG__ROSIDL_TYPESUPPORT_FASTRTPS_CPP__VISIBILITY_CONTROL_H_
