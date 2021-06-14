// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from karelics_vesc_can_driver:msg/VescStatus.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "karelics_vesc_can_driver/msg/detail/vesc_status__struct.h"
#include "karelics_vesc_can_driver/msg/detail/vesc_status__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool karelics_vesc_can_driver__msg__vesc_status__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[53];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("karelics_vesc_can_driver.msg._vesc_status.VescStatus", full_classname_dest, 52) == 0);
  }
  karelics_vesc_can_driver__msg__VescStatus * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // erpm
    PyObject * field = PyObject_GetAttrString(_pymsg, "erpm");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->erpm = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // rpm
    PyObject * field = PyObject_GetAttrString(_pymsg, "rpm");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->rpm = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // duty_cycle
    PyObject * field = PyObject_GetAttrString(_pymsg, "duty_cycle");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->duty_cycle = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // current
    PyObject * field = PyObject_GetAttrString(_pymsg, "current");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->current = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // amp_hours
    PyObject * field = PyObject_GetAttrString(_pymsg, "amp_hours");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->amp_hours = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // amp_hours_charged
    PyObject * field = PyObject_GetAttrString(_pymsg, "amp_hours_charged");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->amp_hours_charged = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // watt_hours
    PyObject * field = PyObject_GetAttrString(_pymsg, "watt_hours");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->watt_hours = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // watt_hours_charged
    PyObject * field = PyObject_GetAttrString(_pymsg, "watt_hours_charged");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->watt_hours_charged = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // temp_fet
    PyObject * field = PyObject_GetAttrString(_pymsg, "temp_fet");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->temp_fet = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // temp_motor
    PyObject * field = PyObject_GetAttrString(_pymsg, "temp_motor");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->temp_motor = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // current_in
    PyObject * field = PyObject_GetAttrString(_pymsg, "current_in");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->current_in = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // pid_pos_now
    PyObject * field = PyObject_GetAttrString(_pymsg, "pid_pos_now");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->pid_pos_now = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // tacho_value
    PyObject * field = PyObject_GetAttrString(_pymsg, "tacho_value");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->tacho_value = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // v_in
    PyObject * field = PyObject_GetAttrString(_pymsg, "v_in");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->v_in = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // rotations
    PyObject * field = PyObject_GetAttrString(_pymsg, "rotations");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->rotations = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * karelics_vesc_can_driver__msg__vesc_status__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of VescStatus */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("karelics_vesc_can_driver.msg._vesc_status");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "VescStatus");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  karelics_vesc_can_driver__msg__VescStatus * ros_message = (karelics_vesc_can_driver__msg__VescStatus *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // erpm
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->erpm);
    {
      int rc = PyObject_SetAttrString(_pymessage, "erpm", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rpm
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->rpm);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rpm", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // duty_cycle
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->duty_cycle);
    {
      int rc = PyObject_SetAttrString(_pymessage, "duty_cycle", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // current
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->current);
    {
      int rc = PyObject_SetAttrString(_pymessage, "current", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // amp_hours
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->amp_hours);
    {
      int rc = PyObject_SetAttrString(_pymessage, "amp_hours", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // amp_hours_charged
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->amp_hours_charged);
    {
      int rc = PyObject_SetAttrString(_pymessage, "amp_hours_charged", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // watt_hours
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->watt_hours);
    {
      int rc = PyObject_SetAttrString(_pymessage, "watt_hours", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // watt_hours_charged
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->watt_hours_charged);
    {
      int rc = PyObject_SetAttrString(_pymessage, "watt_hours_charged", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // temp_fet
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->temp_fet);
    {
      int rc = PyObject_SetAttrString(_pymessage, "temp_fet", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // temp_motor
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->temp_motor);
    {
      int rc = PyObject_SetAttrString(_pymessage, "temp_motor", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // current_in
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->current_in);
    {
      int rc = PyObject_SetAttrString(_pymessage, "current_in", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pid_pos_now
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->pid_pos_now);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pid_pos_now", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // tacho_value
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->tacho_value);
    {
      int rc = PyObject_SetAttrString(_pymessage, "tacho_value", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // v_in
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->v_in);
    {
      int rc = PyObject_SetAttrString(_pymessage, "v_in", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rotations
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->rotations);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rotations", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
