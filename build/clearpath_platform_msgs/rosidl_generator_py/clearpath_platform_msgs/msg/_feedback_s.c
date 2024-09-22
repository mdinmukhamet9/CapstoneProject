// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from clearpath_platform_msgs:msg/Feedback.idl
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
#include "clearpath_platform_msgs/msg/detail/feedback__struct.h"
#include "clearpath_platform_msgs/msg/detail/feedback__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

// Nested array functions includes
#include "clearpath_platform_msgs/msg/detail/drive_feedback__functions.h"
// end nested array functions include
ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);
bool clearpath_platform_msgs__msg__drive_feedback__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * clearpath_platform_msgs__msg__drive_feedback__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool clearpath_platform_msgs__msg__feedback__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[47];
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
    assert(strncmp("clearpath_platform_msgs.msg._feedback.Feedback", full_classname_dest, 46) == 0);
  }
  clearpath_platform_msgs__msg__Feedback * ros_message = _ros_message;
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
  {  // drivers
    PyObject * field = PyObject_GetAttrString(_pymsg, "drivers");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'drivers'");
    if (!seq_field) {
      Py_DECREF(field);
      return false;
    }
    Py_ssize_t size = 2;
    clearpath_platform_msgs__msg__DriveFeedback * dest = ros_message->drivers;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!clearpath_platform_msgs__msg__drive_feedback__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }
  {  // commanded_mode
    PyObject * field = PyObject_GetAttrString(_pymsg, "commanded_mode");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->commanded_mode = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // actual_mode
    PyObject * field = PyObject_GetAttrString(_pymsg, "actual_mode");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->actual_mode = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * clearpath_platform_msgs__msg__feedback__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Feedback */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("clearpath_platform_msgs.msg._feedback");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Feedback");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  clearpath_platform_msgs__msg__Feedback * ros_message = (clearpath_platform_msgs__msg__Feedback *)raw_ros_message;
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
  {  // drivers
    PyObject * field = NULL;
    size_t size = 2;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    clearpath_platform_msgs__msg__DriveFeedback * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->drivers[i]);
      PyObject * pyitem = clearpath_platform_msgs__msg__drive_feedback__convert_to_py(item);
      if (!pyitem) {
        Py_DECREF(field);
        return NULL;
      }
      int rc = PyList_SetItem(field, i, pyitem);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "drivers", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // commanded_mode
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->commanded_mode);
    {
      int rc = PyObject_SetAttrString(_pymessage, "commanded_mode", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // actual_mode
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->actual_mode);
    {
      int rc = PyObject_SetAttrString(_pymessage, "actual_mode", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
